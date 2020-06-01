/****************************************************************************
*
*   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file gyro_calibration.cpp
 *
 * Gyroscope calibration routine
 */

#include <px4_platform_common/px4_config.h>
#include "gyro_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>

static constexpr char sensor_name[] {"gyro"};
static constexpr unsigned MAX_GYROS = 3;

using matrix::Vector3f;

/// Data passed to calibration worker routine
struct gyro_worker_data_t {
	orb_advert_t *mavlink_log_pub{nullptr};
	int32_t device_id[MAX_GYROS] {};
	Vector3f offset[MAX_GYROS] {};
	Vector3f last_sample_0{};
};

static calibrate_return gyro_calibration_worker(int cancel_sub, gyro_worker_data_t &worker_data)
{
	unsigned calibration_counter[MAX_GYROS] {};
	static constexpr unsigned calibration_count = 250;
	unsigned poll_errcount = 0;

	uORB::Subscription sensor_correction_sub{ORB_ID(sensor_correction)};
	sensor_correction_s sensor_correction{}; /**< sensor thermal corrections */

	uORB::SubscriptionBlocking<sensor_gyro_s> gyro_sub[MAX_GYROS] {
		{ORB_ID(sensor_gyro), 0, 0},
		{ORB_ID(sensor_gyro), 0, 1},
		{ORB_ID(sensor_gyro), 0, 2},
	};

	worker_data.last_sample_0.zero();

	/* use slowest gyro to pace, but count correctly per-gyro for statistics */
	unsigned slow_count = 0;

	while (slow_count < calibration_count) {
		if (calibrate_cancel_check(worker_data.mavlink_log_pub, cancel_sub)) {
			return calibrate_return_cancelled;
		}

		if (gyro_sub[0].updatedBlocking(100000)) {
			unsigned update_count = calibration_count;

			for (unsigned s = 0; s < MAX_GYROS; s++) {
				if (calibration_counter[s] >= calibration_count) {
					// Skip if instance has enough samples
					continue;
				}

				sensor_gyro_s gyro_report;

				if (gyro_sub[s].update(&gyro_report)) {

					// fetch optional thermal offset corrections in sensor/board frame
					Vector3f offset{0, 0, 0};
					sensor_correction_sub.update(&sensor_correction);

					if (sensor_correction.timestamp > 0 && gyro_report.device_id != 0) {
						for (uint8_t i = 0; i < MAX_GYROS; i++) {
							if (sensor_correction.gyro_device_ids[i] == gyro_report.device_id) {
								switch (i) {
								case 0:
									offset = Vector3f{sensor_correction.gyro_offset_0};
									break;
								case 1:
									offset = Vector3f{sensor_correction.gyro_offset_1};
									break;
								case 2:
									offset = Vector3f{sensor_correction.gyro_offset_2};
									break;
								}
							}
						}
					}

					worker_data.offset[s] += Vector3f{gyro_report.x, gyro_report.y, gyro_report.z} - offset;
					calibration_counter[s]++;

					if (s == 0) {
						worker_data.last_sample_0 = Vector3f{gyro_report.x, gyro_report.y, gyro_report.z};
					}
				}

				// Maintain the sample count of the slowest sensor
				if (calibration_counter[s] && calibration_counter[s] < update_count) {
					update_count = calibration_counter[s];
				}
			}

			if (update_count % (calibration_count / 20) == 0) {
				calibration_log_info(worker_data.mavlink_log_pub, CAL_QGC_PROGRESS_MSG, (update_count * 100) / calibration_count);
			}

			// Propagate out the slowest sensor's count
			if (slow_count < update_count) {
				slow_count = update_count;
			}

		} else {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {
			calibration_log_critical(worker_data.mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			return calibrate_return_error;
		}
	}

	for (unsigned s = 0; s < MAX_GYROS; s++) {
		if (worker_data.device_id[s] != 0 && calibration_counter[s] < calibration_count / 2) {
			calibration_log_critical(worker_data.mavlink_log_pub, "ERROR: missing data, sensor %d", s)
			return calibrate_return_error;
		}

		worker_data.offset[s] /= calibration_counter[s];
	}

	return calibrate_return_ok;
}

int do_gyro_calibration(orb_advert_t *mavlink_log_pub)
{
	int res = PX4_OK;

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	gyro_worker_data_t worker_data{};
	worker_data.mavlink_log_pub = mavlink_log_pub;

	enum ORB_PRIO device_prio_max = ORB_PRIO_UNINITIALIZED;
	int32_t device_id_primary = 0;

	// We should not try to subscribe if the topic doesn't actually exist and can be counted.
	const unsigned orb_gyro_count = orb_group_count(ORB_ID(sensor_gyro));

	// Warn that we will not calibrate more than MAX_GYROS gyroscopes
	if (orb_gyro_count > MAX_GYROS) {
		calibration_log_critical(mavlink_log_pub, "Detected %u gyros, but will calibrate only %u", orb_gyro_count, MAX_GYROS);
	}

	for (uint8_t cur_gyro = 0; cur_gyro < orb_gyro_count && cur_gyro < MAX_GYROS; cur_gyro++) {

		uORB::Subscription gyro_sensor_sub{ORB_ID(sensor_gyro), cur_gyro};
		sensor_gyro_s report{};
		gyro_sensor_sub.copy(&report);

		worker_data.device_id[cur_gyro] = report.device_id;

		if (worker_data.device_id[cur_gyro] != 0) {
			// Get priority
			enum ORB_PRIO prio = gyro_sensor_sub.get_priority();

			if (prio > device_prio_max) {
				device_prio_max = prio;
				device_id_primary = worker_data.device_id[cur_gyro];
			}

		} else {
			calibration_log_critical(mavlink_log_pub, "Gyro #%u no device id, abort", cur_gyro);
		}
	}

	int cancel_sub = calibrate_cancel_subscribe();

	unsigned try_count = 0;
	unsigned max_tries = 20;
	res = PX4_ERROR;

	do {
		// Calibrate gyro and ensure user didn't move
		calibrate_return cal_return = gyro_calibration_worker(cancel_sub, worker_data);

		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already sent, we are done here
			res = PX4_ERROR;
			break;

		} else if (cal_return == calibrate_return_error) {
			res = PX4_ERROR;

		} else {
			/* check offsets */
			Vector3f diff = worker_data.last_sample_0 - worker_data.offset[0];

			/* maximum allowable calibration error */
			const float maxoff = math::radians(0.4f);

			if (!PX4_ISFINITE(worker_data.offset[0](0))
			    || !PX4_ISFINITE(worker_data.offset[0](1))
			    || !PX4_ISFINITE(worker_data.offset[0](2)) ||
			    fabsf(diff(0)) > maxoff || fabsf(diff(1)) > maxoff || fabsf(diff(2)) > maxoff) {

				calibration_log_critical(mavlink_log_pub, "motion, retrying..");
				res = PX4_ERROR;

			} else {
				res = PX4_OK;
			}
		}

		try_count++;

	} while (res == PX4_ERROR && try_count <= max_tries);

	if (try_count >= max_tries) {
		calibration_log_critical(mavlink_log_pub, "ERROR: Motion during calibration");
		res = PX4_ERROR;
	}

	calibrate_cancel_unsubscribe(cancel_sub);

	if (res == PX4_OK) {

		/* set offset parameters to new values */
		bool failed = (PX4_OK != param_set_no_notification(param_find("CAL_GYRO_PRIME"), &device_id_primary));

		for (unsigned uorb_index = 0; uorb_index < MAX_GYROS; uorb_index++) {

			char str[30] {};

			if (uorb_index < orb_gyro_count) {
				float x_offset = worker_data.offset[uorb_index](0);
				sprintf(str, "CAL_GYRO%u_XOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &x_offset));

				float y_offset = worker_data.offset[uorb_index](1);
				sprintf(str, "CAL_GYRO%u_YOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &y_offset));

				float z_offset = worker_data.offset[uorb_index](2);
				sprintf(str, "CAL_GYRO%u_ZOFF", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &z_offset));

				int32_t device_id = worker_data.device_id[uorb_index];
				sprintf(str, "CAL_GYRO%u_ID", uorb_index);
				failed |= (PX4_OK != param_set_no_notification(param_find(str), &device_id));

			} else {
				// reset unused calibration offsets
				sprintf(str, "CAL_GYRO%u_XOFF", uorb_index);
				param_reset(param_find(str));
				sprintf(str, "CAL_GYRO%u_YOFF", uorb_index);
				param_reset(param_find(str));
				sprintf(str, "CAL_GYRO%u_ZOFF", uorb_index);
				param_reset(param_find(str));

				// reset unused calibration device ID
				sprintf(str, "CAL_GYRO%u_ID", uorb_index);
				param_reset(param_find(str));
			}
		}

		if (failed) {
			calibration_log_critical(mavlink_log_pub, "ERROR: failed to set offset params");
			res = PX4_ERROR;
		}
	}

	param_notify_changes();

	/* if there is a any preflight-check system response, let the barrage of messages through */
	px4_usleep(200000);

	if (res == PX4_OK) {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);

	} else {
		calibration_log_info(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
	}

	/* give this message enough time to propagate */
	px4_usleep(600000);

	return res;
}
