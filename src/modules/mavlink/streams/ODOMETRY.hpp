/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <uORB/topics/vehicle_odometry.h>

class MavlinkStreamOdometry : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOdometry(mavlink); }

	static constexpr const char *get_name_static() { return "ODOMETRY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ODOMETRY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_mavlink->odometry_loopback_enabled()) {
			return _vodom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;

		} else {
			return _odom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
		}
	}

private:
	explicit MavlinkStreamOdometry(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _odom_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _vodom_sub{ORB_ID(vehicle_visual_odometry)};

	bool send() override
	{
		vehicle_odometry_s odom;
		// check if it is to send visual odometry loopback or not
		bool odom_updated = false;

		mavlink_odometry_t msg{};

		if (_mavlink->odometry_loopback_enabled()) {
			odom_updated = _vodom_sub.update(&odom);

			// source: external vision system
			msg.estimator_type = MAV_ESTIMATOR_TYPE_VISION;

		} else {
			odom_updated = _odom_sub.update(&odom);

			// source: PX4 estimator
			msg.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;
		}

		if (odom_updated) {
			msg.time_usec = odom.timestamp_sample;

			// set the frame_id according to the local frame of the data
			switch (odom.local_frame) {
			case vehicle_odometry_s::LOCAL_FRAME_NED:
				msg.frame_id = MAV_FRAME_LOCAL_NED;
				break;

			case vehicle_odometry_s::LOCAL_FRAME_FRD:
				msg.frame_id = MAV_FRAME_LOCAL_FRD;
				break;
			}

			switch (odom.velocity_frame) {
			case vehicle_odometry_s::LOCAL_FRAME_NED:
				msg.child_frame_id = MAV_FRAME_LOCAL_NED;
				break;

			case vehicle_odometry_s::LOCAL_FRAME_FRD:
				msg.child_frame_id = MAV_FRAME_LOCAL_FRD;
				break;

			case vehicle_odometry_s::BODY_FRAME_FRD:
				msg.child_frame_id = MAV_FRAME_BODY_FRD;
				break;
			}

			msg.x = odom.position[0];
			msg.y = odom.position[1];
			msg.z = odom.position[2];

			msg.q[0] = odom.q[0];
			msg.q[1] = odom.q[1];
			msg.q[2] = odom.q[2];
			msg.q[3] = odom.q[3];

			msg.vx = odom.velocity[0];
			msg.vy = odom.velocity[1];
			msg.vz = odom.velocity[2];

			// Current body rates
			msg.rollspeed  = odom.angular_velocity[0];
			msg.pitchspeed = odom.angular_velocity[1];
			msg.yawspeed   = odom.angular_velocity[2];

			// pose_covariance
			//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle
			//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.)
			for (auto &pc : msg.pose_covariance) {
				pc = NAN;
			}

			msg.pose_covariance[0]  = odom.position_covariance[odom.POSITION_COVARIANCE_X_VAR];  // X  row 0, col 0
			msg.pose_covariance[1]  = odom.position_covariance[odom.POSITION_COVARIANCE_XY_COV]; // XY row 0, col 1
			msg.pose_covariance[2]  = odom.position_covariance[odom.POSITION_COVARIANCE_XZ_COV]; // XZ row 0, col 2
			msg.pose_covariance[6]  = odom.position_covariance[odom.POSITION_COVARIANCE_Y_VAR];  // Y  row 1, col 1
			msg.pose_covariance[7]  = odom.position_covariance[odom.POSITION_COVARIANCE_YZ_COV]; // YZ row 1, col 2
			msg.pose_covariance[11] = odom.position_covariance[odom.POSITION_COVARIANCE_Z_VAR];  // Z  row 2, col 2

			msg.pose_covariance[15] = odom.orientation_covariance[odom.ORIENTATION_COVARIANCE_R_VAR];  // R  row 3, col 3
			msg.pose_covariance[16] = odom.orientation_covariance[odom.ORIENTATION_COVARIANCE_RP_COV]; // RP row 3, col 4
			msg.pose_covariance[17] = odom.orientation_covariance[odom.ORIENTATION_COVARIANCE_RY_COV]; // RY row 3, col 5
			msg.pose_covariance[18] = odom.orientation_covariance[odom.ORIENTATION_COVARIANCE_P_VAR];  // P  row 4, col 4
			msg.pose_covariance[19] = odom.orientation_covariance[odom.ORIENTATION_COVARIANCE_PY_COV]; // PY row 4, col 5
			msg.pose_covariance[20] = odom.orientation_covariance[odom.ORIENTATION_COVARIANCE_Y_VAR];  // Y  row 5, col 5

			// velocity_covariance
			//  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle
			//  (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.)
			for (auto &vc : msg.velocity_covariance) {
				vc = NAN;
			}

			msg.velocity_covariance[0]  = odom.velocity_covariance[odom.VELOCITY_COVARIANCE_VX_VAR];   // X  row 0, col 0
			msg.velocity_covariance[1]  = odom.velocity_covariance[odom.VELOCITY_COVARIANCE_VXVY_COV]; // XY row 0, col 1
			msg.velocity_covariance[2]  = odom.velocity_covariance[odom.VELOCITY_COVARIANCE_VXVZ_COV]; // XZ row 0, col 2
			msg.velocity_covariance[6]  = odom.velocity_covariance[odom.VELOCITY_COVARIANCE_VY_VAR];   // Y  row 1, col 1
			msg.velocity_covariance[7]  = odom.velocity_covariance[odom.VELOCITY_COVARIANCE_VYVZ_COV]; // YZ row 1, col 2
			msg.velocity_covariance[11] = odom.velocity_covariance[odom.VELOCITY_COVARIANCE_VZ_VAR];   // Z  row 2, col 2

			msg.reset_counter = odom.reset_counter;

			// source: PX4 estimator
			msg.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;

			msg.quality = odom.quality;

			mavlink_msg_odometry_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // ODOMETRY_HPP
