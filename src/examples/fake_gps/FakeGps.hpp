/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <stdio.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

struct UWB{
	int uwb_count{0};
	int solution_iter_num{0};
	double rou{0.};
	double xmm{0};
	double ymm{0};
	double zmm{0};
	double x2{0.};
	double y2{0.};
	double z2{0.};
	double vx{0.};
	double vy{0.};
	double vz{0.};
	double vgps{0.};
	double cog{0.};//course over ground
	double km2lat{0.};
	double km2lon{0.};
	double dlat{0};
	double dlon{0};
	//char buf[300] {};
	char data_buf[200] {};//read char buffer
	int data_buf_wp{0};
	double x_buf[5] {};
	double y_buf[5] {};
	double z_buf[5] {};
	double filter_g_p;
	double filter_g_v;
	double filter_dt;
	double heading{0.f};
	double heading_offset{0.f};
	int32_t sonar{0};
	double cosrou{0.};
	double sinrou{0.};

};

class FakeGps : public ModuleBase<FakeGps>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FakeGps();

	~FakeGps() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);


private:
	static constexpr uint32_t SENSOR_INTERVAL_US{1000000 / 10}; // 5 Hz

	void Run() override;
	void SetGps();

	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};


	// 37.287823 126.807157

	double _latitude{37.287823};   // Latitude in degrees
	double _longitude{126.807157}; // Longitude in degrees
	double _altitude{30.1};         // Altitude in meters above MSL, (millimetres)

	bool init();
	int count{0};
	int setSerialPort(int* fd, int speed, const char* port);
	int	_serial_fd{-1};
	int	_serial_fd2{-1};
	int	_serial_fd0{-1};
	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FAKE_GPS_ROT>) _fake_gps_rot,   /**< example parameter */
		(ParamFloat<px4::params::FAKE_GPS_LAT>) _fake_gps_lat,   /**< example parameter */
		(ParamFloat<px4::params::FAKE_GPS_LON>) _fake_gps_lon,   /**< example parameter */
		(ParamFloat<px4::params::FAKE_GPS_HGT>) _fake_gps_hgt,  /**< another parameter */
		(ParamFloat<px4::params::FAKE_GPS_KP>) _fake_gps_kp,
		(ParamFloat<px4::params::FAKE_GPS_KV>) _fake_gps_kv,
		(ParamFloat<px4::params::FAKE_GPS_DT>) _fake_gps_dt,
		(ParamInt<px4::params::FAKE_GPS_TAG>) _fake_gps_tag,
		(ParamInt<px4::params::FAKE_GPS_USS>) _fake_gps_uss,
		(ParamInt<px4::params::FAKE_GPS_ITR>) _fake_gps_itr,
		(ParamInt<px4::params::FAKE_GPS_UGV>) _fake_gps_ugv,
		(ParamInt<px4::params::FAKE_GPS_NED>) _fake_gps_ned,
		(ParamFloat<px4::params::FAKE_GPS_OFS>) _head_offset
	)
	UWB uwb;
	UWB uwb1;
	UWB uwb2;
	// Subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription	_vehicle_status_sub{ORB_ID(vehicle_status)};
	FILE *pFile = NULL;


	double velocity_estimation(double pos, double *x);
	void Mydatalog();
	void Mydatalog2();
	void calOneTag();
	void calTwoTag();
	int32_t _tag_number;
	bool _armed = false;
	int _uwblog_number{0};

	int _using_uss{0};
	int _using_ugv{0};
	int _using_ned_vel{1};
	void GetUwbData(int serial_fd,UWB *_uwb);
	void Parshing3();
	int _iteration_number{0};
	bool _set_fake_gps {false};


};
