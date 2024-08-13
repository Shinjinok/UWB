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

#include "FakeGps.hpp"
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <lib/parameters/param.h>
#include <uORB/topics/actuator_armed.h>
//#include <uORB/topics/actuator_controls.h>
#include <uORB/uORB.h>
#include <sys/stat.h>

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)
//#define SITL
using namespace time_literals;

const char *TTY2 = "/dev/ttyS2";
const char *TTY3 = "/dev/ttyS3";
const char *TTY0 = "/dev/ttyS0";
const double rad360 = M_PI * 2.0;
const double rad270 = M_PI * 3.0 / 2.0;
const double rad90 = M_PI / 2.0;
FakeGps::FakeGps() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool FakeGps::init()
{
	ScheduleOnInterval(SENSOR_INTERVAL_US);

	parameters_update();


	return true;
}

double  FakeGps::velocity_estimation(double pos, double *x)
{
	double xp[2];
	xp[0] = x[0] + uwb.filter_dt * x[1];
	xp[1] = x[1];
	double e = pos - xp[0];
	x[0] = xp[0] + uwb.filter_g_p * e;
	x[1] = xp[1] + uwb.filter_g_v * e;

	return x[1];

}

void FakeGps::GetUwbData(int serial_fd,UWB * _uwb)
{
	int bytes_available = 0;

	::ioctl(serial_fd,FIONREAD, (unsigned long)&bytes_available);

	if(bytes_available > (int) sizeof( _uwb->data_buf) - 1){ //check buffer overflow
		bytes_available = sizeof(_uwb->data_buf);
	}

	if(bytes_available > 0 ){
		for(int k = 0; k < bytes_available; k++){

			char read_data;
			int ret =::read(serial_fd, &read_data, 1);
			if(ret < 0){

			}
			if(read_data != '\n'){
				_uwb->data_buf[_uwb->data_buf_wp++] = read_data;
			}
			else{
				char *sbuf[10] = {NULL,};
				int sbuf_pointer=0;
				char *ptr = strtok(_uwb->data_buf," ");

				while(ptr != NULL){
					sbuf[sbuf_pointer++]=ptr;
					ptr = strtok(NULL," ");

				};
				_uwb->uwb_count = atoi(sbuf[0]);
				_uwb->solution_iter_num = atoi(sbuf[1]);
				if(_uwb->solution_iter_num < _iteration_number){
					_uwb->ymm = atoi(sbuf[2]);
					_uwb->xmm = atoi(sbuf[3]);
					_uwb->zmm = atoi(sbuf[4]);
					_uwb->heading = ((double) atoi(sbuf[5]))* M_PI/ 1800.;
				}
				_uwb->data_buf_wp = 0;
				memset(_uwb->data_buf,'\0',sizeof(_uwb->data_buf));

			}
		}
	}

}
void FakeGps::Parshing3()
{
	if(_tag_number == 2){
		GetUwbData(_serial_fd, &uwb1);
		GetUwbData(_serial_fd2, &uwb2);
		calTwoTag();
		Mydatalog2();
	}
	else{
		GetUwbData(_serial_fd, &uwb);
		calOneTag();
	}

	if(!_set_fake_gps) {
		SetGps();
		_set_fake_gps = true;
	}
	else{
		_set_fake_gps = false;
	}

}
void FakeGps::calOneTag()
{
	//rotation uwb coordnation to wgs84 system
	uwb.x2 = uwb.cosrou * (double) uwb.xmm + uwb.sinrou * (double) uwb.ymm;
	uwb.y2 = -uwb.sinrou * (double) uwb.xmm + uwb.cosrou * (double) uwb.ymm;
	uwb.z2 = (double) uwb.zmm;
	//velocity estimation
	uwb.vx = velocity_estimation(uwb.x2 / 1000., uwb.x_buf);
	uwb.vy = velocity_estimation(uwb.y2 / 1000., uwb.y_buf);
	uwb.vz = velocity_estimation(uwb.z2 / 1000., uwb.z_buf);
	//flight speed
	uwb.vgps = sqrt(uwb.vx*uwb.vx+uwb.vy*uwb.vy);
	//course over ground
	uwb.cog=0.;
	if(uwb.vy > -1e-7 && uwb.vy < 1e-7 && uwb.vx > 0. ){
		uwb.cog = 0.;
	}
	else if(uwb.vy > 0.){
		uwb.cog = rad90 - atan(uwb.vx/uwb.vy);
	}
	else if (uwb.vy < 0.){
		uwb.cog = rad270 - atan(uwb.vx/uwb.vy);
	}
	else{
		uwb.cog = rad360;
	}


	uwb.dlat = uwb.x2 * uwb.km2lat * 10.;
	uwb.dlon = uwb.y2 * uwb.km2lon * 10.;
}


void FakeGps::calTwoTag()
{
	//position
	uwb.xmm = (uwb1.xmm + uwb2.xmm)/2;
	uwb.ymm = (uwb1.ymm + uwb2.ymm)/2;
	uwb.zmm = (uwb1.zmm + uwb2.zmm)/2;
	//heading
	int dx = uwb1.xmm - uwb2.xmm;
	int dy = uwb1.ymm - uwb2.ymm;
	if(dx >= 0 && dy == 0){
		uwb.heading = 0.;
	}
	else if(dy > 0) {
		uwb.heading = rad90  - atan((double) dx/(double)dy);
	}
	else if(dy < 0){
		uwb.heading = rad270 - atan((double) dx/(double)dy);
	}
	else{
		uwb.heading = rad360;
	}
	uwb.heading = uwb.heading + uwb.heading_offset - uwb.rou;

	if(uwb.heading < 0. ){
		uwb.heading = uwb.heading + rad360;
	}
	else if(uwb.heading > rad360){
		uwb.heading = uwb.heading - rad360;
	}



	//rotation uwb coordnation to wgs84 system
	uwb.x2 = uwb.cosrou * (double) uwb.xmm + uwb.sinrou * (double) uwb.ymm;
	uwb.y2 = -uwb.sinrou * (double) uwb.xmm + uwb.cosrou * (double) uwb.ymm;
	uwb.z2 = (double) uwb.zmm;
	//velocity estimation
	uwb.vx = velocity_estimation(uwb.x2 / 1000., uwb.x_buf);
	uwb.vy = velocity_estimation(uwb.y2 / 1000., uwb.y_buf);
	uwb.vz = velocity_estimation(uwb.z2 / 1000., uwb.z_buf);
	//flight speed
	uwb.vgps = sqrt(uwb.vx*uwb.vx+uwb.vy*uwb.vy);
	//course over ground
	uwb.cog=0.;
	if(uwb.vy > -1e-7 && uwb.vy < 1e-7 && uwb.vx > 0. ){
		uwb.cog = 0.;
	}
	else if(uwb.vy > 0.){
		uwb.cog = rad90 - atan(uwb.vx/uwb.vy);
	}
	else if (uwb.vy < 0.){
		uwb.cog = rad270 - atan(uwb.vx/uwb.vy);
	}
	else{
		uwb.cog = rad360;
	}


	uwb.dlat = (double) uwb.x2 * uwb.km2lat * 10.;
	uwb.dlon = (double) uwb.y2 * uwb.km2lon * 10.;
}


void FakeGps::SetGps()
{
		sensor_gps_s sensor_gps;
		sensor_gps.device_id = 1;
		sensor_gps.time_utc_usec = hrt_absolute_time();// + 1613692609599954;
		sensor_gps.latitude_deg = _latitude + uwb.dlat;
		sensor_gps.longitude_deg = _longitude + uwb.dlon;

		if(_using_ugv == 1){
			sensor_gps.altitude_msl_m = _altitude;
			sensor_gps.altitude_ellipsoid_m = _altitude;
			sensor_gps.vel_d_m_s = 0 ;// m/s
		}
		else{
			sensor_gps.altitude_msl_m = _altitude + uwb.zmm;
			sensor_gps.altitude_ellipsoid_m = _altitude + uwb.zmm;
			sensor_gps.vel_d_m_s = -uwb.vz ;// m/s
		}
		sensor_gps.s_variance_m_s = 0.15f;
		sensor_gps.c_variance_rad = 0.2f;
		sensor_gps.eph = 0.55f;
		sensor_gps.epv = 1.f;
		sensor_gps.hdop = 0.58f;
		sensor_gps.vdop = 1.f;
		sensor_gps.noise_per_ms = 100;
		sensor_gps.jamming_indicator = 9;
		sensor_gps.vel_m_s = uwb.vgps ;// m/s
		sensor_gps.vel_n_m_s = uwb.vx ;// m/s
		sensor_gps.vel_e_m_s = uwb.vy ;// m/s

		sensor_gps.cog_rad = uwb.cog;
		sensor_gps.timestamp_time_relative = 0;
		sensor_gps.heading = uwb.heading;
		sensor_gps.heading_offset = 0.0;
		sensor_gps.fix_type = 3;
		sensor_gps.jamming_state = 0;
		sensor_gps.spoofing_state = 0;
		sensor_gps.vel_ned_valid = (bool) _using_ned_vel;
		sensor_gps.satellites_used = 14;
		sensor_gps.timestamp = hrt_absolute_time();
		_sensor_gps_pub.publish(sensor_gps);
}

void FakeGps::Mydatalog()
{
	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.update(&vehicle_status)) {

		if(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){
			_armed = true;
		}
		else _armed = false;

	}
	if(pFile == NULL && _armed ){
		struct stat st;

		if (stat("/fs/microsd/log/uwblog", &st) == -1) {
			mkdir("/fs/microsd/log/uwblog", 0700);
		}

		char path[100] = "";
		sprintf(path,"/fs/microsd/log/uwblog/uwblog%dcsv.ulg",_uwblog_number++);
		pFile = fopen(path,"w");
		fprintf(pFile,"#cnt1,cnt2,sonar,x2,y2,z2,vx,vy,vz,vgps,cog,heading\n");
		//PX4_WARN("SD File open\n");
	}
	else if(_armed) {
		fprintf(pFile,"%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
		uwb1.uwb_count, uwb2.uwb_count, (int)  uwb.sonar,
		(int) uwb.x2,(int)  uwb.y2,(int)  uwb.z2,
		uwb.vx,uwb.vy,uwb.vz,
		uwb.vgps,uwb.cog,uwb.heading);
	}
	else if(pFile != NULL){
		fclose(pFile);
		pFile = NULL;
	}
}
void FakeGps::Mydatalog2()
{
	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.update(&vehicle_status)) {

		if(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED){
			_armed = true;
		}
		else _armed = false;

	}
	if(pFile == NULL && _armed ){
		struct stat st;

		if (stat("/fs/microsd/log/uwblog", &st) == -1) {
			mkdir("/fs/microsd/log/uwblog", 0700);
		}

		char path[100] = "";
		sprintf(path,"/fs/microsd/log/uwblog/uwblog%dcsv.ulg",_uwblog_number++);
		pFile = fopen(path,"w");
		fprintf(pFile,"#cnt1,cnt2,uwb1.xmm,uwb1.ymm,uwb1.zmm,uwb2.xmm,uwb2.ymm,uwb2.zmm\n");
		//PX4_WARN("SD File open\n");
	}
	else if(_armed) {
		fprintf(pFile,"%d,%d,%d,%d,%d,%d,%d,%d\n",
		uwb1.uwb_count, uwb2.uwb_count, uwb1.xmm,uwb1.ymm,uwb1.zmm,uwb2.xmm,uwb2.ymm,uwb2.zmm);
	}
	else if(pFile != NULL){
		fclose(pFile);
		pFile = NULL;
	}
}
void FakeGps::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();

		return;
	}

	if(_tag_number == 1){
		if(_serial_fd < 0 ){
			setSerialPort(&_serial_fd, B115200,TTY2);
		}
		else{
			//parameters_update();
			Parshing3();
			//SetGps();
		}
	}

	if(_tag_number == 2){

		if(_serial_fd < 0 || _serial_fd2 < 0 ){
			setSerialPort(&_serial_fd, B115200,TTY2);
			setSerialPort(&_serial_fd2, B115200,TTY3);

		}/*
		else if(_using_uss == 1  && _serial_fd0 < 0){
			setSerialPort(&_serial_fd0, 9600,TTY0);
		}*/
		else{
			//parameters_update();
			Parshing3();

		}

	}

	//SetGps();


}
int FakeGps::setSerialPort(int* serial_fd, int speed, const char* name)
{
	int fd = *serial_fd;
	if (fd < 0) {
			/* open the serial port */
			fd = ::open(name, O_RDWR | O_NOCTTY);

			if (fd < 0) {
				PX4_WARN("failed to open err: %d",  errno);
				px4_sleep(1);

			}
			PX4_WARN("serial opened %d",fd);
	}


	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);

	uart_config.c_oflag = 0;

	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_WARN("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_WARN("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

	*serial_fd = fd;

	return 0;
}
int FakeGps::task_spawn(int argc, char *argv[])
{
	FakeGps *instance = new FakeGps();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FakeGps::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeGps::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}
void FakeGps::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
		_using_ned_vel  = _fake_gps_ned.get();
		_using_ugv = _fake_gps_ugv.get();
		_using_uss = _fake_gps_uss.get();
		_iteration_number = _fake_gps_itr.get();
		/* get yaw rotation from sensor frame to body frame */
		uwb.rou = (double) _fake_gps_rot.get() * M_PI / 180.;
		uwb.cosrou = cos(uwb.rou);
		uwb.sinrou = sin(uwb.rou);
		//Initial latitude longitude
		_latitude  = (double) _fake_gps_lat.get();// * 1e7;
		_longitude = (double) _fake_gps_lon.get();// * 1e7;
		_altitude  = (double) _fake_gps_hgt.get();// * 1e3;
		//Kalman filter gain
		uwb.filter_g_p = _fake_gps_kp.get();
		uwb.filter_g_v = _fake_gps_kv.get();
		//sampling time(s)
		uwb.filter_dt = _fake_gps_dt.get();
		//tag number check
		_tag_number = _fake_gps_tag.get();
		uwb.heading_offset = (double) _head_offset.get() * M_PI / 180.;

		double a = 6378.1370; // Earth radius km
		double b = 6356.7523; // Earth radius km
		double Ly = 2.0*M_PI*sqrt(0.5*(pow(a,2)+pow(b,2)));
		double theta = (double) _fake_gps_lat.get() * M_PI /180.;
		uwb.km2lat = 360./Ly;
		double Rytheta = sqrt(pow(a*b,2)/(pow(b,2)+pow(a*tan(theta),2)));
		uwb.km2lon = 180./(M_PI*Rytheta);

	}
}

extern "C" __EXPORT int fake_gps_main(int argc, char *argv[])
{
	return FakeGps::main(argc, argv);
}




/*
if(_using_uss == 1){
		::ioctl(_serial_fd0,FIONREAD, (unsigned long)&bytes_available);

		if(bytes_available > (int) sizeof( uwb.buf) - 1){ //check buffer overflow
			bytes_available = sizeof(uwb.buf);
		}


		if(bytes_available > 0 ){

			for(int k = 0; k < bytes_available; k++){

				char read_data;
				int ret = ::read(_serial_fd0, &read_data, 1);
				if (ret < 0 ){
					printf("error");
				}
				if(read_data == 'R'){
					uwb.data_buf_wp = 0;
					memset(uwb.data_buf,NULL,sizeof(uwb.data_buf));
					//PX4_WARN("R %d",uwb.data_buf_wp);
				}
				else if(read_data == 0x0D){
					int d = atoi(uwb.data_buf);
					if(d > 5 && d < 256){
						uwb.sonar = (int32_t)( (double) atoi(uwb.data_buf) * 25.4);
					}

					//PX4_WARN("0x0D %d",uwb.data_buf_wp);
				}
				else{
					uwb.data_buf[uwb.data_buf_wp++] = read_data;
				}
				if(sizeof(uwb.data_buf) - 1 < (unsigned int) uwb.data_buf_wp ) uwb.data_buf_wp =0;

			}
		}
	}

*/
