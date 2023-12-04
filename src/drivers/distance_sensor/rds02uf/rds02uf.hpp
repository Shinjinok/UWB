/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file rds02uf.cpp
 * @author
 *
 * Driver for the MicrobrainRadar RDS02UF rangefinder series
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>

#define RDS02UF_DEFAULT_PORT	"/dev/ttyS3"
#define RDS02_HEAD1 0x55
#define RDS02_HEAD2 0x55
#define RDS02_END 0xAA

#define RDS02_DATA_START_INDEX 8
#define RDS02_DATA_Y_INDEX 13
#define RDS02_DATA_FC_INDEX    RDS02_DATA_START_INDEX

#define RDS02_BUFFER_SIZE 21

#define RDS02_TARGET_INFO  0x0C
// Data Format for Rds02UF
// ===============================
// 21 bytes total per message:
// 1) 0x55
// 2) 0x55
// 3) address
// 4) error_code
// 5) FC_CODE_L (low 8bit)
// 6) FC_CODE_H (high 8bit)
// 7) LENGTH_L (low 8bit)
// 8) LENGTH_H (high 8bit)
// 9) REAL_DATA (10Byte)
// 10) CRC8
// 11) END_1 0xAA
// 12) END_2 0xAA

using namespace time_literals;

class Rds02uf : public px4::ScheduledWorkItem
{
public:
	Rds02uf(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~Rds02uf();

	int init();

	void print_info();

private:
	PX4Rangefinder	_px4_rangefinder;
	enum RDS02UF_PARSE_STATE {
		STATE0_SYNC_1 = 0,
		STATE1_SYNC_2,
		STATE2_ADDRESS,
		STATE3_ERROR_CODE,
		STATE4_FC_CODE_L,
		STATE5_FC_CODE_H,
		STATE6_LENGTH_L,
		STATE7_LENGTH_H,
		STATE8_REAL_DATA,
		STATE9_CRC,
		STATE10_END_1,
		STATE11_END_2
	};
	RDS02UF_PARSE_STATE _parse_state {RDS02UF_PARSE_STATE::STATE0_SYNC_1};
	static constexpr int kCONVERSIONINTERVAL{50_ms};
	char parser_buffer[RDS02_BUFFER_SIZE] {};
	char _port[20] {};
	int _fd{-1};
	unsigned int parserbuf_index{0};
	hrt_abstime _last_read{0};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	int collect();
	void Run() override;
	void start();
	void stop();
	int rds02uf_parse(char c, float *dist);

#ifdef RDS02UF_USE_CRC	 // To save Flash and Ram, do not use CRC check
	const uint8_t crc8_table[256] = {
		0x93, 0x98, 0xE4, 0x46, 0xEB, 0xBA, 0x04, 0x4C,
		0xFA, 0x40, 0xB8, 0x96, 0x0E, 0xB2, 0xB7, 0xC0,
		0x0C, 0x32, 0x9B, 0x80, 0xFF, 0x30, 0x7F, 0x9D,
		0xB3, 0x81, 0x58, 0xE7, 0xF1, 0x19, 0x7E, 0xB6,
		0xCD, 0xF7, 0xB4, 0xCB, 0xBC, 0x5C, 0xD6, 0x09,
		0x20, 0x0A, 0xE0, 0x37, 0x51, 0x67, 0x24, 0x95,
		0xE1, 0x62, 0xF8, 0x5E, 0x38, 0x15, 0x54, 0x77,
		0x63, 0x57, 0x6D, 0xE9, 0x89, 0x76, 0xBE, 0x41,
		0x5D, 0xF9, 0xB1, 0x4D, 0x6C, 0x53, 0x9C, 0xA2,
		0x23, 0xC4, 0x8E, 0xC8, 0x05, 0x42, 0x61, 0x71,
		0xC5, 0x00, 0x18, 0x6F, 0x5F, 0xFB, 0x7B, 0x11,
		0x65, 0x2D, 0x8C, 0xED, 0x14, 0xAB, 0x88, 0xD5,
		0xD9, 0xC2, 0x36, 0x34, 0x7C, 0x5B, 0x3C, 0xF6,
		0x48, 0x0B, 0xEE, 0x02, 0x83, 0x79, 0x17, 0xE6,
		0xA8, 0x78, 0xF5, 0xD3, 0x4E, 0x50, 0x52, 0x91,
		0xD8, 0xC6, 0x22, 0xEC, 0x3B, 0xE5, 0x3F, 0x86,
		0x06, 0xCF, 0x2B, 0x2F, 0x3D, 0x59, 0x1C, 0x87,
		0xEF, 0x4F, 0x10, 0xD2, 0x7D, 0xDA, 0x72, 0xA0,
		0x9F, 0xDE, 0x6B, 0x75, 0x56, 0xBD, 0xC7, 0xC1,
		0x70, 0x1D, 0x25, 0x92, 0xA5, 0x31, 0xE2, 0xD7,
		0xD0, 0x9A, 0xAF, 0xA9, 0xC9, 0x97, 0x08, 0x33,
		0x5A, 0x99, 0xC3, 0x16, 0x84, 0x82, 0x8A, 0xF3,
		0x4A, 0xCE, 0xDB, 0x29, 0x0F, 0xAE, 0x6E, 0xE3,
		0x8B, 0x07, 0x3A, 0x74, 0x47, 0xB0, 0xBB, 0xB5,
		0x7A, 0xAA, 0x2C, 0xD4, 0x03, 0x3E, 0x1A, 0xA7,
		0x27, 0x64, 0x06, 0xBF, 0x55, 0x73, 0x1E, 0xFE,
		0x49, 0x01, 0x39, 0x28, 0xF4, 0x26, 0xDF, 0xDD,
		0x44, 0x0D, 0x21, 0xF2, 0x85, 0xB9, 0xEA, 0x4B,
		0xDC, 0x6A, 0xCA, 0xAC, 0x12, 0xFC, 0x2E, 0x2A,
		0xA3, 0xF0, 0x66, 0xE8, 0x60, 0x45, 0xA1, 0x8D,
		0x68, 0x35, 0xFD, 0x8F, 0x9E, 0x1F, 0x13, 0xD1,
		0xAD, 0x69, 0xCC, 0xA4, 0x94, 0x90, 0x1B, 0x43,
	};

	uint8_t crc8(char *pbuf, int32_t len);
#endif

#ifdef RDS02UF_DEBUG
	const char *parser_state[] = {

		"0_STATE1_SYNC_1",
		"1_STATE2_SYNC_2",
		"2_STATE3_ADDRESS",
		"3_STATE4_ERROR_CODE",
		"4_STATE5_FC_CODE_L",
		"5_STATE6_FC_CODE_H",
		"6_STATE7_LENGTH_L",
		"7_STATE8_LENGTH_H",
		"8_STATE9_REAL_DATA",
		"9_STATE10_CRC",
		"10_STATE11_END_1",
		"11_STATE12_END_2"
	};
#endif
};
