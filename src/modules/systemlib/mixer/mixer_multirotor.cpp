/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mixer_multirotor.cpp
 *
 * Multi-rotor mixers.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <math.h>

#include "mixer.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)
//#include <debug.h>
//#define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)

/*
 * Clockwise: 1
 * Counter-clockwise: -1
 */

namespace
{

/*
 * These tables automatically generated by multi_tables - do not edit.
 */
const MultirotorMixer::Rotor _config_quad_x[] = {
	{ -0.707107,  0.707107,  1.00 },
	{  0.707107, -0.707107,  1.00 },
	{  0.707107,  0.707107, -1.00 },
	{ -0.707107, -0.707107, -1.00 },
};
const MultirotorMixer::Rotor _config_quad_plus[] = {
	{ -1.000000,  0.000000,  1.00 },
	{  1.000000,  0.000000,  1.00 },
	{  0.000000,  1.000000, -1.00 },
	{ -0.000000, -1.000000, -1.00 },
};
const MultirotorMixer::Rotor _config_quad_v[] = {
	{ -0.927184,  0.374607,  1.00 },
	{  0.694658, -0.719340,  1.00 },
	{  0.927184,  0.374607, -1.00 },
	{ -0.694658, -0.719340, -1.00 },
};
const MultirotorMixer::Rotor _config_quad_wide[] = {
	{ -0.927184,  0.374607,  1.00 },
	{  0.777146, -0.629320,  1.00 },
	{  0.927184,  0.374607, -1.00 },
	{ -0.777146, -0.629320, -1.00 },
};
const MultirotorMixer::Rotor _config_hex_x[] = {
	{ -1.000000,  0.000000, -1.00 },
	{  1.000000,  0.000000,  1.00 },
	{  0.500000,  0.866025, -1.00 },
	{ -0.500000, -0.866025,  1.00 },
	{ -0.500000,  0.866025,  1.00 },
	{  0.500000, -0.866025, -1.00 },
};
const MultirotorMixer::Rotor _config_hex_plus[] = {
	{  0.000000,  1.000000, -1.00 },
	{ -0.000000, -1.000000,  1.00 },
	{  0.866025, -0.500000, -1.00 },
	{ -0.866025,  0.500000,  1.00 },
	{  0.866025,  0.500000,  1.00 },
	{ -0.866025, -0.500000, -1.00 },
};
const MultirotorMixer::Rotor _config_octa_x[] = {
	{ -0.382683,  0.923880, -1.00 },
	{  0.382683, -0.923880, -1.00 },
	{ -0.923880,  0.382683,  1.00 },
	{ -0.382683, -0.923880,  1.00 },
	{  0.382683,  0.923880,  1.00 },
	{  0.923880, -0.382683,  1.00 },
	{  0.923880,  0.382683, -1.00 },
	{ -0.923880, -0.382683, -1.00 },
};
const MultirotorMixer::Rotor _config_octa_plus[] = {
	{  0.000000,  1.000000, -1.00 },
	{ -0.000000, -1.000000, -1.00 },
	{ -0.707107,  0.707107,  1.00 },
	{ -0.707107, -0.707107,  1.00 },
	{  0.707107,  0.707107,  1.00 },
	{  0.707107, -0.707107,  1.00 },
	{  1.000000,  0.000000, -1.00 },
	{ -1.000000,  0.000000, -1.00 },
};
const MultirotorMixer::Rotor *_config_index[MultirotorMixer::Geometry::MAX_GEOMETRY] = {
	&_config_quad_x[0],
	&_config_quad_plus[0],
	&_config_quad_v[0],
	&_config_quad_wide[0],
	&_config_hex_x[0],
	&_config_hex_plus[0],
	&_config_octa_x[0],
	&_config_octa_plus[0],
	&_config_octa_cox[0],
};
const unsigned _config_rotor_count[MultirotorMixer::Geometry::MAX_GEOMETRY] = {
	4, /* quad_x */
	4, /* quad_plus */
	4, /* quad_v */
	4, /* quad_wide */
	6, /* hex_x */
	6, /* hex_plus */
	8, /* octa_x */
	8, /* octa_plus */
	8, /* octa_cox */
};

}

MultirotorMixer::MultirotorMixer(ControlCallback control_cb,
				 uintptr_t cb_handle,
				 Geometry geometry,
				 float roll_scale,
				 float pitch_scale,
				 float yaw_scale,
				 float deadband) :
	Mixer(control_cb, cb_handle),
	_roll_scale(roll_scale),
	_pitch_scale(pitch_scale),
	_yaw_scale(yaw_scale),
	_deadband(-1.0f + deadband),	/* shift to output range here to avoid runtime calculation */
	_rotor_count(_config_rotor_count[geometry]),
	_rotors(_config_index[geometry])
{
}

MultirotorMixer::~MultirotorMixer()
{
}

MultirotorMixer *
MultirotorMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	MultirotorMixer::Geometry geometry;
	char geomname[8];
	int s[4];
	int used;

	if (sscanf(buf, "R: %s %d %d %d %d%n", geomname, &s[0], &s[1], &s[2], &s[3], &used) != 5) {
		debug("multirotor parse failed on '%s'", buf);
		return nullptr;
	}

	if (used > (int)buflen) {
		debug("multirotor spec used %d of %u", used, buflen);
		return nullptr;
	}

	buflen -= used;

	if (!strcmp(geomname, "4+")) {
		geometry = MultirotorMixer::QUAD_PLUS;

	} else if (!strcmp(geomname, "4x")) {
		geometry = MultirotorMixer::QUAD_X;

	} else if (!strcmp(geomname, "4v")) {
		geometry = MultirotorMixer::QUAD_V;

	} else if (!strcmp(geomname, "4w")) {
		geometry = MultirotorMixer::QUAD_WIDE;

	} else if (!strcmp(geomname, "6+")) {
		geometry = MultirotorMixer::HEX_PLUS;

	} else if (!strcmp(geomname, "6x")) {
		geometry = MultirotorMixer::HEX_X;

	} else if (!strcmp(geomname, "8+")) {
		geometry = MultirotorMixer::OCTA_PLUS;

	} else if (!strcmp(geomname, "8x")) {
		geometry = MultirotorMixer::OCTA_X;
		
	} else if (!strcmp(geomname, "8c")) {
		geometry = MultirotorMixer::OCTA_COX;

	} else {
		debug("unrecognised geometry '%s'", geomname);
		return nullptr;
	}

	debug("adding multirotor mixer '%s'", geomname);

	return new MultirotorMixer(
		       control_cb,
		       cb_handle,
		       geometry,
		       s[0] / 10000.0f,
		       s[1] / 10000.0f,
		       s[2] / 10000.0f,
		       s[3] / 10000.0f);
}

unsigned
MultirotorMixer::mix(float *outputs, unsigned space)
{
	float		roll    = get_control(0, 0) * _roll_scale;
	//lowsyslog("roll: %d, get_control0: %d, %d\n", (int)(roll), (int)(get_control(0, 0)), (int)(_roll_scale));
	float		pitch   = get_control(0, 1) * _pitch_scale;
	float		yaw     = get_control(0, 2) * _yaw_scale;
	float		thrust  = get_control(0, 3);
	//lowsyslog("thrust: %d, get_control3: %d\n", (int)(thrust), (int)(get_control(0, 3)));
	float		max     = 0.0f;
	float		fixup_scale;

	/* use an output factor to prevent too strong control signals at low throttle */
	float min_thrust = 0.05f;
	float max_thrust = 1.0f;
	float startpoint_full_control = 0.40f;
	float output_factor;

	/* keep roll, pitch and yaw control to 0 below min thrust */
	if (thrust <= min_thrust) {
		output_factor = 0.0f;
		/* linearly increase the output factor from 0 to 1 between min_thrust and startpoint_full_control */

	} else if (thrust < startpoint_full_control && thrust > min_thrust) {
		output_factor = (thrust / max_thrust) / (startpoint_full_control - min_thrust);
		/* and then stay at full control */

	} else {
		output_factor = max_thrust;
	}

	roll *= output_factor;
	pitch *= output_factor;
	yaw *= output_factor;


	/* perform initial mix pass yielding un-bounded outputs */
	for (unsigned i = 0; i < _rotor_count; i++) {
		float tmp = roll  * _rotors[i].roll_scale +
			    pitch * _rotors[i].pitch_scale +
			    yaw   * _rotors[i].yaw_scale +
			    thrust;

		if (tmp > max)
			max = tmp;

		outputs[i] = tmp;
	}

	/* scale values into the -1.0 - 1.0 range */
	if (max > 1.0f) {
		fixup_scale = 2.0f / max;

	} else {
		fixup_scale = 2.0f;
	}

	for (unsigned i = 0; i < _rotor_count; i++)
		outputs[i] = -1.0f + (outputs[i] * fixup_scale);

	/* ensure outputs are out of the deadband */
	for (unsigned i = 0; i < _rotor_count; i++)
		if (outputs[i] < _deadband)
			outputs[i] = _deadband;

	return _rotor_count;
}

void
MultirotorMixer::groups_required(uint32_t &groups)
{
	/* XXX for now, hardcoded to indexes 0-3 in control group zero */
	groups |= (1 << 0);
}

