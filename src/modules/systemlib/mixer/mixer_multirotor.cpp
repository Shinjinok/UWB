/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
#include <px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <math.h>

#include <px4iofirmware/protocol.h>
#include <drivers/drv_pwm_output.h>

#include "mixer.h"

// This file is generated by the multi_tables script which is invoked during the build process
#include "mixer_multirotor.generated.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)
//#include <debug.h>
//#define debug(fmt, args...)	syslog(fmt "\n", ##args)

/*
 * Clockwise: 1
 * Counter-clockwise: -1
 */

namespace
{

float constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

} // anonymous namespace

MultirotorMixer::MultirotorMixer(ControlCallback control_cb,
				 uintptr_t cb_handle,
				 MultirotorGeometry geometry,
				 float roll_scale,
				 float pitch_scale,
				 float yaw_scale,
				 float idle_speed) :
	Mixer(control_cb, cb_handle),
	_roll_scale(roll_scale),
	_pitch_scale(pitch_scale),
	_yaw_scale(yaw_scale),
	_idle_speed(-1.0f + idle_speed * 2.0f),	/* shift to output range here to avoid runtime calculation */
	_delta_out_max(0.0f),
	_thrust_factor(0.0f),
	_limits_pub(),
	_rotor_count(_config_rotor_count[(MultirotorGeometryUnderlyingType)geometry]),
	_rotors(_config_index[(MultirotorGeometryUnderlyingType)geometry]),
	_geometry(geometry),
	_outputs_prev(new float[_rotor_count])
{
	memset(_outputs_prev, _idle_speed, _rotor_count * sizeof(float));
}


//MultirotorMixer::MultirotorMixer(ControlCallback control_cb,
//				 uintptr_t cb_handle,
//				 mixer_multi_s *mixer_info) :
//	Mixer(control_cb, cb_handle),
//	_roll_scale(mixer_info->roll_scale),
//	_pitch_scale(mixer_info->pitch_scale),
//	_yaw_scale(mixer_info->yaw_scale),
//	_idle_speed(-1.0f + mixer_info->idle_speed * 2.0f),	/* shift to output range here to avoid runtime calculation */
//	_delta_out_max(0.0f),
//	_thrust_factor(0.0f),
//	_limits_pub(),
//	_rotor_count(_config_rotor_count[(MultirotorGeometryUnderlyingType)mixer_info->geometry]),
//	_rotors(_config_index[(MultirotorGeometryUnderlyingType)mixer_info->geometry]),
//	_geometry(mixer_info->geometry),
//	_outputs_prev(new float[_rotor_count])
//{
//	memset(_outputs_prev, _idle_speed, _rotor_count * sizeof(float));
//}


MultirotorMixer::~MultirotorMixer()
{
	if (_outputs_prev != nullptr) {
		delete[] _outputs_prev;
	}
}

MultirotorMixer *
MultirotorMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	MultirotorGeometry geometry;
	char geomname[8];
	int s[4];
	int used;

	/* enforce that the mixer ends with a new line */
	if (!string_well_formed(buf, buflen)) {
		return nullptr;
	}

	if (sscanf(buf, "R: %7s %d %d %d %d%n", geomname, &s[0], &s[1], &s[2], &s[3], &used) != 5) {
		debug("multirotor parse failed on '%s'", buf);
		return nullptr;
	}

	if (used > (int)buflen) {
		debug("OVERFLOW: multirotor spec used %d of %u", used, buflen);
		return nullptr;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		return nullptr;
	}

	debug("remaining in buf: %d, first char: %c", buflen, buf[0]);

	if (!strcmp(geomname, "4+")) {
		geometry = MultirotorGeometry::QUAD_PLUS;

	} else if (!strcmp(geomname, "4x")) {
		geometry = MultirotorGeometry::QUAD_X;

	} else if (!strcmp(geomname, "4h")) {
		geometry = MultirotorGeometry::QUAD_H;

	} else if (!strcmp(geomname, "4v")) {
		geometry = MultirotorGeometry::QUAD_V;

	} else if (!strcmp(geomname, "4w")) {
		geometry = MultirotorGeometry::QUAD_WIDE;

	} else if (!strcmp(geomname, "4s")) {
		geometry = MultirotorGeometry::QUAD_S250AQ;

	} else if (!strcmp(geomname, "4dc")) {
		geometry = MultirotorGeometry::QUAD_DEADCAT;

	} else if (!strcmp(geomname, "6+")) {
		geometry = MultirotorGeometry::HEX_PLUS;

	} else if (!strcmp(geomname, "6x")) {
		geometry = MultirotorGeometry::HEX_X;

	} else if (!strcmp(geomname, "6c")) {
		geometry = MultirotorGeometry::HEX_COX;

	} else if (!strcmp(geomname, "6t")) {
		geometry = MultirotorGeometry::HEX_T;

	} else if (!strcmp(geomname, "8+")) {
		geometry = MultirotorGeometry::OCTA_PLUS;

	} else if (!strcmp(geomname, "8x")) {
		geometry = MultirotorGeometry::OCTA_X;

	} else if (!strcmp(geomname, "8c")) {
		geometry = MultirotorGeometry::OCTA_COX;

#if 0

	} else if (!strcmp(geomname, "8cw")) {
		geometry = MultirotorGeometry::OCTA_COX_WIDE;
#endif

	} else if (!strcmp(geomname, "2-")) {
		geometry = MultirotorGeometry::TWIN_ENGINE;

	} else if (!strcmp(geomname, "3y")) {
		geometry = MultirotorGeometry::TRI_Y;

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

#if defined(MIXER_TUNING)
#if !defined(MIXER_REMOTE)
int
MultirotorMixer::to_text(char *buf, unsigned &buflen)
{

	char geomname[8];

	switch (_geometry) {
	case MultirotorGeometry::QUAD_PLUS:
		strcpy(geomname, "4+");
		break;

	case MultirotorGeometry::QUAD_X:
		strcpy(geomname, "4x");
		break;

	case MultirotorGeometry::QUAD_H:
		strcpy(geomname, "4");
		break;

	case MultirotorGeometry::QUAD_V:
		strcpy(geomname, "4v");
		break;

	case MultirotorGeometry::QUAD_WIDE:
		strcpy(geomname, "4w");
		break;

	case MultirotorGeometry::QUAD_DEADCAT:
		strcpy(geomname, "4dc");
		break;

	case MultirotorGeometry::HEX_PLUS:
		strcpy(geomname, "6+");
		break;

	case MultirotorGeometry::HEX_X:
		strcpy(geomname, "6x");
		break;

	case MultirotorGeometry::HEX_COX:
		strcpy(geomname, "6c");
		break;

	case MultirotorGeometry::OCTA_PLUS:
		strcpy(geomname, "8+");
		break;

	case MultirotorGeometry::OCTA_X:
		strcpy(geomname, "8x");
		break;

	case MultirotorGeometry::OCTA_COX:
		strcpy(geomname, "8c");
		break;

	case MultirotorGeometry::OCTA_COX_WIDE:
		strcpy(geomname, "8cw");
		break;

	case MultirotorGeometry::TWIN_ENGINE:
		strcpy(geomname, "2-");
		break;

	case MultirotorGeometry::TRI_Y:
		strcpy(geomname, "3y");
		break;

	default:
		return -1;
		break;
	}

	int written = snprintf(buf, buflen, "R: %s %d %d %d %d\n",
			       geomname,
			       (int)(_roll_scale * 10000.0f),
			       (int)(_pitch_scale * 10000.0f),
			       (int)(_yaw_scale * 10000.0f),
			       (int)((_idle_speed + 1.0f) * 5000.0f)
			      );

	if (written >= buflen - 1) {
		return -1;
	}

	buflen = written;
	return 0;
}
#endif //MIXER_REMOTE
#endif //defined(MIXER_TUNING)

unsigned
MultirotorMixer::mix(float *outputs, unsigned space, uint16_t *status_reg)
{
	/* Summary of mixing strategy:
	1) mix roll, pitch and thrust without yaw.
	2) if some outputs violate range [0,1] then try to shift all outputs to minimize violation ->
		increase or decrease total thrust (boost). The total increase or decrease of thrust is limited
		(max_thrust_diff). If after the shift some outputs still violate the bounds then scale roll & pitch.
		In case there is violation at the lower and upper bound then try to shift such that violation is equal
		on both sides.
	3) mix in yaw and scale if it leads to limit violation.
	4) scale all outputs to range [idle_speed,1]
	*/

	float		roll    = constrain(get_control(0, 0) * _roll_scale, -1.0f, 1.0f);
	float		pitch   = constrain(get_control(0, 1) * _pitch_scale, -1.0f, 1.0f);
	float		yaw     = constrain(get_control(0, 2) * _yaw_scale, -1.0f, 1.0f);
	float		thrust  = constrain(get_control(0, 3), 0.0f, 1.0f);
	float		min_out = 1.0f;
	float		max_out = 0.0f;

	// clean out class variable used to capture saturation
	_saturation_status.value = 0;

	// thrust boost parameters
	float thrust_increase_factor = 1.5f;
	float thrust_decrease_factor = 0.6f;

	/* perform initial mix pass yielding unbounded outputs, ignore yaw */
	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = roll * _rotors[i].roll_scale +
			    pitch * _rotors[i].pitch_scale +
			    thrust;

		out *= _rotors[i].out_scale;

		/* calculate min and max output values */
		if (out < min_out) {
			min_out = out;
		}

		if (out > max_out) {
			max_out = out;
		}

		outputs[i] = out;
	}

	float boost = 0.0f;		// value added to demanded thrust (can also be negative)
	float roll_pitch_scale = 1.0f;	// scale for demanded roll and pitch

	if (min_out < 0.0f && max_out < 1.0f && -min_out <= 1.0f - max_out) {
		float max_thrust_diff = thrust * thrust_increase_factor - thrust;

		if (max_thrust_diff >= -min_out) {
			boost = -min_out;

		} else {
			boost = max_thrust_diff;
			roll_pitch_scale = (thrust + boost) / (thrust - min_out);
		}

	} else if (max_out > 1.0f && min_out > 0.0f && min_out >= max_out - 1.0f) {
		float max_thrust_diff = thrust - thrust_decrease_factor * thrust;

		if (max_thrust_diff >= max_out - 1.0f) {
			boost = -(max_out - 1.0f);

		} else {
			boost = -max_thrust_diff;
			roll_pitch_scale = (1 - (thrust + boost)) / (max_out - thrust);
		}

	} else if (min_out < 0.0f && max_out < 1.0f && -min_out > 1.0f - max_out) {
		float max_thrust_diff = thrust * thrust_increase_factor - thrust;
		boost = constrain(-min_out - (1.0f - max_out) / 2.0f, 0.0f, max_thrust_diff);
		roll_pitch_scale = (thrust + boost) / (thrust - min_out);

	} else if (max_out > 1.0f && min_out > 0.0f && min_out < max_out - 1.0f) {
		float max_thrust_diff = thrust - thrust_decrease_factor * thrust;
		boost = constrain(-(max_out - 1.0f - min_out) / 2.0f, -max_thrust_diff, 0.0f);
		roll_pitch_scale = (1 - (thrust + boost)) / (max_out - thrust);

	} else if (min_out < 0.0f && max_out > 1.0f) {
		boost = constrain(-(max_out - 1.0f + min_out) / 2.0f, thrust_decrease_factor * thrust - thrust,
				  thrust_increase_factor * thrust - thrust);
		roll_pitch_scale = (thrust + boost) / (thrust - min_out);
	}

	// capture saturation
	if (min_out < 0.0f) {
		_saturation_status.flags.motor_neg = true;
	}

	if (max_out > 1.0f) {
		_saturation_status.flags.motor_pos = true;
	}

	// Thrust reduction is used to reduce the collective thrust if we hit
	// the upper throttle limit
	float thrust_reduction = 0.0f;

	// mix again but now with thrust boost, scale roll/pitch and also add yaw
	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = (roll * _rotors[i].roll_scale +
			     pitch * _rotors[i].pitch_scale) * roll_pitch_scale +
			    yaw * _rotors[i].yaw_scale +
			    thrust + boost;

		out *= _rotors[i].out_scale;

		// scale yaw if it violates limits. inform about yaw limit reached
		if (out < 0.0f) {
			if (fabsf(_rotors[i].yaw_scale) <= FLT_EPSILON) {
				yaw = 0.0f;

			} else {
				yaw = -((roll * _rotors[i].roll_scale + pitch * _rotors[i].pitch_scale) *
					roll_pitch_scale + thrust + boost) / _rotors[i].yaw_scale;
			}

		} else if (out > 1.0f) {
			// allow to reduce thrust to get some yaw response
			float prop_reduction = fminf(0.15f, out - 1.0f);
			// keep the maximum requested reduction
			thrust_reduction = fmaxf(thrust_reduction, prop_reduction);

			if (fabsf(_rotors[i].yaw_scale) <= FLT_EPSILON) {
				yaw = 0.0f;

			} else {
				yaw = (1.0f - ((roll * _rotors[i].roll_scale + pitch * _rotors[i].pitch_scale) *
					       roll_pitch_scale + thrust + boost)) / _rotors[i].yaw_scale;
			}
		}
	}

	// Apply collective thrust reduction, the maximum for one prop
	thrust -= thrust_reduction;

	// add yaw and scale outputs to range idle_speed...1
	for (unsigned i = 0; i < _rotor_count; i++) {
		outputs[i] = (roll * _rotors[i].roll_scale +
			      pitch * _rotors[i].pitch_scale) * roll_pitch_scale +
			     yaw * _rotors[i].yaw_scale +
			     thrust + boost;

		/*
			implement simple model for static relationship between applied motor pwm and motor thrust
			model: thrust = (1 - _thrust_factor) * PWM + _thrust_factor * PWM^2
			this model assumes normalized input / output in the range [0,1] so this is the right place
			to do it as at this stage the outputs are in that range.
		 */
		if (_thrust_factor > 0.0f) {
			outputs[i] = -(1.0f - _thrust_factor) / (2.0f * _thrust_factor) + sqrtf((1.0f - _thrust_factor) *
					(1.0f - _thrust_factor) / (4.0f * _thrust_factor * _thrust_factor) + (outputs[i] < 0.0f ? 0.0f : outputs[i] /
							_thrust_factor));
		}

		outputs[i] = constrain(_idle_speed + (outputs[i] * (1.0f - _idle_speed)), _idle_speed, 1.0f);

	}

	/* slew rate limiting and saturation checking */
	for (unsigned i = 0; i < _rotor_count; i++) {
		bool clipping_high = false;
		bool clipping_low = false;

		// check for saturation against static limits
		if (outputs[i] > 0.99f) {
			clipping_high = true;

		} else if (outputs[i] < _idle_speed + 0.01f) {
			clipping_low = true;

		}

		// check for saturation against slew rate limits
		if (_delta_out_max > 0.0f) {
			float delta_out = outputs[i] - _outputs_prev[i];

			if (delta_out > _delta_out_max) {
				outputs[i] = _outputs_prev[i] + _delta_out_max;
				clipping_high = true;

			} else if (delta_out < -_delta_out_max) {
				outputs[i] = _outputs_prev[i] - _delta_out_max;
				clipping_low = true;

			}
		}

		_outputs_prev[i] = outputs[i];

		// update the saturation status report
		update_saturation_status(i, clipping_high, clipping_low);

	}

	// this will force the caller of the mixer to always supply new slew rate values, otherwise no slew rate limiting will happen
	_delta_out_max = 0.0f;

	// Notify saturation status
	if (status_reg != nullptr) {
		(*status_reg) = _saturation_status.value;
	}

	return _rotor_count;
}

/*
 * This function update the control saturation status report using hte following inputs:
 *
 * index: 0 based index identifying the motor that is saturating
 * clipping_high: true if the motor demand is being limited in the positive direction
 * clipping_low: true if the motor demand is being limited in the negative direction
*/
void
MultirotorMixer::update_saturation_status(unsigned index, bool clipping_high, bool clipping_low)
{
	// The motor is saturated at the upper limit
	// check which control axes and which directions are contributing
	if (clipping_high) {
		if (_rotors[index].roll_scale > 0.0f) {
			// A positive change in roll will increase saturation
			_saturation_status.flags.roll_pos = true;

		} else if (_rotors[index].roll_scale < 0.0f) {
			// A negative change in roll will increase saturation
			_saturation_status.flags.roll_neg = true;

		}

		// check if the pitch input is saturating
		if (_rotors[index].pitch_scale > 0.0f) {
			// A positive change in pitch will increase saturation
			_saturation_status.flags.pitch_pos = true;

		} else if (_rotors[index].pitch_scale < 0.0f) {
			// A negative change in pitch will increase saturation
			_saturation_status.flags.pitch_neg = true;

		}

		// check if the yaw input is saturating
		if (_rotors[index].yaw_scale > 0.0f) {
			// A positive change in yaw will increase saturation
			_saturation_status.flags.yaw_pos = true;

		} else if (_rotors[index].yaw_scale < 0.0f) {
			// A negative change in yaw will increase saturation
			_saturation_status.flags.yaw_neg = true;

		}

		// A positive change in thrust will increase saturation
		_saturation_status.flags.thrust_pos = true;

	}

	// The motor is saturated at the lower limit
	// check which control axes and which directions are contributing
	if (clipping_low) {
		// check if the roll input is saturating
		if (_rotors[index].roll_scale > 0.0f) {
			// A negative change in roll will increase saturation
			_saturation_status.flags.roll_neg = true;

		} else if (_rotors[index].roll_scale < 0.0f) {
			// A positive change in roll will increase saturation
			_saturation_status.flags.roll_pos = true;

		}

		// check if the pitch input is saturating
		if (_rotors[index].pitch_scale > 0.0f) {
			// A negative change in pitch will increase saturation
			_saturation_status.flags.pitch_neg = true;

		} else if (_rotors[index].pitch_scale < 0.0f) {
			// A positive change in pitch will increase saturation
			_saturation_status.flags.pitch_pos = true;

		}

		// check if the yaw input is saturating
		if (_rotors[index].yaw_scale > 0.0f) {
			// A negative change in yaw will increase saturation
			_saturation_status.flags.yaw_neg = true;

		} else if (_rotors[index].yaw_scale < 0.0f) {
			// A positive change in yaw will increase saturation
			_saturation_status.flags.yaw_pos = true;

		}

		// A negative change in thrust will increase saturation
		_saturation_status.flags.thrust_neg = true;

	}
}

void
MultirotorMixer::groups_required(uint32_t &groups)
{
	/* XXX for now, hardcoded to indexes 0-3 in control group zero */
	groups |= (1 << 0);
}

uint16_t MultirotorMixer::get_saturation_status()
{
	return _saturation_status.value;
}

#if defined(MIXER_TUNING)
#if !defined(MIXER_REMOTE)

int16_t
MultirotorMixer::get_parameter(mixer_param_s *param, uint16_t param_index)
{
	//Get subixer and parameter inde from linear index.
	param->mix_sub_index = 0;
	param->array_size = 1;
	param->type = MIXER_PARAM_MSG_TYPE_PARAMETER;
	param->flags = 0;   //ReadWrite

	switch (param_index) {
	case 0:
		strcpy(param->name, "MULTIROTOR");
		param->type = MIXER_PARAM_MSG_TYPE_MIXTYPE;
		param->array_size = 0;
		param->flags = 0x01;
		return 0;
		break;

	case 1:
		param->values[0] = _roll_scale;
		strncpy(param->name, "IN_ROLL_SCALE", 16);
		return 1;
		break;

	case 2:
		param->values[0] =  _pitch_scale;
		strncpy(param->name, "IN_PITCH_SCALE", 16);
		return 1;
		break;

	case 3:
		param->values[0] =  _yaw_scale;
		strncpy(param->name, "IN_YAW_SCALE", 16);
		return 1;
		break;

	case 4:
		param->values[0] =  _idle_speed;
		strncpy(param->name, "IN_IDLE_SPEED", 16);
		return 1;
		break;
	}

	param->mix_sub_index = 1;
	param_index -= 5;
	param->flags = 1;       //Read only

	while (param_index > 4) {
		param_index -= 5;
		param->mix_sub_index++;
	}

	if (param->mix_sub_index <= _rotor_count) {

		switch (param_index) {
		case 0:
			strcpy(param->name, "MULTIROTOR_MOTOR");
			param->type = MIXER_PARAM_MSG_TYPE_MIXTYPE;
			param->array_size = 0;
			param->flags = 0x01;
			return 0;
			break;

		case 1:
			param->values[0] =  _rotors[param->mix_sub_index - 1].roll_scale;
			strncpy(param->name, "OUT_ROLL_SCALE", 16);
			return 1;
			break;

		case 2:
			param->values[0] = _rotors[param->mix_sub_index - 1].pitch_scale;
			strncpy(param->name, "OUT_PITCH_SCALE", 16);
			return 1;
			break;

		case 3:
			param->values[0] = _rotors[param->mix_sub_index - 1].yaw_scale;
			strncpy(param->name, "OUT_YAW_SCALE", 16);
			return 1;
			break;

		case 4:
			param->values[0] = _rotors[param->mix_sub_index - 1].out_scale;
			strncpy(param->name, "OUT_SCALE", 16);
			return 1;
			break;
		}
	}

	param->array_size = 0;
	param->flags = 0x80;
	return -1;
}

int16_t
MultirotorMixer::set_parameter(mixer_param_s *param, uint16_t param_index)
{
	return set_param_value(param_index, 0, param->values[0]);
}

#endif  //MIXER_REMOTE


int16_t
MultirotorMixer::parameter_count()
{
	return 5 + _rotor_count * 5;
}

int16_t
MultirotorMixer::set_param_value(int16_t paramIndex, int16_t arrayIndex, float value)
{
	switch (paramIndex) {
	case 1:
		_roll_scale = value;
		break;

	case 2:
		_pitch_scale = value;
		break;

	case 3:
		_yaw_scale = value;
		break;

	case 4:
		_idle_speed = value;
		break;

	default:
		return -1;
		break;
	}

	return 0;
}

#endif //defined(MIXER_TUNING)
