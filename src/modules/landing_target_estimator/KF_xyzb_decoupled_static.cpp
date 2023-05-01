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
 * @file KF_xyzb_decoupled_static.cpp
 * @brief Filter to estimate the pose of static targets. State: [r, r_dot, bias]
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include "KF_xyzb_decoupled_static.h"
#include "python_derivation/generated/decoupled_static_xyzb/predictCov.h"
#include "python_derivation/generated/decoupled_static_xyzb/computeInnovCov.h"

namespace landing_target_estimator
{

void KF_xyzb_decoupled_static::predictState(float dt, float acc)
{
	_state(0, 0) = _state(0, 0) + _state(1, 0) * dt - 0.5f * acc * dt * dt;
	_state(1, 0) = _state(1, 0) - acc * dt;
}

void KF_xyzb_decoupled_static::predictCov(float dt)
{
	matrix::Matrix<float, 3, 3> cov_updated;
	sym::Predictcov(dt, _input_var, _bias_var, _covariance, &cov_updated);
	_covariance = cov_updated;
}


bool KF_xyzb_decoupled_static::update()
{
	// Avoid zero-division
	if (fabsf(_innov_cov) < 1e-6f) {
		return false;
	}

	const float beta = _innov / _innov_cov * _innov;

	// 5% false alarm probability
	if (beta > 3.84f) {
		return false;
	}

	const matrix::Matrix<float, 3, 1> kalmanGain = _covariance * _meas_matrix.transpose() / _innov_cov;

	_state = _state + kalmanGain * _innov;
	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KF_xyzb_decoupled_static::setH(matrix::Vector<float, 15> h_meas, int direction)
{
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz, vtx, vty, vtz]
	// idx    = [0,   1,  2,      3,      4,      5,  6,  7,  8,   9,  10,  11,  12,  13,  14]

	if (direction == Directions::x) {
		_meas_matrix(0, 0) = h_meas(0);
		_meas_matrix(0, 1) = h_meas(3);
		_meas_matrix(0, 2) = h_meas(6);

	} else if (direction == Directions::y) {
		_meas_matrix(0, 0) = h_meas(1);
		_meas_matrix(0, 1) = h_meas(4);
		_meas_matrix(0, 2) = h_meas(7);

	} else {
		_meas_matrix(0, 0) = h_meas(2);
		_meas_matrix(0, 1) = h_meas(5);
		_meas_matrix(0, 2) = h_meas(8);
	}
}

void KF_xyzb_decoupled_static::syncState(float dt, float acc)
{
	_sync_state(0, 0) = _state(0, 0) - _state(1, 0) * dt - 0.5f * acc * dt * dt;
	_sync_state(1, 0) = _state(1, 0) + acc * dt;
	_sync_state(2, 0) = _state(2, 0);
}

float KF_xyzb_decoupled_static::computeInnovCov(float meas_unc)
{
	float innov_cov_updated;
	sym::Computeinnovcov(meas_unc, _covariance, _meas_matrix, &innov_cov_updated);
	_innov_cov = innov_cov_updated;

	return _innov_cov;
}

float KF_xyzb_decoupled_static::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix * _sync_state)(0, 0);
	return _innov;
}

} // namespace landing_target_estimator
