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
 * @file PerformanceModel.cpp
 * Performance model.
 */

#include <geo/geo.h>
#include <px4_platform_common/events.h>
#include "PerformanceModel.h"

// air density of standard athmosphere at 5000m above mean sea level [kg/m^3]
static constexpr float kAirDensityStandardAtmos5000Amsl = 0.7363f;

// air density of standard athmosphere at 1000m above mean sea level [kg/m^3]
static constexpr float kAirDensityStandardAtmos1000Amsl = 1.112f;

// [.] minimum ratio between the actual vehicle weight and the vehicle nominal weight (weight at which the performance limits are derived)
static constexpr float kMinWeightRatio = 0.5f;

// [.] maximum ratio between the actual vehicle weight and the vehicle nominal weight (weight at which the performance limits are derived)
static constexpr float kMaxWeightRatio = 2.0f;

// climbrate defining the service ceiling, used to compensate max climbrate based on air density
static constexpr float kClimbrateMin = 0.5f; // [m/s]

PerformanceModel::PerformanceModel(): ModuleParams(nullptr)
{
	updateParams();
}
float PerformanceModel::getWeightRatio() const
{
	float weight_factor = 1.0f;

	if (_param_weight_base.get() > FLT_EPSILON && _param_weight_gross.get() > FLT_EPSILON) {
		weight_factor = math::constrain(_param_weight_gross.get() / _param_weight_base.get(), kMinWeightRatio,
						kMaxWeightRatio);
	}

	return weight_factor;
}
float PerformanceModel::getMaximumClimbRate(float air_density) const
{

	float climbrate_max = _param_fw_t_clmb_max.get();

	const float density_min = _param_density_min.get();

	if (density_min < kAirDensityStandardAtmos1000Amsl
	    && density_min > kAirDensityStandardAtmos5000Amsl) {
		const float density_gradient = math::max((_param_fw_t_clmb_max.get() - kClimbrateMin) /
					       (CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C -
						density_min), 0.0f);
		const float delta_rho = air_density - CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C;
		climbrate_max = _param_fw_t_clmb_max.get() + density_gradient * delta_rho;
	}

	return climbrate_max / getWeightRatio();
}
float PerformanceModel::getTrimThrottle(float throttle_min, float throttle_max, float airspeed_sp,
					float air_density) const
{
	const float throttle_trim = getTrimThrottleForAirspeed(airspeed_sp) * getAirDensityThrottleScale(
					    air_density) * getWeightThrottleScale();
	return math::constrain(throttle_trim, throttle_min, throttle_max);
}
float PerformanceModel::getWeightThrottleScale() const
{
	return sqrtf(getWeightRatio());
}

float PerformanceModel::getAirDensityThrottleScale(float air_density) const
{
	float air_density_throttle_scale = 1.0f;

	if (PX4_ISFINITE(air_density)) {
		// scale throttle as a function of sqrt(rho0/rho)
		const float eas2tas = sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / air_density);
		const float eas2tas_at_5000m_amsl = sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / kAirDensityStandardAtmos5000Amsl);
		air_density_throttle_scale = math::constrain(eas2tas, 1.f, eas2tas_at_5000m_amsl);
	}

	return air_density_throttle_scale;
}
float PerformanceModel::getTrimThrottleForAirspeed(float airspeed_sp) const
{
	float throttle_trim =
		_param_fw_thr_trim.get(); // throttle required for level flight at trim airspeed, at sea level (standard atmosphere)

	// Drag modelling (parasite drag): calculate mapping airspeed-->throttle, assuming a linear relation with different gradients
// above and below trim. This is tunable thorugh FW_THR_ASPD_MIN and FW_THR_ASPD_MAX.
	const float slope_below_trim = (_param_fw_thr_trim.get() - _param_fw_thr_aspd_min.get()) /
				       (_param_fw_airspd_trim.get() - _param_fw_airspd_min.get());
	const float slope_above_trim = (_param_fw_thr_aspd_max.get() - _param_fw_thr_trim.get()) /
				       (_param_fw_airspd_max.get() - _param_fw_airspd_trim.get());

	if (PX4_ISFINITE(airspeed_sp) && PX4_ISFINITE(slope_below_trim) && _param_fw_thr_aspd_min.get() > FLT_EPSILON
	    && airspeed_sp < _param_fw_airspd_trim.get()) {
		throttle_trim = _param_fw_thr_trim.get() - slope_below_trim * (_param_fw_airspd_trim.get() - airspeed_sp);

	} else if (PX4_ISFINITE(airspeed_sp) && PX4_ISFINITE(slope_above_trim) && _param_fw_thr_aspd_max.get() > FLT_EPSILON
		   && airspeed_sp > _param_fw_airspd_trim.get()) {
		throttle_trim = _param_fw_thr_trim.get() + slope_above_trim * (airspeed_sp - _param_fw_airspd_trim.get());
	}

	return throttle_trim;
}
float PerformanceModel::getMinimumSinkRate(float air_density) const
{
	return _param_fw_t_sink_min.get() * sqrtf(getWeightRatio() * CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / air_density);
}
float PerformanceModel::getTrimAirspeed() const
{
	return math::constrain(_param_fw_airspd_trim.get() * sqrtf(getWeightRatio()), _param_fw_airspd_min.get(),
			       _param_fw_airspd_max.get());
}
float PerformanceModel::getMinimumAirspeed(float load_factor) const
{
	return _param_fw_airspd_min.get() * sqrtf(getWeightRatio() * load_factor);
}

float PerformanceModel::getStallAirspeed(float load_factor) const
{
	return _param_fw_airspd_stall.get() * sqrtf(getWeightRatio() * load_factor);
}

float PerformanceModel::getMaximumAirspeed() const
{
	return _param_fw_airspd_max.get();
}
bool PerformanceModel::runSanityChecks() const
{
	bool ret = true;

	// sanity check parameters
	if (_param_fw_airspd_max.get() < _param_fw_airspd_min.get()) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MAX</param>: {1:.1}
		 * - <param>FW_AIRSPD_MIN</param>: {2:.1}
		 */
		events::send<float, float>(events::ID("fixedwing_position_control_conf_invalid_airspeed"), events::Log::Error,
					   "Invalid configuration: Airspeed max smaller than min",
					   _param_fw_airspd_max.get(), _param_fw_airspd_min.get());
		ret = false;
	}

	if (_param_fw_airspd_max.get() < 5.0f || _param_fw_airspd_min.get() > 100.0f) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MAX</param>: {1:.1}
		 * - <param>FW_AIRSPD_MIN</param>: {2:.1}
		 */
		events::send<float, float>(events::ID("fixedwing_position_control_conf_invalid_airspeed_bounds"), events::Log::Error,
					   "Invalid configuration: Airspeed max \\< 5 m/s or min \\> 100 m/s",
					   _param_fw_airspd_max.get(), _param_fw_airspd_min.get());
		ret = false;
	}

	if (_param_fw_airspd_trim.get() < _param_fw_airspd_min.get() ||
	    _param_fw_airspd_trim.get() > _param_fw_airspd_max.get()) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MAX</param>: {1:.1}
		 * - <param>FW_AIRSPD_MIN</param>: {2:.1}
		 * - <param>FW_AIRSPD_TRIM</param>: {3:.1}
		 */
		events::send<float, float, float>(events::ID("fixedwing_position_control_conf_invalid_trim_bounds"),
						  events::Log::Error,
						  "Invalid configuration: Airspeed trim out of min or max bounds",
						  _param_fw_airspd_max.get(), _param_fw_airspd_min.get(), _param_fw_airspd_trim.get());
		ret = false;
	}

	if (_param_fw_airspd_stall.get() > _param_fw_airspd_min.get()) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MIN</param>: {1:.1}
		 * - <param>FW_AIRSPD_STALL</param>: {2:.1}
		 */
		events::send<float, float>(events::ID("fixedwing_position_control_conf_invalid_stall"), events::Log::Error,
					   "Invalid configuration: FW_AIRSPD_STALL higher FW_AIRSPD_MIN",
					   _param_fw_airspd_min.get(), _param_fw_airspd_stall.get());
		ret = false;
	}

	return ret;

}
void PerformanceModel::	updateParameters()
{
	updateParams();
}
