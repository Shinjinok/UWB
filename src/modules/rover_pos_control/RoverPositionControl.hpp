/****************************************************************************
 *
 *   Copyright (c) 2017, 2021 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


#include <float.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/pid/pid.h>
#include <lib/rate_control/rate_control.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using matrix::Dcmf;

using namespace time_literals;

// Main app function that gets called by NuttX when module is called
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);

class RoverPositionControl final : public ModuleBase<RoverPositionControl>, public ModuleParams, public px4::WorkItem
{
public:
	RoverPositionControl();
	~RoverPositionControl();
	RoverPositionControl(const RoverPositionControl &) = delete;
	RoverPositionControl operator=(const RoverPositionControl &other) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	// Main function to run the module
	void Run() override;

	void parameters_update(bool force = false);

	// Poll functions
	void	position_setpoint_triplet_poll();
	void	attitude_setpoint_poll();
	void	rates_setpoint_poll();
	void	vehicle_control_mode_poll();
	void 	vehicle_attitude_poll();
	void	vehicle_angular_acceleration_poll();
	void	manual_control_setpoint_poll();

	/**
	 * @brief Apply position control
	 *
	 * @param global_pos
	 * @param ground_speed
	 * @param _pos_sp_triplet
	 * @return true
	 * @return false
	 */
	bool	control_position(const matrix::Vector2d &global_pos, const matrix::Vector3f &ground_speed,
				 const position_setpoint_triplet_s &_pos_sp_triplet);

	/**
	 * @brief Apply velocity control to reach the trajectory setpoint's velocity setpoint
	 */
	void	control_velocity(const matrix::Vector3f &current_velocity, const trajectory_setpoint_s &trajectory_setpoint);

	/**
	 * @brief Apply attitude control
	 *
	 * @param att
	 * @param att_sp
	 */
	void	control_attitude(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp);

	/**
	 * @brief Apply rate control
	 *
	 * @param rates
	 * @param acc
	 * @param local_pos
	 * @param rates_sp
	 */
	void	control_rates(const vehicle_angular_velocity_s &rates, const  vehicle_angular_acceleration_s &acc,
			      const vehicle_local_position_s &local_pos,
			      const vehicle_rates_setpoint_s &rates_sp);

	// Print rate control status
	void	rate_control_status();

	// Subscription
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)}; /**< control mode subscription */
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; /**< notification of manual control updates */
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
	uORB::SubscriptionData<vehicle_acceleration_s>		_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	// Publication
	uORB::Publication<vehicle_rates_setpoint_s>	_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<position_controller_status_s>	_pos_ctrl_status_pub{ORB_ID(position_controller_status)};  /**< navigation capabilities publication */
	uORB::Publication<actuator_controls_s>		_actuator_controls_pub{ORB_ID(actuator_controls_0)};  /**< actuator controls publication */
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	// Struct / enum definitions
	enum POS_CTRLSTATES {
		GOTO_WAYPOINT,
		STOPPING
	} _pos_ctrl_state {STOPPING}; /// Position control state machine

	enum class VelocityFrame {
		NED,
		BODY,
	} _velocity_frame{VelocityFrame::NED};

	// Internal variables
	manual_control_setpoint_s		_manual_control_setpoint{};			    /**< r/c channel data */
	position_setpoint_triplet_s		_pos_sp_triplet{};		/**< triplet of mission items */
	vehicle_attitude_setpoint_s		_att_sp{};			/**< attitude setpoint > */
	vehicle_rates_setpoint_s		_rates_sp{};			/**< rate setpoint > */
	vehicle_control_mode_s			_control_mode{};		/**< control mode */
	vehicle_global_position_s		_global_pos{};			/**< global vehicle position */
	vehicle_local_position_s		_local_pos{};			/**< global vehicle position */
	actuator_controls_s				_act_controls{};		/**< direct control of actuators */
	vehicle_attitude_s				_vehicle_att{};
	vehicle_angular_acceleration_s		_vehicle_angular_acceleration{};
	vehicle_angular_velocity_s _vehicle_rates{};

	trajectory_setpoint_s _trajectory_setpoint{};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	vehicle_angular_velocity_s 		_vehicle_rates{};
	vehicle_angular_acceleration_s 		_vehicle_angular_acceleration{};

	MapProjection _global_local_proj_ref{};
	float _global_local_alt0{NAN};

	matrix::Vector2d _prev_wp{0, 0}; // Previous waypoint
	float _manual_yaw_sp{0.0};
	bool _reset_yaw_sp{true};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	hrt_abstime _control_position_last_called{0}; 	/**<last call of control_position  */
	hrt_abstime _control_rates_last_called{0};
	hrt_abstime _manual_setpoint_last_called{0};

	// Position control
	ECL_L1_Pos_Controller _gnd_control;

	// Velocity control
	// Pid controller for the speed (based on speed setpoint error) The output gets scaled and gets applied as throttle.
	PID_t _speed_ctrl{};

	// estimator reset counters
	uint8_t _pos_reset_counter{0};		// captures the number of times the estimator has reset the horizontal position
	ECL_L1_Pos_Controller				_gnd_control;
	RateControl				_rate_control;
	float _steering_input{0.0};

	enum UGV_POSCTRL_MODE {
		UGV_POSCTRL_MODE_AUTO,
		UGV_POSCTRL_MODE_OTHER
	} _control_mode_current{UGV_POSCTRL_MODE_OTHER};			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.


	enum POS_CTRLSTATES {
		GOTO_WAYPOINT,
		STOPPING
	} _pos_ctrl_state {STOPPING};			/// Position control state machine

	/* previous waypoint */
	matrix::Vector2d _prev_wp{0, 0};

	enum class VelocityFrame {
		NED,
		BODY,
	} _velocity_frame{VelocityFrame::NED};
	// Attitude control
	// Rate control

	DEFINE_PARAMETERS(
		// L1 guidance
		(ParamFloat<px4::params::GND_L1_PERIOD>) _param_l1_period,
		(ParamFloat<px4::params::GND_L1_DAMPING>) _param_l1_damping,
		(ParamFloat<px4::params::GND_L1_DIST>) _param_l1_distance,

		// Position control
		(ParamFloat<px4::params::GND_SPEED_TRIM>) _param_gndspeed_trim,
		(ParamFloat<px4::params::GND_SPEED_MAX>) _param_gndspeed_max,
		(ParamFloat<px4::params::GND_SPEED_MIN>) _param_gndspeed_min,

		// Velocity control
		(ParamInt<px4::params::GND_SP_CTRL_MODE>) _param_speed_control_mode,
		(ParamFloat<px4::params::GND_SPEED_P>) _param_speed_p,
		(ParamFloat<px4::params::GND_SPEED_I>) _param_speed_i,
		(ParamFloat<px4::params::GND_SPEED_D>) _param_speed_d,
		(ParamFloat<px4::params::GND_SPEED_IMAX>) _param_speed_imax,
		(ParamFloat<px4::params::GND_SPEED_THR_SC>) _param_throttle_speed_scaler,

		// Attitude control
		(ParamFloat<px4::params::GND_ATT_P>) _param_att_p,

		// Rate control
		(ParamFloat<px4::params::GND_RATE_P>) _param_rate_p,
		(ParamFloat<px4::params::GND_RATE_I>) _param_rate_i,
		(ParamFloat<px4::params::GND_RATE_D>) _param_rate_d,
		(ParamFloat<px4::params::GND_RATE_FF>) _param_rate_ff,
		(ParamFloat<px4::params::GND_RATE_IMAX>) _param_rate_imax,
		(ParamFloat<px4::params::GND_RATE_MAX>) _param_rate_max,
		(ParamFloat<px4::params::GND_RATE_IMINSPD>) _param_rate_i_minspeed,

		// Throttle settings
		(ParamFloat<px4::params::GND_THR_MIN>) _param_throttle_min,
		(ParamFloat<px4::params::GND_THR_MAX>) _param_throttle_max,
		(ParamFloat<px4::params::GND_THR_CRUISE>) _param_throttle_cruise,

		// Rover geometry
		(ParamFloat<px4::params::GND_WHEEL_BASE>) _param_wheel_base,
		(ParamFloat<px4::params::GND_MAX_ANG>) _param_max_turn_angle,

		// Other
		(ParamFloat<px4::params::GND_MAN_Y_MAX>) _param_gnd_man_y_max,
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad
	)
};
