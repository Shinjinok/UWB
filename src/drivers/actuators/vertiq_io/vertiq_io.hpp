/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_test.h>

#include "vertiq_telemetry_manager.hpp"
#include "vertiq_client_manager.hpp"
#include "vertiq_serial_interface.hpp"
#include "vertiq_configuration_handler.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

#include "iq-module-communication-cpp/inc/esc_propeller_input_parser_client.hpp"
#include "iq-module-communication-cpp/inc/iquart_flight_controller_interface_client.hpp"

#ifdef CONFIG_USE_PULSING_CONFIGURATION
#include "iq-module-communication-cpp/inc/voltage_superposition_client.hpp"
#include "iq-module-communication-cpp/inc/pulsing_rectangular_input_parser_client.hpp"
#endif //CONFIG_USE_PULSING_CONFIGURATION

enum DISARM_BEHAVIORS {TRIGGER_MOTOR_DISARM, COAST_MOTOR, SEND_PREDEFINED_THROTTLE};
enum ARM_BEHAVIORS {USE_MOTOR_ARMING, FORCE_ARMING};

class VertiqIo : public ModuleBase<VertiqIo>, public OutputModuleInterface
{

public:
	/**
	* @brief Create a new VertiqIo object
	*/
	VertiqIo();

	/**
	* @brief destruct a VertiqIo object
	*/
	~VertiqIo() override;

	/**
	* @brief initialize the VertiqIo object. This will be called by the task_spawn function. Makes sure that the thread gets scheduled.
	*/
	bool init();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	/** @see ModuleBase::run() */ //I do not think this actually comes from ModuleBase. it should come from scheduled work item
	void Run() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	/**
	* @brief Used to package and transmit controls via IQUART
	* @param outputs The output throttles calculated by the mixer
	*/
	void OutputControls(uint16_t outputs[MAX_ACTUATORS]);

private:

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")};

	static const uint8_t MAX_SUPPORTABLE_IFCI_CVS = 16;

	static px4::atomic_bool _request_telemetry_init; //Determines whether or not we should initialize or re-initialize the serial connection

	MixingOutput _mixing_output{"VERTIQ_IO", MAX_SUPPORTABLE_IFCI_CVS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	static char _telemetry_device[20]; //The name of the device we're connecting to. this will be something like /dev/ttyS3

	VertiqSerialInterface _serial_interface; //We need a serial handler in order to talk over the serial port
	VertiqClientManager _client_manager; //We need someone who can manage our clients
	VertiqTelemetryManager _telem_manager; //We need a telemetry handler
	VertiqConfigurationHandler _configuration_handler;

////////////////////////////////////////////////////////////////////////
//Vertiq client information

/**
In order to communicate with connected Vertiq modules with the most possible flexibility, we have introduced two types of Clients (see comment above about
Vertiq clients and entries) to be used within PX4. We have designated "Operational Clients" as those whose object IDs (a.k.a target module IDs) are constant.
For example, when sending IFCI commands, we are always transmitting them using the object ID 63 in order to broadcast to all connected Vertiq modules. There is no
reason to change its object ID. In order to interact with specific client entries from specific modules, we have introduced "Configuration Clients." Configuration
clients are those whose object IDs are not constant, and which are created and destroyed as the Target Module ID parameter is updated. An example of a configuration
parameter is each module's Velocity Max entry. Since each module has its own version of a Velocity Max entry, designated by unique module IDs, we need a way to
dynamically change the client's object ID to reach the correct module. Our "Configuration Clients" are used to meet this end.

An example of a Vertiq client is documented here https://iqmotion.readthedocs.io/en/latest/modules/vertiq_2306_2200.html#propeller-motor-control. In this case,
Propeller Motor Control is the client, and its entries are specified in the message table (https://iqmotion.readthedocs.io/en/latest/modules/vertiq_2306_2200.html#id6).
You can find the C++ representation in ./src/drivers/actuators/vertiq_io/iq-module-communication-cpp/inc/propeller_motor_control_client.hpp
*/

	//Known Operational Clients can be created as concrete objects
	PropellerMotorControlClient _broadcast_prop_motor_control;
	ArmingHandlerClient _broadcast_arming_handler;
	IQUartFlightControllerInterfaceClient _operational_ifci;
	IFCIPackedMessage _transmission_message;
	static const uint16_t MAX_IFCI_MESSAGE = 40; //Up to 16 2 byte commands, one telemetry byte, plus 7 IQUART added bytes
	uint8_t _output_message[MAX_IFCI_MESSAGE];
	uint8_t _output_len;
////////////////////////////////////////////////////////////////////////

	uint8_t _cvs_in_use = 0; //Store the number of control variables that we're using

	//Store the telemetry bitmask for who we want to get telemetry from
	uint64_t _telem_bitmask = 0;
	uint32_t _telemetry_ids_1 = 0;
	uint32_t _telemetry_ids_2 = 0;

	bool _send_forced_arm = true;
	bool _actuator_test_active = false;

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)}; //We want to publish our ESC Status to anyone who will listen

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	//We need to know what's going on with the actuator test to make sure we handle it properly
	uORB::Subscription _actuator_test_sub{ORB_ID(actuator_test)};
	actuator_test_s _actuator_test;

	//We need to bring in the parameters that we define in module.yaml in order to view them in the
	//control station, as well as to use them in the firmware
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VERTIQ_BAUD>) _param_vertiq_baud,
		(ParamInt<px4::params::TARGET_MODULE_ID>) _param_vertiq_target_module_id,
		(ParamBool<px4::params::TRIGGER_READ>) _param_vertiq_trigger_read,
		(ParamInt<px4::params::CONTROL_MODE>) _param_vertiq_control_mode,
		(ParamFloat<px4::params::MAX_VELOCITY>) _param_vertiq_max_velo,
		(ParamFloat<px4::params::MAX_VOLTS>) _param_vertiq_max_volts,
		(ParamInt<px4::params::VERTIQ_MOTOR_DIR>) _param_vertiq_motor_direction,
		(ParamInt<px4::params::VERTIQ_FC_DIR>) _param_vertiq_fc_direction
#ifdef CONFIG_USE_IFCI_CONFIGURATION
		, (ParamInt<px4::params::VERTIQ_NUM_CVS>) _param_vertiq_number_of_cvs
		, (ParamInt<px4::params::DISARM_THROT>) _param_vertiq_disarm_throttle
		, (ParamInt<px4::params::DISARM_TRIGGER>) _param_vertiq_disarm_behavior
		, (ParamInt<px4::params::ARMING_BEHAVE>) _param_vertiq_arm_behavior
		, (ParamInt<px4::params::THROTTLE_CVI>) _param_vertiq_throttle_cvi
		, (ParamInt<px4::params::TELEM_IDS_1>) _param_vertiq_telem_ids_1
		, (ParamInt<px4::params::TELEM_IDS_2>) _param_vertiq_telem_ids_2
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
		, (ParamInt<px4::params::PULSE_VOLT_MODE>) _param_vertiq_pulse_volt_mode
		, (ParamInt<px4::params::X_CVI>) _param_vertiq_pulse_x_cvi
		, (ParamInt<px4::params::Y_CVI>) _param_vertiq_pulse_y_cvi
		, (ParamFloat<px4::params::ZERO_ANGLE>) _param_vertiq_pulse_zero_angle
		, (ParamFloat<px4::params::VELOCITY_CUTOFF>) _param_vertiq_pulse_velo_cutoff
		, (ParamFloat<px4::params::TORQUE_OFF_ANGLE>) _param_vertiq_pulse_torque_offset_angle
		, (ParamFloat<px4::params::PULSE_VOLT_LIM>) _param_vertiq_pulse_voltage_limit
#endif //CONFIG_USE_PULSING_CONFIGURATION
	)
};




