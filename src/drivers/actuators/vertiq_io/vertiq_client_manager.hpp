/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
#ifndef VERTIQ_CLLIENT_MANAGER_HPP
#define VERTIQ_CLLIENT_MANAGER_HPP

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <parameters/param.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include "vertiq_serial_interface.hpp"
#include "ifci.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

#include "iq-module-communication-cpp/inc/esc_propeller_input_parser_client.hpp"

#ifdef CONFIG_USE_IFCI_CONFIGURATION
#include "iq-module-communication-cpp/inc/iquart_flight_controller_interface_client.hpp"
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
#include "iq-module-communication-cpp/inc/voltage_superposition_client.hpp"
#include "iq-module-communication-cpp/inc/pulsing_rectangular_input_parser_client.hpp"
#endif //CONFIG_USE_PULSING_CONFIGURATION

template <typename module_data_type, typename px4_data_type>
struct combo_entry {
	param_t _param;
	ClientEntry<module_data_type> *_entry;
	bool _needs_init;

	void ConfigureStruct(param_t parameter, ClientEntry<module_data_type> *entry)
	{
		_param = parameter;
		_entry = entry;
		_needs_init = true;
	}

	void SetNeedsInit()
	{
		_needs_init = true;
	}

	ClientEntry<module_data_type> *GetClientEntry()
	{
		return _entry;
	}
};

class VertiqClientManager
{
public:

	/**
	* @brief Construct a new VertiqClientManager object
	* @param serial_interface A pointer to a VertiqSerialInterface object
	*/
	VertiqClientManager(VertiqSerialInterface *serial_interface);

	/**
	* @brief Initialize all of our clients with the object ID given by the PX4 parameter TARGET_MODULE_ID
	*/
	void Init(uint8_t object_id);

	/**
	* @brief Handle the IQUART interface. Make sure that we update TX and RX buffers
	*/
	void HandleClientCommunication();

	/**
	* @brief Returns access to our IFCI interface
	* @return A pointer to our IFCI interface _motor_interface
	*/
	IFCI *GetMotorInterface();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to arm
	*/
	void SendSetForceArm();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to disarm
	*/
	void SendSetForceDisarm();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to coast
	*/
	void SendSetCoast();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to spin at a given setpoint
	* @param velocity_setpoint the raw 16-bit velocity command going to the motor
	*/
	void SendSetVelocitySetpoint(uint16_t velocity_setpoint);

	/**
	* @brief Returns the number of clients that we've initialized
	* @return The value contained in _clients_in_use
	*/
	uint8_t GetNumberOfClients();

	/**
	* @brief Send the connected module both a set and a save for a given IQUART entry
	* @param entry A pointer to the entry that you want to communicate with
	* @param value A pointer to a union that holds the value that we are setting in the conrrect format
	*/
	template <class module_data_type>
	void SendSetAndSave(ClientEntry<module_data_type> *entry, module_data_type value);

	/**
	* @brief Initializes the PX4 parameter version of an IQUART Entry to have the same value as is stored on the module. After setting
	*        this function lowers the flag indicating that we should initialize the value during updating
	* @param parameter The PX4 parameter we're editing
	* @param init_bool A pointer to the bool that we need to put down
	* @param value A pointer to a union that holds the value that we are setting in the conrrect format
	*/
	template <class px4_data_type>
	void InitParameter(param_t parameter, bool *init_bool, px4_data_type *value);

	/**
	* @brief Handles calling either InitParameter or SendSetAndSave depending on the state of the parameter combined_entry
	* @param combined_entry A pointer to the combo_entry that we're updating
	*/
	template <class module_data_type, class px4_data_type>
	void UpdateParameter(combo_entry<module_data_type, px4_data_type> *combined_entry);

	/**
	* @brief Set all of the IQUART configuration init flags to true
	*/
	void MarkIquartConfigsForRefresh();

	/**
	* @brief Send a Get command to all of the parameters involved in IQUART configuration, and make sure the PX4 parameters and module values agree
	*/
	void UpdateIquartConfigParams();

	/**
	* @brief Until the timeout is reached, keep trying to update the PX4 parameters to match, as appropraite, the value on the module. This can
	*	mean either setting the PX4 parameter to match the motor or vice versa
	*/
	void CoordinateIquartWithPx4Params(hrt_abstime timeout = 2_s);

private:

	void InitVertiqClients(uint8_t object_id);
	void InitComboEntries();

	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface *_serial_interface;

	//IQUART Client configuration
	IFCI _motor_interface;

	//These exist whenever we're using IQUART
	combo_entry<float, float> _velocity_max_entry;
	combo_entry<float, float> _voltage_max_entry;
	combo_entry<uint8_t, int32_t> _control_mode_entry;
	combo_entry<uint8_t, int32_t> _motor_direction_entry;
	combo_entry<uint8_t, int32_t> _fc_direction_entry;

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	combo_entry<uint8_t, int32_t> _throttle_cvi_entry;
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	combo_entry<uint8_t, int32_t> _pulsing_voltage_mode_entry;
	combo_entry<uint8_t, int32_t> _x_cvi_entry;
	combo_entry<uint8_t, int32_t> _y_cvi_entry;
	combo_entry<float, float> _pulse_zero_angle_entry;
	combo_entry<float, float> _pulse_velo_cutoff_entry;
	combo_entry<float, float> _pulse_torque_offset_entry;
	combo_entry<float, float> _pulse_volt_limit_entry;
#endif //CONFIG_USE_PULSING_CONFIGURATION

//Decide who to put in our array of entries. Using Pulsing will have the most, then IFCI, then only base IQUART by default
#ifdef CONFIG_USE_PULSING_CONFIGURATION
	static const uint8_t num_floats = 6;
	static const uint8_t num_uints = 7;
	combo_entry<float, float> *float_combo_entries[num_floats] = {&_velocity_max_entry, &_voltage_max_entry, &_pulse_zero_angle_entry, &_pulse_velo_cutoff_entry, &_pulse_torque_offset_entry, &_pulse_volt_limit_entry};
	combo_entry<uint8_t, int32_t> *uint8_combo_entries[num_uints] = {&_control_mode_entry, &_motor_direction_entry, &_fc_direction_entry, &_throttle_cvi_entry, &_pulsing_voltage_mode_entry, &_x_cvi_entry, &_y_cvi_entry};
#elif defined(CONFIG_USE_IFCI_CONFIGURATION)
	static const uint8_t num_floats = 2;
	static const uint8_t num_uints = 4;
	combo_entry<float, float> *float_combo_entries[num_floats] = {&_velocity_max_entry, &_voltage_max_entry};
	combo_entry<uint8_t, int32_t> *uint8_combo_entries[num_uints] = {&_control_mode_entry, &_motor_direction_entry, &_fc_direction_entry, &_throttle_cvi_entry};
#else
	static const uint8_t num_floats = 2;
	static const uint8_t num_uints = 3;
	combo_entry<float, float> *float_combo_entries[num_floats] = {&_velocity_max_entry, &_voltage_max_entry};
	combo_entry<uint8_t, int32_t> *uint8_combo_entries[num_uints] = {&_control_mode_entry, &_motor_direction_entry, &_fc_direction_entry};
#endif

	//Vertiq client information
	//Some constants to help us out
	static const uint8_t _kBroadcastID = 63;
	static const uint8_t MINIMUM_NUM_CLIENTS = 2;
	static const uint8_t MAXIMUM_NUM_CLIENTS = 15;

	//Array information
	ClientAbstract *_client_array[MAXIMUM_NUM_CLIENTS];
	uint8_t _clients_in_use = MINIMUM_NUM_CLIENTS;

	//Clients
	PropellerMotorControlClient _broadcast_prop_motor_control;
	ArmingHandlerClient _broadcast_arming_handler;

	EscPropellerInputParserClient *_prop_input_parser_client;

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	//Make all of the clients that we need to talk to the IFCI config params
	IQUartFlightControllerInterfaceClient *_ifci_client;
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	VoltageSuperPositionClient *_voltage_superposition_client;
	PulsingRectangularInputParserClient *_pulsing_rectangular_input_parser_client;
#endif //CONFIG_USE_PULSING_CONFIGURATION
};

#endif
