
#include "vertiq_io.hpp"

#include <px4_platform_common/log.h>

px4::atomic_bool VertiqIo::_request_telemetry_init{false};
char VertiqIo::_telemetry_device[] {};

VertiqIo::VertiqIo() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_telem_manager(&_motor_interface),
	_serial_interface(NUM_CLIENTS),
	_broadcast_prop_motor_control(_kBroadcastID), //Initialize with a module ID of 63 for broadcasting
	_arming_handler(_kBroadcastID)

{
	_client_array[0] = &_broadcast_prop_motor_control;
	_client_array[1] = &_arming_handler;
	//Make sure we get the correct initial values for our parameters
	update_params();
}

VertiqIo::~VertiqIo()
{
	//Free our counters/timers
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

//called by our task_spawn function
bool VertiqIo::init()
{
	//Grab the number of IFCI control values the user wants to use
        _cvs_in_use = (uint8_t)_param_vertiq_number_of_cvs.get();

	//Grab the bitmask that we're going to use to decide who we get telemetry from
	_telem_bitmask = (uint16_t)_param_vertiq_telemetry_mask.get();

	//Initialize our telemetry handler
	_telem_manager.Init(_telem_bitmask);
	_telem_manager.StartPublishing(&_esc_status_pub);

	//Make sure we get our thread into execution
	ScheduleNow();

	return true;
}

//This is the same as a while(1) loop. Gets called at a set interval, or
//is triggered by some uORB publication
void VertiqIo::Run()
{
	//Start the loop timer
	//Increment our loop counter
	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//If we should leave, then clean up our mess and get out
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// telemetry device update request?
	//This happens whenever the module is started
	if (_request_telemetry_init.load()) {
		_serial_interface.init_serial(_telemetry_device, _param_vertiq_baud.get());
		_request_telemetry_init.store(false);
	}

	//Handle IQUART reception and transmission
	handle_iquart();

	//If we're supposed to ask for telemetry from someone
	if(_telem_bitmask){
		//Grab the next ID, and if it's real get it ready to send out
		uint16_t next_telem = _telem_manager.UpdateTelemetry();//update_telemetry();
		if(next_telem != _impossible_module_id){
			_telemetry_request_id = next_telem;
		}
	}

	//Get the most up to date version of our parameters
	update_params();

	//Make sure we also update the mixing output to get the most up to date configuration
	_mixing_output.update();
	_mixing_output.updateSubscriptions(true);

	//stop our timer
	perf_end(_loop_perf);
}

int VertiqIo::task_spawn(int argc, char *argv[])
{
	VertiqIo *instance = new VertiqIo();

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

int VertiqIo::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	_mixing_output.printStatus();
	return 0;
}

int VertiqIo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("vertiq_io", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int VertiqIo::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "telemetry")) {
		if (argc > 1) {
			// telemetry can be requested before the module is started
			strncpy(_telemetry_device, argv[1], sizeof(_telemetry_device) - 1);
			_telemetry_device[sizeof(_telemetry_device) - 1] = '\0';
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	return print_usage("unknown command");
}

void VertiqIo::handle_iquart()
{
	//Update our serial rx
	_serial_interface.process_serial_rx(&_motor_interface, _client_array);

	//Update our serial tx
	_serial_interface.process_serial_tx();
}

void VertiqIo::update_params()
{
	updateParams();
}

bool VertiqIo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			     unsigned num_control_groups_updated)
{
	if(_mixing_output.armed().armed){

		if(_param_vertiq_arm_behavior.get() == FORCE_ARMING && _send_forced_arm){
			_arming_handler.motor_armed_.set(*_serial_interface.get_iquart_interface(), 1);
			_send_forced_arm = false;
		}

		//We already get a mixer value from [0, 65535]. We can send that right to the motor, and let the input parser handle
		//conversions
		_motor_interface.BroadcastPackedControlMessage(*_serial_interface.get_iquart_interface(), outputs, _cvs_in_use, _telemetry_request_id);

		//We want to make sure that we send a valid telem request only once to ensure that we're not getting extraneous responses.
		//So, here we'll set the telem request ID to something that no one will respond to. Another function will take charge of setting it to a
		//proper value when necessary
		_telemetry_request_id = _impossible_module_id;
	}else{
		//Put the modules into coast
		switch(_param_vertiq_disarm_behavior.get()){
			case TRIGGER_MOTOR_DISARM:
				_arming_handler.motor_armed_.set(*_serial_interface.get_iquart_interface(), 0);
			break;

			case COAST_MOTOR:
				_broadcast_prop_motor_control.ctrl_coast_.set(*_serial_interface.get_iquart_interface());
			break;

			case SEND_PREDEFINED_THROTTLE:
				_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface.get_iquart_interface(), _param_vertiq_disarm_throttle.get());
			break;

			default:
			break;
		}

		if(!_send_forced_arm){
			_send_forced_arm = true;
		}
	}

	//Publish our esc status to uORB
	_esc_status_pub.publish(_telem_manager.GetEscStatus());

	return true;
}


extern "C" __EXPORT int vertiq_io_main(int argc, char *argv[])
{
	return VertiqIo::main(argc, argv);
}
