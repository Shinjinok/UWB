#include "analog_battery.h"

// Defaults to use if the parameters are not set
#if BOARD_NUMBER_BRICKS > 0
#if defined(BOARD_BATT_V_LIST) && defined(BOARD_BATT_I_LIST)
static constexpr int   DEFAULT_V_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
static constexpr int   DEFAULT_I_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;
#else
static constexpr int   DEFAULT_V_CHANNEL[BOARD_NUMBER_BRICKS] = {0};
static constexpr int   DEFAULT_I_CHANNEL[BOARD_NUMBER_BRICKS] = {0};
#endif
#else
static constexpr int DEFAULT_V_CHANNEL[1] = {0};
static constexpr int DEFAULT_I_CHANNEL[1] = {0};
#endif

AnalogBattery::AnalogBattery() :
	BatteryBase(),
	ModuleParams(nullptr)
{}

void
AnalogBattery::updateBatteryStatusRawADC(int32_t voltage_raw, int32_t current_raw, hrt_abstime timestamp,
		bool selected_source, int priority,
		float throttle_normalized,
		bool armed)
{
	updateParams();

	float voltage_v = (voltage_raw * _get_cnt_v_volt()) * _get_v_div();
	float current_a = ((current_raw * _get_cnt_v_curr()) - _get_v_offs_cur()) * _get_a_per_v();

	bool connected = voltage_v > BOARD_ADC_OPEN_CIRCUIT_V &&
			 (BOARD_ADC_OPEN_CIRCUIT_V <= BOARD_VALID_UV || is_valid());

	updateBatteryStatus(voltage_v, current_a, timestamp, selected_source, priority, throttle_normalized, armed, connected);
}

/**
 * Whether the ADC channel for the voltage of this battery is valid.
 * Corresponds to BOARD_BRICK_VALID_LIST
 */
bool AnalogBattery::is_valid()
{
#ifdef BOARD_BRICK_VALID_LIST
	bool valid[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
	return valid[_get_brick_index()];
#else
	// TODO: Maybe return false instead?
	return true;
#endif
}

int AnalogBattery::getVoltageChannel()
{
	if (_get_adc_channel() >= 0) {
		return _get_adc_channel();

	} else {
		return DEFAULT_V_CHANNEL[_get_brick_index()];
	}
}

int AnalogBattery::getCurrentChannel()
{
	// TODO: Possibly implement parameter for current sense channel
	return DEFAULT_I_CHANNEL[_get_brick_index()];
}

float
AnalogBattery::_get_cnt_v_volt()
{
	float val = _get_cnt_v_volt_raw();


	if (val < 0.0f) {
		// TODO: This magic constant was hardcoded into sensors.cpp before I did the refactor. I don't know
		//  what the best way is to make it not a magic number.
		return 3.3f / 4096.0f;

	} else {
		return val;
	}
}

float
AnalogBattery::_get_cnt_v_curr()
{
	float val = _get_cnt_v_curr_raw();

	if (val < 0.0f) {
		// TODO: Same magic number problem as above
		return 3.3f / 4096.0f;

	} else {
		return val;
	}
}

float
AnalogBattery::_get_v_div()
{
	float val = _get_v_div_raw();

	if (val <= 0.0f) {
		return BOARD_BATTERY1_V_DIV;

	} else {
		return val;
	}
}

float
AnalogBattery::_get_a_per_v()
{
	float val = _get_a_per_v_raw();

	if (val <= 0.0f) {
		return BOARD_BATTERY1_A_PER_V;

	} else {
		return val;
	}
}