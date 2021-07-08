/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file lps22hb_i2c.cpp
 *
 * I2C interface for lps22hb
 */

#include "LPS22HB.hpp"

#include <drivers/device/i2c.h>

device::Device *LPS22HB_I2C_interface(int bus, int bus_frequency);

class LPS22HB_I2C : public device::I2C
{
public:
	LPS22HB_I2C(int bus, int bus_frequency);
	~LPS22HB_I2C() override = default;

	int	read(unsigned address, void *data, unsigned count) override;
	int	write(unsigned address, void *data, unsigned count) override;

protected:
	int	probe() override;

};

device::Device *
LPS22HB_I2C_interface(int bus, int bus_frequency)
{
	return new LPS22HB_I2C(bus, bus_frequency);
}

LPS22HB_I2C::LPS22HB_I2C(int bus, int bus_frequency) :
	I2C(DRV_BARO_DEVTYPE_LPS22HB, MODULE_NAME, bus, LPS22HB_ADDRESS, bus_frequency)
{
}

int LPS22HB_I2C::probe()
{
	uint8_t id = 0;

	_retries = 10;

	if (read(WHO_AM_I, &id, 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if (id != LPS22HB_ID_WHO_AM_I) {
		DEVICE_DEBUG("ID byte mismatch (%02x != %02x)", LPS22HB_ID_WHO_AM_I, id);
		return -EIO;
	}

	return PX4_OK;
}

int LPS22HB_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int
LPS22HB_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}
