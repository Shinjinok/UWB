/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file lis3mdl_i2c.cpp
 *
 * I2C interface for LIS3MDL
 */

#include <px4_config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "board_config.h"
#include "lis3mdl.h"

#if defined(PX4_I2C_BUS_ONBOARD) || defined(PX4_I2C_BUS_EXPANSION)

#define LIS3MDLL_ADDRESS        0x1e

class LIS3MDL_I2C : public device::I2C
{
public:
	LIS3MDL_I2C(int bus);
	virtual ~LIS3MDL_I2C() = default;

	virtual int     ioctl(unsigned operation, unsigned &arg);
	virtual int     read(unsigned address, void *data, unsigned count);
	virtual int     write(unsigned address, void *data, unsigned count);

protected:
	virtual int     probe();

};

device::Device *
LIS3MDL_I2C_interface(int bus);

device::Device *
LIS3MDL_I2C_interface(int bus)
{
	return new LIS3MDL_I2C(bus);
}

LIS3MDL_I2C::LIS3MDL_I2C(int bus) :
	I2C("LIS3MDL_I2C", nullptr, bus, LIS3MDLL_ADDRESS, 400000)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LIS3MDL;
}

int
LIS3MDL_I2C::ioctl(unsigned operation, unsigned &arg)
{
	switch (operation) {

	case MAGIOCGEXTERNAL:
		return external();

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default:
		return  -EINVAL;
	}
}

int
LIS3MDL_I2C::probe()
{
	uint8_t data = 0;

	_retries = 10;

	if (read(ADDR_WHO_AM_I, &data, 1)) {
		PX4_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if (data != ID_WHO_AM_I) {
		PX4_DEBUG("LIS3MDL bad ID: %02x", data);
		return -EIO;
	}

	return OK;
}

int
LIS3MDL_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int
LIS3MDL_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

#endif /* PX4_I2C_OBDEV_LIS3MDL */
