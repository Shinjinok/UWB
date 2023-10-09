/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file can.c
 *
 * Board-specific CAN functions.
 */

#if !defined(CONFIG_CAN)

#include <stdint.h>

#include "board_config.h"


__EXPORT
uint16_t board_get_can_interfaces(void)
{
	uint16_t enabled_interfaces = 0x7;

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_CAN2)) {
		enabled_interfaces &= ~(1 << 1);
	}

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_CAN3)) {
		enabled_interfaces &= ~(1 << 2);
	}

	return enabled_interfaces;
}

#else

#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"

#include "chip.h"
#include "stm32_can.h"
#include "board_config.h"

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
int can_devinit(void);

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int can_devinit(void)
{
	static bool initialized = false;
	struct can_dev_s *can;
	int ret;

	/* Check if we have already initialized */

	if (!initialized) {
		/* Call stm32_caninitialize() to get an instance of the CAN interface */
#if defined(CONFIG_STM32F7_CAN1)
		can = stm32_caninitialize(1);

		if (can == NULL) {
			canerr("ERROR:  Failed to get CAN interface\n");
			return -ENODEV;
		}

		/* Register the CAN driver at "/dev/can0" */

		ret = can_register("/dev/can0", can);

		if (ret < 0) {
			canerr("ERROR: can_register failed: %d\n", ret);
			return ret;
		}
#endif
#if defined(CONFIG_STM32F7_CAN2)
		can = stm32_caninitialize(2);

		if (can == NULL) {
			canerr("ERROR:  Failed to get CAN interface\n");
			return -ENODEV;
		}

		/* Register the CAN driver at "/dev/can1" */

		ret = can_register("/dev/can1", can);

		if (ret < 0) {
			canerr("ERROR: can_register failed: %d\n", ret);
			return ret;
		}
#endif
		/* Now we are initialized */

		initialized = true;
	}

	return OK;
}

#endif /* CONFIG_CAN */
