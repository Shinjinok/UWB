/****************************************************************************
 *
 *   Copyright (C) 2019, 2022 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_hw_rev_ver.c
 * Implementation of IMXRT based Board Hardware Revision and Version ID API
 */
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/board_determine_hw_info.h>
#include <stdio.h>
#include <board_config.h>

#include <systemlib/px4_macros.h>

#if defined(BOARD_HAS_HW_VERSIONING) || defined(BOARD_HAS_HW_SPLIT_VERSIONING)

#  if defined(GPIO_HW_VER_REV_DRIVE)
#    define GPIO_HW_REV_DRIVE GPIO_HW_VER_REV_DRIVE
#    define GPIO_HW_VER_DRIVE GPIO_HW_VER_REV_DRIVE
#  endif

#define HW_INFO_SIZE (int) arraySize(HW_INFO_INIT_PREFIX) + HW_INFO_VER_DIGITS + HW_INFO_REV_DIGITS


/****************************************************************************
 * Private Data
 ****************************************************************************/
static int hw_version = 0;
static int hw_revision = 0;
static char hw_info[HW_INFO_SIZE] = {0};
#if defined(BOARD_HAS_HW_SPLIT_VERSIONING)
static char hw_base_info[HW_INFO_SIZE] = {0};
#endif

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

static int dn_to_ordinal(uint16_t dn)
{
	// Refernece is 3.8933 = (1.825f * 64.0f / 30.0f)
	// LSB is 0.000950521 = 3.8933 / 4096
	// DN's are V/LSB

	const struct {
		uint16_t low;  // High(n-1) + 1
		uint16_t high; // Average High(n)+Low(n+1) EX. 1356 = AVRG(1331,1382)
	} dn2o[] = {
		//                   R2(down) R1(up)    V min       V Max       DN Min DN Max
		{0,    0   },  // 0                     No Resistors
		{1,    490 },  // 1  24.9K   442K   0.166255191  0.44102252    174	 463
		{491,  819 },  // 2  32.4K   174K   0.492349322  0.770203609   517	 810
		{820,  1149},  // 3  38.3K   115K   0.787901749  1.061597759   828	1116
		{1150, 1488},  // 4  46.4K   84.5K  1.124833577  1.386007306   1183	1458
		{1489, 1811},  // 5  51.1K   61.9K  1.443393279  1.685367869   1518	1773
		{1812, 2135},  // 6  61.9K   51.1K  1.758510242  1.974702534   1850	2077
		{2136, 2474},  // 7  84.5K   46.4K  2.084546498  2.267198261   2193	2385
		{2475, 2804},  // 8  115K    38.3K  2.437863827  2.57656294    2564	2710
		{2805, 3135},  // 9  174K    32.4K  2.755223792  2.847933804   2898	2996
		{3136, 4095},  // 10 442K    24.9K  3.113737849  3.147347506   3275	3311
	};

	for (unsigned int i = 0; i < arraySize(dn2o); i++) {
		if (dn >= dn2o[i].low && dn <= dn2o[i].high) {
			return i;
		}
	}

	return -1;
}

/************************************************************************************
 * Name: read_id_dn
 *
 * Description:
 *   Read the HW sense set to get a DN of the value formed by
 *                0 VDD
 *                |
 *                /
 *                \   R1
 *                /
 *                |
 *                +--------------- GPIO_HW_xxx_SENCE  | ADC channel N
 *                |
 *                /
 *                \ R2
 *                /
 *                |
 *                |
 *                +--------------- GPIO_HW_xxx_DRIVE or GPIO_HW_VER_REV_DRIVE
 *
 * Input Parameters:
 *   id          - pointer to receive the dn for the id set
 *   gpio_drive  - gpio that is the drive
 *   gpio_sense  - gpio that is the sence
 *   adc_channel - the Channel number associated with gpio_sense
 *
 * Returned Value:
 *    0    - Success and id is set
 *   -EIO  - FAiled to init or read the ADC
 *
 ************************************************************************************/

static int read_id_dn(int *id, uint32_t gpio_drive, uint32_t gpio_sense, int adc_channel)
{
	int rv = -EIO;
	const unsigned int samples  = 16;
	/*
	 * Step one is there resistors?
	 *
	 * With the common REV/VER Drive we have to look at the ADC values.
	 * to determine if the R's are hooked up. This is because the
	 * the REV and VER pairs will influence each other and not make
	 * digital thresholds.
	 *
	 * I.E
	 *
	 *     VDD
	 *     442K
	 *       REV is a Float
	 *     24.9K
	 *        Drive as input
	 *     442K
	 *       VER is 0.
	 *     24.9K
	 *     VDD
	 *
	 *   This is 466K up and 442K down.
	 *
	 *  Driving VER Low and reading DRIVE will result in approximately mid point
	 *  values not a digital Low.
	 */

	uint32_t dn_sum = 0;
	uint32_t dn = 0;
	uint32_t high = 0;
	uint32_t low = 0;

	imxrt_config_gpio(gpio_sense);

	/*  Turn the drive lines to digital outputs High */

	imxrt_config_gpio(gpio_drive);

	up_udelay(100); /* About 10 TC assuming 485 K */

	for (unsigned av = 0; av < samples; av++) {
		if (px4_arch_adc_init(HW_REV_VER_ADC_BASE) == OK) {
			dn = px4_arch_adc_sample(HW_REV_VER_ADC_BASE, adc_channel);

			if (dn == UINT32_MAX) {
				break;
			}

			dn_sum  += dn;
		}
	}

	if (dn != UINT32_MAX) {
		high = dn_sum / samples;
	}

	/*  Turn the drive lines to digital outputs LOW */

	imxrt_config_gpio(gpio_drive ^ GPIO_OUTPUT_SET);

	up_udelay(100); /* About 10 TC assuming 485 K */

	dn_sum = 0;

	for (unsigned av = 0; av < samples; av++) {

		dn = px4_arch_adc_sample(HW_REV_VER_ADC_BASE, adc_channel);

		if (dn == UINT32_MAX) {
			break;
		}

		dn_sum  += dn;
	}

	if (dn != UINT32_MAX) {
		low = dn_sum / samples;
	}

	if ((high > low) && high > ((px4_arch_adc_dn_fullcount() * 800) / 1000)) {

		*id = low;
		rv = OK;


	} else {
		/* No - No Resistors is ID 0 */
		*id = 0;
		rv = OK;
	}

	/*  Turn the drive lines to digital outputs High */

	imxrt_config_gpio(gpio_drive);
	return rv;
}


static int determine_hw_info(int *revision, int *version)
{
	int dn;
	int rv = read_id_dn(&dn, GPIO_HW_REV_DRIVE, GPIO_HW_REV_SENSE, ADC_HW_REV_SENSE_CHANNEL);

	if (rv == OK) {
		*revision =  dn_to_ordinal(dn);
		rv = read_id_dn(&dn, GPIO_HW_VER_DRIVE, GPIO_HW_VER_SENSE, ADC_HW_VER_SENSE_CHANNEL);

		if (rv == OK) {
			*version =  dn_to_ordinal(dn);
		}
	}

	return rv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_get_hw_type
 *
 * Description:
 *   Optional returns a 0 terminated string defining the HW type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_type_name()
{
	return (const char *) hw_info;
}

#if defined(BOARD_HAS_HW_SPLIT_VERSIONING)
/************************************************************************************
 * Name: board_get_hw_base_type_name
 *
 * Description:
 *   Optional returns a 0 terminated string defining the base type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_base_type_name()
{
	return (const char *) hw_base_info;
}
#endif

/************************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware version.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having version.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_version()
{
	return  hw_version;
}

/************************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware revision.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having revision.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_revision()
{
	return  hw_revision;
}

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses the HW revision and version detection added in FMUv5.
 *	See https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY
 *	HW REV and VER ID tab.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0  - on success or negated errono
 *   1) The values for integer value of this boards hardware revision is set
 *   2) The integer value of this boards hardware version is set.
 *   3) hw_info is populated
 *
 *   A value of 0 is the default for boards supporting the BOARD_HAS_HW_VERSIONING API.
 *   but not having R1 and R2.
 *
 ************************************************************************************/

int board_determine_hw_info()
{
	int rv = determine_hw_info(&hw_revision, &hw_version);

	if (rv == OK) {

		if (rv == OK) {

			snprintf(hw_info, sizeof(hw_info), HW_INFO_INIT_PREFIX HW_INFO_SUFFIX, hw_version, hw_revision);

		}

		path = NULL;
		rvmft = px4_mtd_query("MTD_MFT_REV", NULL, &path);

		if (rvmft == OK && path != NULL && hw_revision == HW_ID_EEPROM) {

			mtd_mft_v0_t mtd_mft = {MTD_MFT_v0};
			rv = board_get_eeprom_hw_info(path, (mtd_mft_t *)&mtd_mft);

			if (rv == OK) {
				hw_revision = mtd_mft.hw_extended_id;
			}
		}
	}

	if (rv == OK) {
#if defined(BOARD_HAS_HW_SPLIT_VERSIONING)
		snprintf(hw_info, sizeof(hw_info), HW_INFO_INIT_PREFIX HW_INFO_FMUM_SUFFIX, GET_HW_FMUM_ID());
		snprintf(hw_base_info, sizeof(hw_info), HW_INFO_BASE_SUFFIX, GET_HW_BASE_ID());
#else
		snprintf(hw_info, sizeof(hw_info), HW_INFO_INIT_PREFIX HW_INFO_SUFFIX, hw_version, hw_revision);
#endif
	}

	return rv;
}
#endif
