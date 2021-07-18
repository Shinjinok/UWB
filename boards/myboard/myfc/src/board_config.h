/**
 * @file board_config.h
 *
 * myboardmyfc internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

#ifndef __ASSEMBLY__

#include <px4_platform_common/board_common.h>

#define HRT_TIMER 1
#define HRT_TIMER_CHANNEL 1

#endif /* __ASSEMBLY__ */

__END_DECLS
