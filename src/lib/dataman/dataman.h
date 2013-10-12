/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file dataman.h
 *
 * DATAMANAGER driver.
 */
#ifndef _DATAMANAGER_H
#define _DATAMANAGER_H

#include <uORB/topics/mission.h>

#ifdef __cplusplus
extern "C" {
#endif

	/* Types of items that the data manager can store */
	typedef enum {
		DM_KEY_RTL_POINT = 0,		/* Return to landing point coordinates */
		DM_KEY_FENCE_RETURN_POINT,	/* Fence violation return coordinates */
		DM_KEY_SAFE_POINTS,		/* Safe points coordinates */
		DM_KEY_FENCE_POINTS,		/* Fence vertex coordinates */
		DM_KEY_WAY_POINTS,		/* Mission way point coordinates */
		DM_KEY_NEW_WAY_POINTS,		/* New mission way point coordinates received by mavlink */
		DM_KEY_NUM_KEYS			/* Total number of item types defined */
	} dm_item_t;

	/* The maximum number of instances for each item type */
	enum {
		DM_KEY_RTL_POINT_MAX = 1,
		DM_KEY_RETURN_POINT_MAX = 1,
		DM_KEY_SAFE_POINTS_MAX = 5,
		DM_KEY_FENCE_POINTS_MAX = 10,
		DM_KEY_WAY_POINTS_MAX = MAX_MISSION_ITEMS,
		DM_KEY_NEW_WAY_POINTS_MAX = MAX_MISSION_ITEMS,
	};

	/* Data persistence levels */
	typedef enum {
		DM_PERSIST_POWER_ON_RESET = 0,	/* Data survives all resets */
		DM_PERSIST_IN_FLIGHT_RESET,     /* Data survives in-flight resets only */
		DM_PERSIST_VOLATILE             /* Data does not survive resets */
	} dm_persitence_t;

	/* The reason for the last reset */
	typedef enum {
		DM_INIT_REASON_POWER_ON = 0,	/* Data survives resets */
		DM_INIT_REASON_IN_FLIGHT	/* Data survives in-flight resets only */
	} dm_reset_reason;

	/* Maximum size in bytes of a single item instance */
	#define DM_MAX_DATA_SIZE 126

	/* Open a data manager handle */
	__EXPORT int
	dm_open(void);

	/* Close the data manager handle */
	__EXPORT void
	dm_close(
		int fd				/* The handle to be closed */
	);

	/* Retrieve from the data manager store */
	__EXPORT ssize_t
	dm_read(
		int fd,				/* Data manager handle */
		dm_item_t item,			/* The item type to retrieve */
		unsigned char index,		/* The index of the item */
		void *buffer,			/* Pointer to caller data buffer */
		size_t buflen			/* Length in bytes of data to retrieve */
	);

	/* write to the data manager store */
	__EXPORT ssize_t
	dm_write(
		int fd,				/* Data manager handle */
		dm_item_t  item,		/* The item type to store */
		unsigned char index,		/* The index of the item */
		dm_persitence_t persistence,	/* The persistence level of this item */
		const void *buffer,		/* Pointer to caller data buffer */
		size_t buflen			/* Length in bytes of data to retrieve */
	);

	/* Tell the data manager about the type of the last reset */
	__EXPORT int
	dm_restart(
		dm_reset_reason restart_type	/* The last reset type */
	);

#ifdef __cplusplus
}
#endif

#endif
