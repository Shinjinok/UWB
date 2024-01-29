/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "UavcanPublisherBase.hpp"

#include <dronecan/sensors/rc/RCInput.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/input_rc.h>

namespace uavcannode
{

class RCInput :
	public UavcanPublisherBase,
	private uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<dronecan::sensors::rc::rcinput>
{
public:
	RCInput(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(dronecan::sensors::rc::rcinput::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(input_rc)),
		uavcan::Publisher<dronecan::sensors::rc::rcinput>(node)
	{
		this->setPriority(uavcan::TransferPriority::OneLowerThanHighest);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       dronecan::sensors::rc::rcinput::getDataTypeFullName(),
			       dronecan::sensors::rc::rcinput::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// input_rc -> dronecan::sensors::rc::rcinput
		input_rc_s input_rc;

		if (uORB::SubscriptionCallbackWorkItem::update(&input_rc)) {

			dronecan::sensors::rc::rcinput msg{};

			msg.status = 0;

			if (input_rc.rc_failsafe) {
				msg.status = dronecan::sensors::rc::rcinput::STATUS_FAILSAFE;
			} else {
				msg.status = dronecan::sensors::rc::rcinput::STATUS_RSSI_VALID;
			}

			msg.rssi = input_rc.rssi;

			for (size_t i = 0; i < input_rc::RC_INPUT_MAX_CHANNELS; i++) {
				msg.rcin = input_rc.values;
			}

			uavcan::Publisher<dronecan::sensors::rc::rcinput>::broadcast(msg);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcan
