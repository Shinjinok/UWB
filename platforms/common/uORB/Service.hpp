/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file Service.hpp
 *
 * Implements a uORB Service that acts like ROS2 Service.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <systemlib/err.h>
#include <future>

#include <uORB/uORB.h>
#include "uORBManager.hpp"
#include "uORB/Subscription.hpp"
#include "uORB/Publication.hpp"

#include <uORB/topics/uORBTopics.hpp>

namespace uORB
{
/**
 * @brief Service class that provides the service
 *
 * @tparam req Request data structure defined by the Service
 * @tparam resp Response data structure defined by the Service
 */
template<typename req, typename resp>
class Service
{
public:
	/**
	 * @brief Construct a new Service object with the given request & response uORB topics
	 */
	Service(const orb_metadata *req_, const orb_metadata *resp_)
		: _request_sub(req_), _response_pub(resp_) {};

	/**
	 * @brief Registers the callback function that will be called when a new request arrives
	 *
	 * @return true
	 * @return false
	 */
	bool registerCallback(const void *cb_func(const req &, resp))
	{
		_request_sub.registerCallback(cb_func);
		_cb_func = cb_func; // Internally save the function pointer
	}

private:
	void *_cb_func(const req &, resp); // Function pointer to the callback

	uORB::Subscription _request_sub;
	uORB::Publication<resp> _response_pub;
};


/**
 * @brief Client class that subscribes to the Service
 *
 * @tparam req Request data structure defined by the Service
 * @tparam resp Response data structure defined by the Service
 */
template<typename req, typename resp>
class Client
{
public:
	/**
	 * @brief Construct a new Client object with the given request & response uORB topics
	 */
	Client(const orb_metadata *req_, const orb_metadata *resp_)
		: _request_pub(req_), _response_sub(resp_) {};

	/**
	 * @brief Send the request asynchronously
	 *
	 * @return Future instance of the response that will asynchronously be updated
	 */
	std::future<resp *> async_send_request(const &req)
	{
		_request_pub.update(req); // Publish new request
	}

private:
	uORB::Publication<req> _request_pub;
	uORB::Subscription _response_sub;
};


}
