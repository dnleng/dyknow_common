/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Daniel de Leng
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_DYKNOW_NODELETPROXY_NODE_HANDLE_HPP_
#define INCLUDE_DYKNOW_NODELETPROXY_NODE_HANDLE_HPP_

#include "ros/ros.h"
#include "proxy.hpp"
#include "boost/shared_ptr.hpp"
#include "subscriber.hpp"
#include "publisher.hpp"
#include "timer.hpp"

namespace dyknow {

typedef boost::shared_ptr<Proxy> ProxyPtr;

/**
 * \brief A lightweight DyKnow variant of the ros::NodeHandle class.
 */
class NodeHandle {
public:
	NodeHandle() : proxyPtr(new Proxy()) {}
	NodeHandle(ros::NodeHandle nh) : proxyPtr(new Proxy(nh)) {}
	~NodeHandle() {}

	NodeHandle& operator=(const NodeHandle& other) {
		this->proxyPtr = other.proxyPtr;
		return *this;
	}

	bool isReady() {
		return proxyPtr != NULL;
	}

	ProxyPtr getProxy() {
		return proxyPtr;
	}

	template<class M, class T>
	/**
	 * \see dyknow::Proxy::subscribe
	 */
	dyknow::Subscriber subscribe(std::string chan, uint32_t queueSize, void(T::*fp)(M), T* obj) {
		return proxyPtr->subscribe(chan, queueSize, fp, obj);
	}

	template<class M>
	dyknow::Subscriber subscribe(const std::string& chan, uint32_t queueSize, const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
							 const ros::VoidConstPtr& trackedObj = ros::VoidConstPtr(), const ros::TransportHints& transportHints = ros::TransportHints()) {
		return proxyPtr->subscribe(chan, queueSize, callback, trackedObj, transportHints);
	}

	template<class M, class T>
	dyknow::Subscriber subscribe(uint32_t queueSize, void(T::*fp)(M), T* obj) {
		return proxyPtr->subscribe(queueSize, fp, obj);
	}

	dyknow::Subscriber subscribe(SubscriberState* statePtr) {
		return proxyPtr->subscribe(statePtr);
	}

	template<class M>
	/**
	 * \see dyknow::Proxy::advertise
	 */
	dyknow::Publisher advertise(std::string chan, uint32_t queueSize, bool latch = false) {
		return proxyPtr->advertise<M>(chan, queueSize, latch);
	}

	template<class M>
	dyknow::Publisher advertise(uint32_t queueSize, bool latch = false) {
		return proxyPtr->advertise<M>(queueSize, latch);
	}

	dyknow::Publisher advertise(PublisherState* statePtr) {
		return proxyPtr->advertise(statePtr);
	}

	template<class T, class MReq, class MRes>
	/**
	 * \see ros::NodeHandle::advertiseService
	 */
	ros::ServiceServer advertiseService(const std::string& service, bool(T::*srv_func)(MReq &, MRes &), T *obj) {
		return proxyPtr->getNodeHandle().advertiseService<T, MReq, MRes>(service, srv_func, obj);
	}


	template<class Service>
	/**
	 * \see ros::NodeHandle::serviceClient
	 */
	ros::ServiceClient serviceClient(const std::string& service_name, bool persistent = false, const std::map<std::string, std::string>& header_values = std::map<std::string, std::string>()) {
		return proxyPtr->getNodeHandle().serviceClient<Service>(service_name, persistent, header_values);
	}

	template<class T>
	dyknow::Timer createTimer(ros::Duration period, void(T::*fp)(const ros::TimerEvent&), T* obj, bool oneshot = false, bool autostart = true, bool decorate = true) {
		return proxyPtr->createTimer(period, fp, obj, oneshot, autostart, decorate);
	}

	dyknow::Timer createTimer(TimerState* statePtr) {
		return proxyPtr->createTimer(statePtr);
	}

private:
	ProxyPtr proxyPtr;
};

} // namespace

#endif /* INCLUDE_DYKNOW_NODELETPROXY_NODE_HANDLE_HPP_ */
