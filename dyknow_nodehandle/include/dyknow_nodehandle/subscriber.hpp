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

#ifndef INCLUDE_DYKNOW_NODELETPROXY_SUBSCRIBER_HPP_
#define INCLUDE_DYKNOW_NODELETPROXY_SUBSCRIBER_HPP_

#include "ros/ros.h"
#include "boost/function.hpp"
#include <boost/thread.hpp>
#include "dyknow_analytics/monitor.hpp"
#include "dyknow_analytics/analysis.hpp"

namespace dyknow {

class SubscriberState {
public:
	SubscriberState() : size(0), received(0), connected(false) {}
	virtual ~SubscriberState() {}

	ros::NodeHandle nh;
	bool connected;
	ros::Subscriber sub;
	std::string chan;
	uint32_t size;
	boost::function<void(std::string, uint32_t)> createSubscriber;
	uint32_t received;

	virtual void init(ros::NodeHandle nh, std::string chan, uint32_t queueSize, boost::function<void(std::string, uint32_t)> createSubscriberFn) {
		this->nh = nh;
		this->chan = chan;
		this->size = queueSize;
		this->createSubscriber = createSubscriberFn;
	}

	template<class M, class T>
	/**
	 * \brief Delegation method to set up a subscription.
	 * Calls the subscribe method for the ros::NodeHandle class and returns a ros::Subscriber object.
	 * The delegation method is used for the two-step subscription process after binding the callback method and object.
	 */
	void delegateSubscribe(std::string topic, uint32_t queueSize, void(T::*fp)(M), T* obj, bool decorate = true) {
		if(!decorate) {
			// Delegates user callback directly to ROS
			this->sub = nh.subscribe(topic, queueSize, fp, obj);
		}
		else {
			// Decorates user callback before delegating resulting method to ROS
			const boost::function<void (M)>& callbackWrapper = boost::bind(&dyknow::SubscriberState::genericCallback<M,T>, this, _1, fp, obj);
			this->sub = nh.subscribe<M>(topic, queueSize, callbackWrapper);
		}
		this->connected = true;
	}

	template<class M, class T>
	void delegateSubscribe(std::string topic, uint32_t queueSize, const boost::function<void (const boost::shared_ptr<M const>&)>& callback, bool decorate = true) {
		if(!decorate) {
			// Delegates user callback directly to ROS
			this->sub = nh.subscribe<M>(topic, queueSize, callback);
		}
		else {
			// Decorates user callback before delegating resulting method to ROS
			const boost::function<void (M)>& callbackWrapper = boost::bind(&dyknow::SubscriberState::genericCallback<M,T>, this, _1, callback);
			this->sub = nh.subscribe<M>(topic, queueSize, callbackWrapper);
		}
		this->connected = true;
	}

	template<class M, class T>
	void genericCallback(M data, void(T::*fp)(M), T* obj) {
		// Monitor the user-provided callback
		dyknow::Monitor start, end;
		start.init();
		(obj->*fp)(data);
		end.init();
		log(start, end);
		received++;
	}

	virtual std::string getTopic() { return sub.getTopic(); }

	std::vector<std::pair<dyknow::Monitor, dyknow::Monitor> > collectTrace();

private:
	std::vector<std::pair<dyknow::Monitor, dyknow::Monitor> > trace;
	boost::mutex traceAccess;

	void log(dyknow::Monitor first, dyknow::Monitor second);
};

class Subscriber {
public:
	Subscriber() : statePtr(NULL) {}
	Subscriber(ros::NodeHandle nh, SubscriberState* statePtr) : nh(nh), statePtr(statePtr) {}
	~Subscriber() {}

	bool isConnected() { return statePtr != NULL && statePtr->connected; }
	std::string getTopic() { return statePtr->getTopic(); }
	std::string getChannel() { return statePtr->chan; }
	uint32_t getSize() { return statePtr->size; }

private:
	ros::NodeHandle nh;
	SubscriberState* statePtr;
};

} // namespace



#endif /* INCLUDE_DYKNOW_NODELETPROXY_SUBSCRIBER_HPP_ */
