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

#ifndef INCLUDE_DYKNOW_NODEHANDLE_PROXY_HPP_
#define INCLUDE_DYKNOW_NODEHANDLE_PROXY_HPP_

#include "ros/ros.h"
#include "boost/shared_ptr.hpp"
#include "subscriber.hpp"
#include "publisher.hpp"
#include "timer.hpp"
#include <map>
#include "dyknow_nodehandle/Configure.h"
#include "dyknow_nodehandle/GetConfig.h"
#include "dyknow_nodehandle/Sample.h"
#include <dyknow_analytics/Monitor.h>
#include <dyknow_analytics/MonitorPair.h>
#include <dyknow_analytics/MonitorSet.h>

namespace dyknow {

class Proxy {

public:
	static const uint32_t DEFAULT_QUEUE_SIZE = 1024;

	Proxy();
	Proxy(ros::NodeHandle nh);
	~Proxy();

	ros::NodeHandle getNodeHandle() { return nh; }
	boost::function<void(std::map<std::string, std::string>, std::map<std::string, std::string>)> reconfigCallbackFn;

	template<class M>
	dyknow::Subscriber subscribe(const std::string& chan, uint32_t queueSize, const boost::function<void (const boost::shared_ptr<M const>&)>& callback, bool decorate = true,
							 const ros::VoidConstPtr& trackedObj = ros::VoidConstPtr(), const ros::TransportHints& transportHints = ros::TransportHints()) {
		SubscriberState* statePtr = new SubscriberState();
		statePtr->init(nh, chan, queueSize, boost::bind(&dyknow::SubscriberState::delegateSubscribe<M>, statePtr, _1, _2, callback, decorate));
		return subscribe(statePtr);
	}

	template<class M, class T>
	/**
	 * \brief Performs the first step in the two-step subscription process, yielding a dyknow::Subscription object.
	 * Binds the object T and the callback method with message argument M and to the ros::NodeHandle::subscribe method so that it can be stored without requiring templates.
	 * \returns A dyknow::Subscriber that can be initialised by DyKnow with a topic corresponding to the provided channel.
	 */
	dyknow::Subscriber subscribe(std::string chan, uint32_t queueSize, void(T::*fp)(M), T* obj, bool decorate = true) {
		SubscriberState* statePtr = new SubscriberState();
		statePtr->init(nh, chan, queueSize, boost::bind(&dyknow::SubscriberState::delegateSubscribe<M,T>, statePtr, _1, _2, fp, obj, decorate));
		return subscribe(statePtr);
	}

	dyknow::Subscriber subscribe(SubscriberState* statePtr);

	template<class M, class T>
	dyknow::Subscriber subscribe(uint32_t queueSize, void(T::*fp)(M), T* obj) {
		std::stringstream ss;
		ss << (subCount+1);
		return subscribe("_" + ss.str(), queueSize, fp, obj);
	}

	template<class M>
	/**
	 * \brief Performs the first step in the two-step advertisement process, yielding a dyknow::Publisher object.
	 * Binds the message type M and to the ros::NodeHandle::advertise method so that it can be stored without requiring the message template.
	 * \returns A dyknow::Publisher that can be initialised by DyKnow with a topic corresponding to the provided channel.
	 */
	dyknow::Publisher advertise(std::string chan, uint32_t queueSize, bool latch = false) {
		PublisherState* statePtr = new PublisherState();
		statePtr->init(nh, chan, queueSize, latch, boost::bind(&dyknow::PublisherState::delegateAdvertise<M>, statePtr, _1, _2, latch));
		return advertise(statePtr);
	}

	dyknow::Publisher advertise(PublisherState* statePtr);

	template<class M>
	dyknow::Publisher advertise(uint32_t queueSize, bool latch = false) {
		std::stringstream ss;
		ss << (pubCount+1);
		return advertise<M>("/" + ss.str(), queueSize, latch);
	}

	void connectInput(std::string chan, std::string topic);
	void connectOutput(std::string chan, std::string topic);

	std::vector<std::string> getInputs();
	std::vector<std::string> getOutputs();

	bool configureCallback(dyknow_nodehandle::Configure::Request& req, dyknow_nodehandle::Configure::Response& res);
	bool getConfigCallback(dyknow_nodehandle::GetConfig::Request& req, dyknow_nodehandle::GetConfig::Response& res);

	template<class T>
	void setReconfigureCallback(void(T::*fp)(std::map<std::string, std::string>, std::map<std::string, std::string>), T* obj) {
		this->reconfigCallbackFn = boost::bind(fp, obj, _1, _2);
	}

	template<class T>
	dyknow::Timer createTimer(ros::Duration period, void(T::*fp)(const ros::TimerEvent&), T* obj, bool oneshot = false, bool autostart = true, bool decorate = true) {
		dyknow::TimerState* statePtr = new TimerState(nh);
		statePtr->init(period, fp, obj, oneshot, autostart, decorate);
		timers.push_back(statePtr);
		return createTimer(statePtr);
	}

	dyknow::Timer createTimer(TimerState* statePtr);

	void analyticsCallback(const ros::TimerEvent& event);


private:
	ros::NodeHandle nh;
	bool ready;
	uint32_t subCount;
	uint32_t pubCount;
	uint32_t reconfigCount;

	std::map<std::string, dyknow::SubscriberState*> subscriberMap;
	std::map<std::string, dyknow::PublisherState*> publisherMap;
	std::vector<dyknow::TimerState*> timers;

	ros::ServiceServer configureServiceOld;
	ros::ServiceServer configureService;
	ros::ServiceServer getConfigService;

	ros::Timer analyticsTimer;
	ros::Publisher analyticsPublisher;
};


} //namespace



#endif /* INCLUDE_DYKNOW_NODEHANDLE_PROXY_HPP_ */
