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

#ifndef INCLUDE_DYKNOW_NODELETPROXY_PUBLISHER_HPP_
#define INCLUDE_DYKNOW_NODELETPROXY_PUBLISHER_HPP_

#include "ros/ros.h"
#include "boost/function.hpp"

namespace dyknow {

class PublisherState {
public:
	PublisherState() : size(0), sent(0), connected(false), latch(false) {}
	virtual ~PublisherState() {}

	ros::NodeHandle nh;
	bool connected;
	std::string chan;
	uint32_t size;
	bool latch;
	boost::function<void(std::string, uint32_t)> createPublisher;
	ros::Publisher pub;
	uint32_t sent;

	virtual void init(ros::NodeHandle nh, std::string chan, uint32_t queueSize, bool latch, boost::function<void(std::string, uint32_t)> createPublisherFn) {
		this->nh = nh;
		this->chan = chan;
		this->size = queueSize;
		this->latch = latch;
		this->createPublisher = createPublisherFn;
	}

	template<class M>
	/**
	 * \brief Delegation method to set up a publisher.
	 * Calls the advertise method for the ros::NodeHandle class and returns a ros::Publisher object.
	 * The delegation method is used for the two-step advertisement process.
	 */
	void delegateAdvertise(std::string topic, uint32_t queueSize, bool latch = false) {
		pub = nh.template advertise<M>(topic, queueSize, latch);
		connected = true;
	}

	template<class M>
	void defaultPublish(const M& msg) {
		pub.publish(msg);
		sent++;
	}

	virtual uint32_t getNumSubscribers() const {
		return connected ? pub.getNumSubscribers() : 0;
	}


	virtual std::string getTopic() { return pub.getTopic(); }
};

class Publisher {
public:
	Publisher() : statePtr(NULL) {}
	Publisher(ros::NodeHandle nh, PublisherState* statePtr) : nh(nh), statePtr(statePtr) {}
	~Publisher() {}

	bool isConnected() { return statePtr != NULL && statePtr->connected; }
	std::string getTopic() { return statePtr != NULL ? statePtr->getTopic() : ""; }
	std::string getChannel() { return statePtr != NULL ? statePtr->chan : ""; }
	uint32_t getSize() { return statePtr != NULL ? statePtr->size : 0; }
	PublisherState* getState() { return statePtr; }
	bool isLatched() { return statePtr != NULL ? statePtr->latch : false; }
	uint32_t getNumSubscribers() const { return statePtr != NULL && statePtr->chan != "" ? statePtr->getNumSubscribers() : 0; }

	template<class M>
	void publish(const M& msg) { publish<M>(boost::bind(&PublisherState::defaultPublish<M>, statePtr, _1), msg); }

	template<class M>
	void publish(boost::function<void(const M&)> publishFn, const M& msg) {
		if(isConnected()) publishFn(msg);
	}

private:
	ros::NodeHandle nh;
	PublisherState* statePtr;
};

} // namespace



#endif /* INCLUDE_DYKNOW_NODELETPROXY_PUBLISHER_HPP_ */
