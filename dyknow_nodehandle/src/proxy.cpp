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

#include "dyknow_nodehandle/proxy.hpp"

namespace dyknow {

Proxy::Proxy() : subCount(0), pubCount(0), reconfigCount(0), reconfigCallbackFn(NULL) {
	this->ready = false;
//	setReconfigureCallback(&Proxy::reconfigureCallback, this);
}

Proxy::Proxy(ros::NodeHandle nh) : subCount(0), pubCount(0), reconfigCount(0), reconfigCallbackFn(NULL) {
	this->nh = nh;
	configureService = nh.advertiseService("set_config", &Proxy::configureCallback, this);
	getConfigService = nh.advertiseService("get_config", &Proxy::getConfigCallback, this);
	analyticsTimer = nh.createTimer(ros::Duration(1.0), &Proxy::analyticsCallback, this);
	analyticsPublisher = nh.advertise<dyknow_analytics::MonitorSet>("analytics", 1024);

	this->ready = true;
}

Proxy::~Proxy() {
	std::map<std::string, dyknow::SubscriberState*>::iterator subIter;
	for(subIter = subscriberMap.begin(); subIter != subscriberMap.end(); subIter++) {
		delete subIter->second;
	}

	std::map<std::string, dyknow::PublisherState*>::iterator pubIter;
	for(pubIter = publisherMap.begin(); pubIter != publisherMap.end(); pubIter++) {
		delete pubIter->second;
	}
}

void Proxy::connectInput(std::string chan, std::string topic) {
	std::map<std::string, dyknow::SubscriberState*>::iterator iter;
	if((iter = subscriberMap.find(chan)) != subscriberMap.end()) {
		iter->second->createSubscriber(topic, iter->second->size == 0 ? DEFAULT_QUEUE_SIZE : iter->second->size);
	}
	else {
		// Throw error if channel is explicit
		if(chan.at(0) != '_') {
			ROS_WARN("[%s] Could not find a Subscriber for input channel %s", nh.getNamespace().c_str(), chan.c_str());
		}
	}
}

void Proxy::connectOutput(std::string chan, std::string topic) {
	std::map<std::string, dyknow::PublisherState*>::iterator iter;
	if((iter = publisherMap.find(chan)) != publisherMap.end()) {
		iter->second->createPublisher(topic, iter->second->size == 0 ? DEFAULT_QUEUE_SIZE : iter->second->size);
	}
	else {
		// Throw error if channel if explicit
		if(chan.at(0) != '/') {
			ROS_WARN("[%s] Could not find a Publisher for output channel %s", nh.getNamespace().c_str(), chan.c_str());
		}
	}
}

std::vector<std::string> Proxy::getInputs() {
	std::vector<std::string> inputs;
	std::map<std::string, dyknow::SubscriberState*>::iterator iter;
	for(iter = subscriberMap.begin(); iter != subscriberMap.end(); iter++) {
		inputs.push_back(iter->second->getTopic());
	}
	return inputs;
}

std::vector<std::string> Proxy::getOutputs() {
	std::vector<std::string> outputs;
	std::map<std::string, dyknow::PublisherState*>::iterator iter;
	for(iter = publisherMap.begin(); iter != publisherMap.end(); iter++) {
		outputs.push_back(iter->second->getTopic());
	}
	return outputs;
}


bool Proxy::configureCallback(dyknow_nodehandle::Configure::Request& req, dyknow_nodehandle::Configure::Response& res) {
	std::map<std::string, std::string> incoming;
	std::map<std::string, std::string> outgoing;

	if(req.config.in_names.size() == req.config.in_channels.size()) {
		for(int i = 0; i < req.config.in_names.size(); i++) {
			std::stringstream ss;
			ss << i;
			std::string port = "_" + ss.str();

			connectInput(req.config.in_channels[i] == "" ? port : req.config.in_channels[i], req.config.in_names[i]);
			incoming[req.config.in_channels[i] == "" ? port : req.config.in_channels[i]] = req.config.in_names[i];
		}
	}
	else {
		// Incorrect service call
		ROS_WARN("[%s] Number of incoming topics does not match number of incoming channels: skipping", nh.getNamespace().c_str());
	}

	if(req.config.out_names.size() == req.config.out_channels.size()) {
		for(int i = 0; i < req.config.out_names.size(); i++) {
			std::stringstream ss;
			ss << i;
			std::string port = "/" + ss.str();

			connectOutput(req.config.out_channels[i] == "" ? port : req.config.out_channels[i], req.config.out_names[i]);
			outgoing[req.config.out_channels[i] == "" ? port : req.config.out_channels[i]] = req.config.out_names[i];
		}
	}
	else {
		// Incorrect service call
		ROS_WARN("[%s] Number of outgoing topics does not match number of outgoing channels: skipping", nh.getNamespace().c_str());
	}

	if(this->reconfigCallbackFn != NULL) { this->reconfigCallbackFn(incoming, outgoing); }
	reconfigCount++;
	res.success = true;
	return true;
}

bool Proxy::getConfigCallback(dyknow_nodehandle::GetConfig::Request& req, dyknow_nodehandle::GetConfig::Response& res) {
	res.inputs = getInputs();
	res.outputs = getOutputs();
	res.success = true;
	return true;
}

dyknow::Subscriber Proxy::subscribe(SubscriberState* statePtr) {
	std::map<std::string, dyknow::SubscriberState*>::iterator iter;
	if((iter = subscriberMap.find(statePtr->chan)) != subscriberMap.end()) {
		// Update existing subscriber
		ROS_WARN("[%s] Input channel %s already exists", nh.getNamespace().c_str(), statePtr->chan.c_str());
//		*iter->second = *statePtr;
//		delete statePtr;
		delete subscriberMap[statePtr->chan];
		subscriberMap[statePtr->chan] = statePtr;
		return dyknow::Subscriber(nh, iter->second);
	}
	else {
		// Create new subscriber
		subCount++;
		subscriberMap[statePtr->chan] = statePtr;
		return dyknow::Subscriber(nh, statePtr);
	}
}

dyknow::Publisher Proxy::advertise(PublisherState* statePtr) {
	std::map<std::string, dyknow::PublisherState*>::iterator iter;
	if((iter = publisherMap.find(statePtr->chan)) != publisherMap.end()) {
		// Update existing publisher
		ROS_WARN("[%s] Output channel %s already exists", nh.getNamespace().c_str(), statePtr->chan.c_str());
//		*iter->second = *statePtr;
//		delete statePtr;
		delete publisherMap[statePtr->chan];
		publisherMap[statePtr->chan] = statePtr;
		return dyknow::Publisher(nh, statePtr);
	}
	else {
		// Create new publisher
		pubCount++;
		publisherMap[statePtr->chan] = statePtr;
		return dyknow::Publisher(nh, statePtr);
	}
}

dyknow::Timer Proxy::createTimer(TimerState* statePtr) {
	timers.push_back(statePtr);
	return dyknow::Timer(nh, statePtr);
}

void Proxy::analyticsCallback(const ros::TimerEvent& event) {
	dyknow_analytics::MonitorSet msg;
	msg.received = 0;
	msg.sent = 0;

	for(int i = 0; i < timers.size(); i++) {
		std::vector<std::pair<dyknow::Monitor, dyknow::Monitor> > trace = timers[i]->collectTrace();
		for(int j = 0; j < trace.size(); j++) {
			dyknow_analytics::MonitorPair pair;
			pair.first = trace[j].first.toMsg();
			pair.second = trace[j].second.toMsg();
			msg.monitors.push_back(pair);
		}
	}

	std::map<std::string, SubscriberState*>::iterator subscriberIter;
	for(subscriberIter = subscriberMap.begin(); subscriberIter != subscriberMap.end(); subscriberIter++) {
		std::vector<std::pair<dyknow::Monitor, dyknow::Monitor> > trace = subscriberIter->second->collectTrace();
		for(int j = 0; j < trace.size(); j++) {
			dyknow_analytics::MonitorPair pair;
			pair.first = trace[j].first.toMsg();
			pair.second = trace[j].second.toMsg();
			msg.monitors.push_back(pair);
		}

		msg.received += subscriberIter->second->received;
	}

	std::map<std::string, PublisherState*>::iterator publisherIter;
	for(publisherIter = publisherMap.begin(); publisherIter != publisherMap.end(); publisherIter++) {
		msg.sent += publisherIter->second->sent;
	}

	if(this->ready) {
		analyticsPublisher.publish(msg);
	}
}

} //namespace
