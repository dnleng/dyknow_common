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

#ifndef INCLUDE_DYKNOW_NODEHANDLE_TIMER_HPP_
#define INCLUDE_DYKNOW_NODEHANDLE_TIMER_HPP_

#include <ros/ros.h>
#include "boost/function.hpp"
#include <boost/thread.hpp>
#include "dyknow_analytics/monitor.hpp"
#include "dyknow_analytics/analysis.hpp"

namespace dyknow {

class TimerState {
public:
	TimerState(ros::NodeHandle nh) : nh(nh), refCount(0) {}
	virtual ~TimerState() {}

	ros::NodeHandle nh;
	ros::Timer timer;
	unsigned int refCount;

	template<class T>
	void init(ros::Duration period, void(T::*fp)(const ros::TimerEvent&), T* obj, bool oneshot = false, bool autostart = true, bool decorate = true) {
		if(!decorate) {
			this->timer = nh.createTimer(period, fp, obj, oneshot, autostart);
		}
		else {
			const boost::function<void(const ros::TimerEvent&)>& callbackWrapper = boost::bind(&dyknow::TimerState::genericCallback<T>, this, _1, fp, obj);
			this->timer = nh.createTimer(period, callbackWrapper, oneshot, autostart);
		}
	}

	template<class T>
	void genericCallback(const ros::TimerEvent& data, void(T::*fp)(const ros::TimerEvent&), T* obj) {
		// Monitor the user-provided callback
		dyknow::Monitor start, end;
		start.init();
		(obj->*fp)(data);
		end.init();
		log(start, end);
	}

	std::vector<std::pair<dyknow::Monitor, dyknow::Monitor> > collectTrace();

private:
	std::vector<std::pair<dyknow::Monitor, dyknow::Monitor> > trace;
	boost::mutex traceAccess;

	void log(dyknow::Monitor first, dyknow::Monitor second);
};

class Timer {
public:
	Timer() : statePtr(NULL) {}

	Timer(ros::NodeHandle nh, TimerState* statePtr) : nh(nh), statePtr(statePtr) {
		statePtr->refCount++;
	}

	Timer& operator=(const Timer& other) {
		this->statePtr = other.statePtr;
		statePtr->refCount++;
		return *this;
	}

	~Timer() {
		statePtr->refCount--;
		if(statePtr->refCount == 0) { delete statePtr; }
	}

private:
	ros::NodeHandle nh;
	TimerState* statePtr;

};

} //namespace



#endif /* INCLUDE_DYKNOW_NODEHANDLE_TIMER_HPP_ */
