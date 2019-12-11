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

#ifndef INCLUDE_DYKNOW_NODEHANDLE_IMAGE_SUBSCRIBER_HPP_
#define INCLUDE_DYKNOW_NODEHANDLE_IMAGE_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/function.hpp>
#include <dyknow_nodehandle/subscriber.hpp>


namespace dyknow {
namespace image {

class ImageSubscriber : public dyknow::SubscriberState {
private:
	image_transport::Subscriber imgSub;

public:
	ImageSubscriber() : SubscriberState() {}
	~ImageSubscriber() {}

	virtual void init(ros::NodeHandle nh, std::string chan, uint32_t queueSize, boost::function<void(std::string, uint32_t)> createSubscriberFn) {
		SubscriberState::init(nh, chan, queueSize, createSubscriberFn);
	}

	template<class M, class T>
	void delegateImgSubscribe(std::string topic, uint32_t queueSize, void(T::*fp)(M), T* obj, bool decorate = true) {
		image_transport::ImageTransport imgTransport(nh);
		if(!decorate) {
			// Delegates user callback directly to ROS
			this->imgSub = imgTransport.subscribe(topic, queueSize, fp, obj);
		}
		else {
			// Decorates user callback before delegating resulting method to ROS
			const boost::function<void (M)>& callbackWrapper = boost::bind(&dyknow::SubscriberState::genericCallback<M,T>, this, _1, fp, obj);
			this->imgSub = imgTransport.subscribe(topic, queueSize, callbackWrapper);
		}
		this->connected = true;
	}

	virtual std::string getTopic() { return imgSub.getTopic(); }
};

template<class M, class T>
static ImageSubscriber* createImageSubscriber(ros::NodeHandle nh, std::string chan, uint32_t queueSize, void(T::*fp)(M), T* obj, bool decorate = true) {
	ImageSubscriber* statePtr = new ImageSubscriber();
	statePtr->init(nh, chan, queueSize, boost::bind(&dyknow::image::ImageSubscriber::delegateImgSubscribe<M,T>, statePtr, _1, _2, fp, obj, decorate));
	return statePtr;
}

} //namespace
} //namespace


#endif /* INCLUDE_DYKNOW_NODEHANDLE_IMAGE_SUBSCRIBER_HPP_ */
