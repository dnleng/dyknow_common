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

#ifndef INCLUDE_DYKNOW_NODEHANDLE_IMAGE_PUBLISHER_HPP_
#define INCLUDE_DYKNOW_NODEHANDLE_IMAGE_PUBLISHER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/function.hpp>
#include <dyknow_nodehandle/publisher.hpp>


namespace dyknow {
namespace image {

class ImagePublisher : public PublisherState {
private:
	image_transport::Publisher imgPub;

public:
	ImagePublisher() : PublisherState() {}
	virtual ~ImagePublisher() {}

	virtual void init(ros::NodeHandle nh, std::string chan, uint32_t queueSize, bool latch, boost::function<void(std::string, uint32_t)> createPublisherFn) {
		PublisherState::init(nh, chan, queueSize, latch, createPublisherFn);
	}

	void delegateImgAdvertise(std::string topic, uint32_t queueSize, bool latch = false) {
		image_transport::ImageTransport imgTransport(nh);
		imgPub = imgTransport.advertise(topic, queueSize, latch);
		connected = true;
	}

	template<class M>
	void publish(const M& msg) {
		imgPub.publish(msg);
		sent++;
	}

	virtual uint32_t getNumSubscribers() const {
		return connected ? imgPub.getNumSubscribers() : 0;
	}

	virtual std::string getTopic() { return imgPub.getTopic(); }
};

static ImagePublisher* createImagePublisher(ros::NodeHandle nh, std::string chan, uint32_t queueSize, bool latch = false) {
	ImagePublisher* statePtr = new ImagePublisher();
	statePtr->init(nh, chan, queueSize, latch, boost::bind(&dyknow::image::ImagePublisher::delegateImgAdvertise, statePtr, _1, _2, latch));
	return statePtr;
}

} //namespace
} //namespace



#endif /* INCLUDE_DYKNOW_NODEHANDLE_IMAGE_PUBLISHER_HPP_ */
