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

#ifndef INCLUDE_DYKNOW_ANALYTICS_ANALYSIS_HPP_
#define INCLUDE_DYKNOW_ANALYTICS_ANALYSIS_HPP_

#include <ros/ros.h>

namespace dyknow {

struct Analysis {
	pid_t pid;
	pid_t tid;
	ros::Duration wallTime;
	ros::Duration userTime;
	ros::Duration systemTime;
	size_t deltaRSS;
	std::string label;

	void print() {
		ROS_INFO("[label: %s; PID: %i; TID: %i; wall: %f sec; utime: %f sec; stime: %f sec; rss: %lu bytes]", label.c_str(), pid, tid, wallTime.toSec(), userTime.toSec(), systemTime.toSec(), deltaRSS);
	}

	Analysis() : tid(0), pid(0), deltaRSS(0), label("Anonymous") {}
};

}



#endif /* INCLUDE_DYKNOW_ANALYTICS_ANALYSIS_HPP_ */
