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

#ifndef INCLUDE_MONITOR_HPP_
#define INCLUDE_MONITOR_HPP_

#include <sys/resource.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <ros/ros.h>
#include <dyknow_analytics/analysis.hpp>
#include <dyknow_analytics/Monitor.h>

namespace dyknow {

class Monitor {
public:
	pid_t pid;
	pid_t tid;
	ros::Time wallTime;
	ros::Time userTime;
	ros::Time systemTime;
	size_t curRSS;
	size_t maxRSS;

	std::string zeroPadLong(long num, int length) {
	    std::ostringstream ss;
	    ss << std::setw( length ) << std::setfill( '0' ) << num;
	    return ss.str();
	}

	pid_t gettid() {
		return syscall(SYS_gettid);
	}

	Monitor() : pid(0), tid(0), curRSS(0), maxRSS(0), suppress(false) {}
	~Monitor() {}

	void init() {
		struct rusage* rusagePtr = new rusage();

		// Collect process and thread identifiers
		this->pid = getpid();
		this->tid = gettid();

		// Collect rusage data
		if(getrusage(RUSAGE_THREAD, rusagePtr) != 0) {
			if(!suppress) { ROS_WARN("Failed call to getrusage for (%i, %i); suppressing repeat warnings", this->pid, this->tid); }
			this->wallTime = ros::Time();
			this->userTime = ros::Time();
			this->systemTime = ros::Time();
			this->maxRSS = 0;
			this->suppress = true;
		}
		else {
			this->wallTime = ros::Time::now();
			this->userTime = ros::Time(rusagePtr->ru_utime.tv_sec, rusagePtr->ru_utime.tv_usec * 1000);
			this->systemTime = ros::Time(rusagePtr->ru_stime.tv_sec, rusagePtr->ru_stime.tv_usec * 1000);
			this->maxRSS = rusagePtr->ru_maxrss;
			this->suppress = false;
		}

		// Finally also get the current RSS value
		this->curRSS = getCurrentRSS(tid);

		delete rusagePtr;
	}

	dyknow::Analysis operator-(const Monitor& other)	{
		dyknow::Analysis analysis;

		if(pid != other.pid || tid != other.tid) {
			ROS_WARN("PID/TID mismatch: (%i, %i) != (%i, %i)", pid, tid, other.pid, other.tid);
		}
		else if(wallTime < other.wallTime) {
			ROS_WARN("Incorrect temporal ordering for monitor comparison");
		}
		else {
			analysis.pid = pid;
			analysis.tid = tid;
			analysis.wallTime = wallTime - other.wallTime;
			analysis.userTime = userTime - other.userTime;
			analysis.systemTime = systemTime - other.systemTime;
			analysis.deltaRSS = curRSS - other.curRSS;
		}

		return analysis;
	}

	dyknow_analytics::Monitor toMsg() {
		dyknow_analytics::Monitor msg;
		msg.pid = pid;
		msg.tid = tid;
		msg.wall.data = wallTime;
		msg.user.data = userTime;
		msg.system.data = systemTime;
		msg.curRSS = curRSS;
		msg.maxRSS = maxRSS;
		return msg;
	}

private:
	bool suppress;

	size_t getCurrentRSS(pid_t tid) {
		/*
		 * Author:  David Robert Nadeau (modified by Daniel de Leng)
		 * Site:    http://NadeauSoftware.com/
		 * License: Creative Commons Attribution 3.0 Unported License
		 *          http://creativecommons.org/licenses/by/3.0/deed.en_US
		 */

		std::string path;
		std::ostringstream convert;
		convert << "/proc/" << tid << "/statm";
		path = convert.str();

		long rss = 0L;
		FILE* fp = NULL;
		//std::string path = "/proc/self/statm";
		if ( (fp = fopen( path.c_str(), "r" )) == NULL ) {
			ROS_WARN("Failed to open %s", path.c_str());
			return (size_t)0L;      /* Can't open? */
		}
		if ( fscanf( fp, "%*s%ld", &rss ) != 1 ) {
			ROS_WARN("Failed to read from %s", path.c_str());
			fclose( fp );
			return (size_t)0L;      /* Can't read? */
		}
		fclose( fp );
		return (size_t)rss * (size_t)sysconf( _SC_PAGESIZE);
	}
};

} // namespace



#endif /* INCLUDE_MONITOR_HPP_ */
