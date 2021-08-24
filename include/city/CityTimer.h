/*
 * Note: This license has also been called the "New BSD License" 
 * or "Modified BSD License".
 * 
 * Copyright (c) 2021 Electronics and Telecommunications Research 
 * Institute All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
*/

/* 
 * CityTimer.h
 *
 * Created on : May 21, 2019
 *		Author : sangmin
 *
 *  $Revision:$
 *  $LastChangedDate:$
 */

#ifndef CITY_TIMER_H_
#define CITY_TIMER_H_

#include <string>
#include <time.h>

namespace dtsim {

/**
 * \class CityTimer
 *
 * \brief
 * Timer implementation
 *
 * Calling start() starts the timer, getTimeString() after calling stop returns 
 * the elapsed time since calling start.
 */
class CityTimer {

public:
	struct timespec startT;
	struct timespec endT;
	double elapsedSec;
	double elapsedMSec;
	double elapsedUSec;
	double elapsedNSec;
	std::string timeStr;

	CityTimer() : elapsedNSec(0) {}
	~CityTimer() { }

	void start();
	void end();

	void lapStart();
	void lapEnd();
	void lapFinish();

	void calculateTime();
	void clearTime();

	std::string getElapsedTimeStr();
	double getElapsedSec();
	double getElapsedMSec();
	double getElapsedUSec();
	double getElapsedNSec();
};

}

#endif /* CITY_TIMER_H_ */
