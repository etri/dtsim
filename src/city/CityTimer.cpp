/*                                                                                                                  
 *                                                                                                                   * Note: This license has also been called the "New BSD License" 
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
 * CityTimer.cpp
 *
 * Created on : May 21, 2019
 *		Author : sangmin
 *
 *  $Revision:$
 *  $LastChangedDate:$
 */

#include <iostream>
#include <sstream>

#include "CityTimer.h"

namespace dtsim {

void CityTimer::start() 
{
	clock_gettime(CLOCK_MONOTONIC, &startT);
}

void CityTimer::end() 
{
	clock_gettime(CLOCK_MONOTONIC, &endT); 
	elapsedNSec = ((endT.tv_sec - startT.tv_sec) * 1000000000LL) + (endT.tv_nsec - startT.tv_nsec);

	calculateTime();
}

void CityTimer::lapStart() 
{
	clock_gettime(CLOCK_MONOTONIC, &startT);
}

void CityTimer::lapEnd() 
{
	clock_gettime(CLOCK_MONOTONIC, &endT); 
	elapsedNSec += ((endT.tv_sec - startT.tv_sec) * 1000000000LL) + (endT.tv_nsec - startT.tv_nsec);
}

void CityTimer::lapFinish() 
{
	calculateTime();
}

void CityTimer::calculateTime()
{
	elapsedUSec = elapsedNSec / 1000LL;
	elapsedMSec = elapsedNSec / 1000000LL;
	elapsedSec  = elapsedNSec / 1000000000LL;

	int h = (int)elapsedSec / 3600;
	int m = (int)(((long)elapsedSec / 60) % 60);
	int s = (int)((long)elapsedSec % 60);

	std::ostringstream os;
	os.width(2);
	os.fill('0');
	os << h << ":" << m << ":" << s;

	timeStr = os.str();
}

std::string CityTimer::getElapsedTimeStr()
{
	return timeStr;
}

double CityTimer::getElapsedSec()
{
	return elapsedSec;
}

double CityTimer::getElapsedMSec() 
{ 
	return elapsedMSec; 
}

double CityTimer::getElapsedUSec() 
{ 
	return elapsedUSec; 
}

double CityTimer::getElapsedNSec() 
{ 
	return elapsedNSec; 
}

void CityTimer::clearTime()
{
	elapsedNSec = 0;
}

} /* namespace dtsim */
