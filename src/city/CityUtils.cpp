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
 
/**
 * CityUtils.cpp
 *
 * $Revision: 662 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "CityUtils.h"
#include "CityException.h"

namespace dtsim {

double random_next_double()
{
	return repast::Random::instance()->nextDouble();
}

RandomUniformInt random_uniform_int(int from, int to)
{
	return repast::Random::instance()->createUniIntGenerator(from, to);
}

RandomUniformDouble random_uniform_double(double from, double to)
{
	return repast::Random::instance()->createUniDoubleGenerator(from, to);
}

RandomNormal random_normal(double mean, double sigma)
{
	return repast::Random::instance()->createNormalGenerator(mean, sigma);
}

RandomCauchy random_cauchy(double median, double sigma)
{
	return repast::Random::instance()->createCauchyGenerator(median, sigma);
}

RandomExponential random_exponential(double lambda)
{
	return repast::Random::instance()->createExponentialGenerator(lambda);
}

double degreesToRadians(double degrees)
{
	return (degrees * boost::math::constants::pi<double>()) / 180;
}

double radiansToDegrees(double radians)
{
	return (radians * 180) / boost::math::constants::pi<double>();
}

std::string currentDateTime(int type)
{
	time_t now = time(0);
	struct tm ts;
	char buf[128];

	ts = *localtime(&now);
	
	switch (type) {
	case 1:
		strftime(buf, sizeof(buf), "%Y-%m-%d", &ts);
		break;

	case 2:
		strftime(buf, sizeof(buf), "%Y/%m/%d", &ts);
		break;

	case 3:
		strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ts);
		break;

	case 4:
		strftime(buf, sizeof(buf), "%Y/%m/%d %H:%M:%S", &ts);
		break;

	case 5:
	default:
		strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &ts);
		break;
	}

	std::string dateStr(buf);

	return dateStr;
}

bool directoryExists(std::string dir)
{
	struct stat st;
	if(stat(dir.c_str(), &st) == 0) {
		return true;
	} else {
		return false;
	}
}

bool directoryCreate(std::string dir)
{
	if(mkdir(dir.c_str(), 0777) < 0) {
		return false;
	}
	return true;
}

int strToInt(const std::string& val)
{
	int i;
	std::istringstream stream(val);
	if (stream >> i)
		return i;
	else
		throw CityException(__func__, "value is not convertible to integer");
}

double strToDouble(const std::string& val)
{
	double i;
	std::istringstream stream(val);
	if (stream >> i)
		return i;
	else
		throw CityException(__func__, "value is not convertible to double");
}

} /* namespace dtsim */
