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

/**
 * CityUtils.h
 *
 * $Revision: 662 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_UTIL_H
#define CITY_UTIL_H

#include <boost/mpi/communicator.hpp>
#include <boost/math/constants/constants.hpp>
#include <string>
#include "repast_hpc/Random.h"

namespace dtsim {

typedef repast::IntUniformGenerator RandomUniformInt;
typedef repast::DoubleUniformGenerator RandomUniformDouble;
typedef repast::NormalGenerator RandomNormal;
typedef repast::CauchyGenerator RandomCauchy;
typedef repast::ExponentialGenerator RandomExponential;

/**
 * Gets the next double in the range[0, 1)
 *
 * @return the next double in the range[0, 1)
 */
double random_next_double();

/** 
 * Creates a generator that produces ints in the range[from, to]
 *
 * @param from the range start(inclusive)
 * @param to the range end(inclusive)
 * @return a generator that produces ints in the range[from, to]
 */
RandomUniformInt random_uniform_int(int from, int to);

/**
 * Creates a generator that produces doubles in the range[from, to)
 *
 * @param from the range start(inclusive)
 * @param to the range end(exclusive)
 * @return a generator that produces doubles in the range[from, to)
 */
RandomUniformDouble random_uniform_double(double from, double to);

/** 
 * Creates a normal generator
 *  
 * @param mean
 * @param sigma
 * @return a normal generator
 */
RandomNormal random_normal(double mean, double sigma);

/** 
 * Creates a cauchy generator
 *
 * @param median
 * @param sigma
 * @return a cauchy generator
 */
RandomCauchy random_cauchy(double median, double sigma);

/**
 * Creates an exponential generator
 *
 * @param lambda must be > 0
 * @return an exponential generator
 */
RandomExponential random_exponential(double lambda);

/**
 * Convert degree to radian
 *
 * @param degrees
 * @return radian
 */
double degreesToRadians(double degrees);

/**
 * Convert radian to degree
 *
 * @param radians
 * @return degree
 */
double radiansToDegrees(double radians);

/**
 * Get current date string 
 *  - Type 1 : YYYY-MM-DD
 *  - Type 2 : YYYY/MM/DD
 *  - Type 3 : YYYY-MM-DD HH:MM:SS
 *  - Type 4 : YYYY/MM/DD HH:MM:SS
 *  - Type 5 : YYYYMMDD-HHMMSS (default)
 *
 * @param type
 * @return current date string
 */
std::string currentDateTime(int type = 5);

/**
 * check if directory exists
 * @param dir
 * @return true or false
 */
bool directoryExists(std::string dir);

/**
 * create directory
 * @param dir
 * @return true or false
 */
bool directoryCreate(std::string dir);

/**
 * Converts the string to a int.
 * @param val
 * @return int
 */
int strToInt(const std::string& val);

/**
 * Converts the string to a double.
 * @param val
 * @return double
 */
double strToDouble(const std::string& val);

} /* namespace dtsim */

#endif /* CITY_UTIL_H */
