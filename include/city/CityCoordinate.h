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
 * CityCoordinate.h
 *
 * $Revision: 630 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_COORDINATE_H
#define CITY_COORDINATE_H

#include <iostream>
#include <string>

#include "geos/geom/Coordinate.h"
#include "geos/geom/CoordinateFilter.h"

namespace dtsim {

/**
 * \class CityCoordinate
 *
 * \brief
 * Coordinate information
 */
class CityCoordinate {
public:
	/// x-coordinate (longitude)
	double x;

	/// y-coordinate (latitude)
	double y;

	/**
	 * Creates a CityCoordinate
	 *
	 * @param x_ x-coordinate(longitude)
	 * @param y_ y-coordinate(latitude)
	 */
	CityCoordinate(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
	virtual ~CityCoordinate() {}

	/// Returns true if this coordinate is identical with given xy coordinate
	bool equals(double x_, double y_) const;

	/// Returns true if this coordinate is identical with given coordinate
	bool equals(const CityCoordinate& coord) const;

	/// Returns true if this coordinate is identical with given coordinate
	bool equals(const CityCoordinate* coord) const;

	/// Returns distance from this coordinate to given xy coordinate
	double distance(double x_, double y_) const;

	/// Returns distance from this coordinate to given coordinate
	double distance(const CityCoordinate& coord) const;

	/// Returns distance from this coordinate to given coordinate
	double distance(const CityCoordinate* coord) const;

	/// Pushes coordinate into the provided vector
	void toVector(std::vector<double>& coord) const;

	/// Returns string representation of coordinate
	std::string toString() const;
};

std::ostream& operator<<(std::ostream& os, const CityCoordinate& c);
std::ostream& operator<<(std::ostream& os, const CityCoordinate* c);
bool operator==(const CityCoordinate& c1, const CityCoordinate& c2);
bool operator!=(const CityCoordinate& c1, const CityCoordinate& c2);

/**
 * \class CityCoordinateFilter
 *
 * \brief
 * Coordinate filter to apply to coordinate
 */
class CityCoordinateFilter: public geos::geom::CoordinateFilter {
private:
	double x;
	double y;

public:
	CityCoordinateFilter(double x_, double y_) : x(x_), y(y_) {}

	virtual ~CityCoordinateFilter() {}

	void filter_rw(geos::geom::Coordinate* geosCoord) const;
};

} /* namespace dtsim */

#endif /* CITY_COORDINATE_H */
