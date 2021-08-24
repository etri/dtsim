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
 * CityCoordinate.cpp
 *
 * $Revision: 625 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <sstream>
#include <iomanip>
#include <cmath>

#include "CityCoordinate.h"

namespace dtsim {

void CityCoordinateFilter::filter_rw(geos::geom::Coordinate* geosCoord) const
{
	geosCoord->x = x;
	geosCoord->y = y;
}

bool CityCoordinate::equals(double x_, double y_) const
{
	if (x != x_)
		return false;
	if (y != y_)
		return false;

	return true;
}

bool CityCoordinate::equals(const CityCoordinate& coord) const
{
	if (x != coord.x)
		return false;
	if (y != coord.y)
		return false;

	return true;
}

bool CityCoordinate::equals(const CityCoordinate* coord) const
{
	if (x != coord->x)
		return false;
	if (y != coord->y)
		return false;

	return true;
}

double CityCoordinate::distance(double x_, double y_) const
{
	double dx = x - x_;
	double dy = y - y_;
	return std::sqrt(dx*dx + dy*dy);
}

double CityCoordinate::distance(const CityCoordinate& coord) const
{
	double dx = x - coord.x;
	double dy = y - coord.y;
	return std::sqrt(dx*dx + dy*dy);
}

double CityCoordinate::distance(const CityCoordinate* coord) const
{
	double dx = x - coord->x;
	double dy = y - coord->y;
	return std::sqrt(dx*dx + dy*dy);
}

std::string CityCoordinate::toString() const
{
	std::ostringstream s;
	s << std::setprecision(17) << *this;
	return s.str();
}

std::ostream& operator<<(std::ostream& os, const CityCoordinate& c)
{
	os << c.x << " " << c.y;
	return os;
}

std::ostream& operator<<(std::ostream& os, const CityCoordinate* c)
{
	os << c->x << " " << c->y;
	return os;
}

bool operator==(const CityCoordinate& c1, const CityCoordinate& c2)
{
	return c1.equals(c2);
}

bool operator!=(const CityCoordinate& c1, const CityCoordinate& c2)
{
	return !c1.equals(c2);
}

} /* namespace dtsim */
