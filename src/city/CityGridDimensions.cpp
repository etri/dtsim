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
 * CityGridDimensions.cpp
 *
 * $Revision$
 * $LastChangedDate$
 */

#include "CityGridDimensions.h"
#include "CityException.h"

namespace dtsim {

CityGridDimensions::CityGridDimensions()
: _origin(1, 0), _extent(1, 0)
{}

CityGridDimensions::CityGridDimensions(std::vector<double> extent)
: _origin(extent.size(), 0), _extent(extent)
{}

CityGridDimensions::CityGridDimensions(std::vector<double> origin, std::vector<double> extent)
: _origin(origin), _extent(extent)
{}

bool CityGridDimensions::contains(double x, double y) const
{
	if (_origin.size() != 2)
		throw CityException(__func__, "not equal dimensions");

	if ((x < _origin[0]) || (x >= (_origin[0] + _extent[0])))
		return false;

	if ((y < _origin[1]) || (y >= (_origin[1] + _extent[1])))
		return false;

	return true;
}

bool CityGridDimensions::contains(std::vector<double>& point) const
{
	if (point.size() != _origin.size())
		throw CityException(__func__, "not equal dimensions");

	for (int i = 0, n = point.size(); i < n; i++) {
		double start = _origin[i];
		double end = start + _extent[i];

		if ((point[i] < start) || (point[i] >= end))
			return false;
	}

	return true;
}

bool operator==(const CityGridDimensions &one, const CityGridDimensions &two)
{
	return ((one._extent == two._extent) && (one._origin == two._origin));
}

bool operator!=(const CityGridDimensions &one, const CityGridDimensions &two)
{
	return !(one == two);
}

} /* namespace dtsim */
