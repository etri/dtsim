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
 * CityGridDimensions.h
 *
 * $Revision$
 * $LastChangedDate$
 */

#ifndef CITY_GRID_DIMENSIONS_H
#define CITY_GRID_DIMENSIONS_H

#include <iostream>
#include <vector>

namespace dtsim {

/**
 * \class CityGridDimensions
 *
 * \brief
 * Grid dimensions implementation for specifying grid dimensions.
 * origin of (-100, -100) and extent of (200, 200) represents a
 * rectangle with corners at (-100, -100), (-100, 100), (100, 100),
 * and (100, -100).
 */
class CityGridDimensions {
private:
	std::vector<double> _origin;
	std::vector<double> _extent;

	friend bool operator==(const CityGridDimensions &one, const CityGridDimensions &two);

public:
	CityGridDimensions();
	explicit CityGridDimensions(std::vector<double> extent);

	/**
	 * Creates a CityGridDimensions with the specified origin and extent
	 */
	CityGridDimensions(std::vector<double> origin, std::vector<double> extent);

	bool contains(double x, double y) const;
	bool contains(std::vector<double>& point) const;

	/// Gets the origin
	std::vector<double> origin() const {
		return _origin;
	}

	/// Gets the extent
	std::vector<double> extent() const {
		return _extent;
	}

	double origin(int index) const {
		return _origin[index];
	}

	double extent(int index) const {
		return _extent[index];
	}

	/// Gets the dimension count
	size_t dimensionCount() const {
		return _extent.size();
	}
};

bool operator==(const CityGridDimensions &one, const CityGridDimensions &two);
bool operator!=(const CityGridDimensions &one, const CityGridDimensions &two);

} /* namespace dtsim */

#endif /* CITY_GRID_DIMENSIONS_H */
