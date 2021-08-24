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
 * CityRoad.h
 *
 * $Revision: 746 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_ROAD_H
#define CITY_ROAD_H

#include <vector>
#include <memory>

#include "CityAgent.h"

namespace dtsim {

/**
 * \class CityRoad
 *
 * \brief
 * Road agent implementation
 *
 * Road information is provided through shapefile.
 * In general, the geometry of a road is arc(linestring).
 */
class CityRoad: public CityAgent {
protected:
	/// road identifier
	double roadId;

	/// start junction of the road
	double startJunctionId;

	/// end junction of the road
	double endJunctionId;

	/// number of lanes
	double lanes;

	/// max speed limit
	double maxSpeed;

	/// length of a road
	double length;

	/// vector of each distance between each pair of consecutive verties
	std::vector<double> distances;

public:
	/**
	 * Creates a CityRoad
	 *
	 * @param id road identifier
	 */
	CityRoad(double roadId_, double startJunctionId_, double endJunctionId_, double lanes_, double maxSpeed_, double length_);
	virtual ~CityRoad() {}

	/// Returns a road identifier
	double getRoadId() const {
		return roadId;
	}

	/// Returns a start junction object
	double getStartJunctionId() const {
		return startJunctionId;
	}

	/// Returns a end junction object
	double getEndJunctionId() const {
		return endJunctionId;
	}

	/// Returns number of lanes
	double getLanes() const {
		return lanes;
	}

	/// Returns max speed limit
	double getMaxSpeed() const {
		return maxSpeed;
	}

	/// Returns a road length
	double getLength() const {
		return length;
	}

	/// Returns a coordinate sequence
	std::unique_ptr<CityCoordinateSequence> getCoordinates() const;

	/// Pushes a coordinate sequence into the provided CoordinateSequence
	void getCoordinates(CityCoordinateSequence& seq) const;

	/// Returns a vector of distances
	std::vector<double> getDistances() const {
		return distances;
	}

	void makeDistances(const CityCoordinateSequence* seq);
};

} /* namespace dtsim */

#endif /* CITY_ROAD_H */
