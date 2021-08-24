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
 * CityJunction.h
 *
 * $Revision: 746 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */
 
#ifndef CITY_JUNCTION_H
#define CITY_JUNCTION_H

#include <unordered_map>
#include <vector>
#include <memory>

#include "CityAgent.h"
#include "CityRoad.h"

namespace dtsim {

/**
 * \class CityJunction
 *
 * \brief
 * Junction agent implementation
 *
 * Junction information is provided through shapefile.
 * In general, the geometry of a junction is point.
 */
class CityJunction: public CityAgent {
protected:
	/// junction identifier
	double junctionId;

	/// road map connected with this junction (destination junction id, CityRoad)
	typedef typename std::unordered_map<double, CityRoad*> RoadMap;
	typedef typename RoadMap::const_iterator RoadMapIter;
	RoadMap roadMap;

public:
	/**
	 * Creates a CityJunction
	 *
	 * @param id junction identifier
	 */
	CityJunction(double id);
	virtual ~CityJunction() {}

	/// Returns a junction identifier
	double getJunctionId() const {
		return junctionId;
	}

	/// Returns a coordinate of a junction geometry
	std::unique_ptr<CityCoordinate> getCoordinate() const;

	/// Returns a coordinate of a junction geometry
	void getCoordinate(CityCoordinate& c) const;

	/// Returns a coordinate of a junction geometry
	void getCoordinate(double& x, double& y) const;

	/// Returns a number of road connected with this junction
	std::size_t getRoadSize() const;

	/**
	 * Get road object connecting this junction with specified destination junction
	 *
	 * @param destJunctionId destination junction ID
	 * @return road object
	 */
	CityRoad* getRoad(double destJunctionId) const;

	/**
	 * Get destination junction ids and road objects connected with this junction
	 *
	 * @param junctionIds returned destination junction IDs
	 * @param roads returned road objects
	 */
	void getRoads(std::vector<double>& junctionIds, std::vector<CityRoad*>& roads) const;

	/**
	 * Add road object connecting this junction with specified destination junction
	 *
	 * @param destJunctionId destination junction ID
	 * @param road road object connecting connecting this junction with specified destination junction
	 */
	void addRoad(double destJunctionId, CityRoad* road);
};

} /* namespace dtsim */

#endif /* CITY_JUNCTION_H */
