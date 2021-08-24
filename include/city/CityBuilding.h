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
 * CityBuilding
 *
 * $Revision: 765 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_BUILDING_H
#define CITY_BUILDING_H

#include <memory>

#include "CityAgent.h"

namespace dtsim {

/**
 * \class CityBuilding
 *
 * \brief
 * Building agent implementation
 *
 * Building information is provided through shapefile.
 * In general, the geometry of a building is polygonal.
 */
class CityBuilding: public CityAgent {
protected:
	/// building identifier
	double buildingId;

	/// building type
	double buildingType;

	/// tmp: building name
	std::string buildingName;

public:
	/**
	 * Creates a CityBuilding
	 *
	 * @param id building identifier
	 * @param type building type
	 */
	CityBuilding(double id, double type);
	
	// tmp
	CityBuilding(double id, double type, std::string name);

	virtual ~CityBuilding() {}

	/// Returns a building identifier
	double getBuildingId() const {
		return buildingId;
	}

	/// Returns a building type
	double getBuildingType() const {
		return buildingType;
	}

	/// tmp: Returns a building name
	std::string getBuildingName() const {
		return buildingName;
	}

	/// Returns a coordinate sequence of a building geometry
	std::unique_ptr<CityCoordinateSequence> getCoordinates() const;

	/// Pushes a coordinate sequence into the provided CityCoordinateSequence
	void getCoordinates(CityCoordinateSequence& seq) const;

	/// Returns a coordinate of a building's centroid
	std::unique_ptr<CityCoordinate> getCoordinate() const;

	/// Pushes a building's centroid coordinate into the provided CityCoordinate
	void getCoordinate(CityCoordinate& c) const;

	/// Pushes a building's centroid coordinate into the provided variables
	void getCoordinate(double& x, double& y) const;
};

} /* namespace dtsim */

#endif /* CITY_BUILDING_H */
