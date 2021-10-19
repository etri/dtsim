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
 * CityArea
 *
 * $Revision: 444 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_AREA_H
#define CITY_AREA_H

#include <string>
#include <memory>
#include <map>

#include "CityAgent.h"
#include "CityZone.h"

namespace dtsim {

/**
 * \class CityArea
 *
 * \brief
 * Area agent implementation
 *
 * Area information is provided through shapefile.
 * In general, the geometry of a area is polygonal.
 */
class CityArea: public CityAgent {
protected:
	/// area identifier
	int areaId;
	std::string areaName;
	std::map<int, CityZone*> zones;

public:
	/**
	 * Creates a CityArea
	 *
	 * @param id area identifier
	 */
	CityArea(int id, std::string name);
	virtual ~CityArea() {}

	/// Returns a area identifier
	int getAreaCode() const {
		return areaId;
	}

	/// Returns a area name
	std::string getAreaName() const {
		return areaName;
	}

	/// add zone to this area
	void addZone(int zoneId, CityZone *zone) {
		zones[zoneId] = zone;
	}

	/// remove zone
	void removeZone(int zonId) {
	}

	/// Returns zone list
	std::map<int, CityZone*> getZones() const {
		return zones;
	}

	/// Returns a coordinate sequence of a area geometry
	std::unique_ptr<CityCoordinateSequence> getCoordinates() const;

	/// Pushes a coordinate sequence into the provided CoordinateSequence
	void getCoordinates(CityCoordinateSequence& seq) const;
};

} /* namespace dtsim */

#endif /* CITY_AREA_H */
