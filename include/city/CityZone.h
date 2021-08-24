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
 * CityZone
 *
 * $Revision: 444 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_ZONE_H
#define CITY_ZONE_H

#include <string>
#include <memory>

#include "CityAgent.h"

namespace dtsim {

/**
 * \class CityZone
 *
 * \brief
 * Zone agent implementation
 *
 * Zone information is provided through shapefile.
 * In general, the geometry of a area is polygonal.
 */
class CityZone: public CityAgent {
protected:
	/// area identifier
	int areaId;
	int zoneId;
	std::string zoneName;

public:
	/**
	 * Creates a CityZone
	 *
	 * @param id area identifier
	 */
	CityZone(int id, int area, std::string name);
	virtual ~CityZone() {}

	/// Returns a area identifier
	int getZoneCode() const {
		return zoneId;
	}

	/// Returns a area identifier
	int getAreaCode() const {
		return areaId;
	}

	/// Returns a area name
	std::string getZoneName() const {
		return zoneName;
	}

	/// Returns a coordinate sequence of a area geometry
	std::unique_ptr<CityCoordinateSequence> getCoordinates() const;

	/// Pushes a coordinate sequence into the provided CoordinateSequence
	void getCoordinates(CityCoordinateSequence& seq) const;
};

} /* namespace dtsim */

#endif /* CITY_ZONE_H */
