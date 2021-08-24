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
 * CityBuildingMap.h
 *
 * $Revision: 717 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITYBUILDINGMAP_H
#define CITYBUILDINGMAP_H

#include <map>
#include <vector>

#include "CityBuilding.h"

namespace dtsim {

class CityBuildingMap {
private:
	/// building type map (building type, vector of CityBuilding)
	typedef typename std::map<int, std::vector<CityBuilding*>> BuildingTypeMap;
	typedef typename BuildingTypeMap::iterator BuildingTypeMapIterator;
	BuildingTypeMap buildingTypeMap;

	/// area building map (area id, building type map)
	typedef typename std::map<int, BuildingTypeMap> AreaBuildingMap;
	typedef typename AreaBuildingMap::iterator AreaBuildingMapIterator;
	AreaBuildingMap areaBuildingMap;

	/// zone building map (zone id, building type map)
	typedef typename std::map<int, BuildingTypeMap> ZoneBuildingMap;
	typedef typename ZoneBuildingMap::iterator ZoneBuildingMapIterator;
	ZoneBuildingMap zoneBuildingMap;

private:
	void buildBuildingMap();

public:
	CityBuildingMap();
	virtual ~CityBuildingMap() {}

public:
	/**
	 * Returns buildings map in area
	 *
	 * @param area area id
	 * @return buildings map : key(building type), value(vector of CityBuilding object pointer)
	 */
	std::vector<CityBuilding*> getBuildingsByArea(int area);
	std::map<int, std::vector<CityBuilding*>>* getBuildingsPerTypeByArea(int area);

	/**
	 * Returns buildings with the specified building type in area
	 *
	 * @param area area id
	 * @param type building type
	 * @return vector of CityBuilding object pointer
	 */
	std::vector<CityBuilding*> getBuildingsByArea(int area, int type);

	/**
	 * Returns buildings map in zone
	 *
	 * @param zone zone id
	 * @return buildings map : key(building type), value(vector of CityBuilding object pointer)
	 */
	std::vector<CityBuilding*> getBuildingsByZone(int zone);
	std::map<int, std::vector<CityBuilding*>>* getBuildingsPerTypeByZone(int zone);

	/**
	 * Returns buildings with the specified building type in zone
	 *
	 * @param zone zone id
	 * @param type building type
	 * @return vector of CityBuilding object pointer
	 */
	std::vector<CityBuilding*> getBuildingsByZone(int zone, int type);

	/**
	 * Returns random building with the specified building type in area
	 *
	 * @param area area id
	 * @param type building type
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getRandomBuildingByArea(int area, int type);

	/**
	 * Returns random building with the specified building type in zone
	 *
	 * @param zone zone id
	 * @param type building type
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getRandomBuildingByZone(int zone, int type);

	/**
	 * Returns nearest building with the specified building type from x-y coordinate in area
	 *
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param area area id
	 * @param type building type
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getNearestBuildingByArea(double x, double y, int area, int type);

	/**
	 * Returns nearest building with the specified building type from x-y coordinate in area
	 * Buildings are scanned randomly by limit
	 *
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param area area id
	 * @param type building type
	 * @param limit scan limit
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getNearestBuildingByArea(double x, double y, int area, int type, int limit);

	/**
	 * Returns nearest building with the specified building type from x-y coordinate in area
	 * Buildings are scanned randomly by probability
	 *
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param area area id
	 * @param type building type
	 * @param probability scan probability
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getNearestBuildingByArea(double x, double y, int area, int type, double probability);

	/**
	 * Returns nearest building with the specified building type from x-y coordinate in zone
	 *
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param zone zone id
	 * @param type building type
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getNearestBuildingByZone(double x, double y, int zone, int type);

	/**
	 * Returns nearest building with the specified building type from x-y coordinate in zone
	 * Buildings are scanned randomly by limit
	 *
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param zone zone id
	 * @param type building type
	 * @param limit scan limit
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getNearestBuildingByZone(double x, double y, int zone, int type, int limit);

	/**
	 * Returns nearest building with the specified building type from x-y coordinate in zone
	 * Buildings are scanned randomly by probability
	 *
	 * @param x x coordinate
	 * @param y y coordinate
	 * @param zone zone id
	 * @param type building type
	 * @param probability scan probability
	 * @return CityBuilding object pointer
	 */
	CityBuilding* getNearestBuildingByZone(double x, double y, int zone, int type, double probability);

	int getBuildingCountByArea(int area);
	int getBuildingCountByZone(int zone);
};

} /* namespace dtsim */

#endif /* CITYBUILDINGMAP_H */
