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
 * CityBuildingMap.cpp
 *
 * $Revision: 717 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <iostream>
#include <fstream>
#include <cmath>

#include <sys/resource.h>

#include "CityBuildingMap.h"
#include "CityGeographySpace.h"

namespace dtsim {

CityBuildingMap::CityBuildingMap()
{
	buildBuildingMap();
}

void CityBuildingMap::buildBuildingMap()
{
	std::vector<CityBuilding*> cityBuildings;
	City::instance()->getGeographySpace()->getLayerAgents<CityBuilding>(cityBuildings);

	AreaBuildingMapIterator areaIter;
	ZoneBuildingMapIterator zoneIter;
	BuildingTypeMapIterator typeIter;
	int area, zone, type;

	for (CityBuilding* B : cityBuildings) {
		area = B->getAreaCode();
		type = B->getBuildingType();

		/* step1: construct building map for all areas */
		areaIter = areaBuildingMap.find(area);
		if (areaIter == areaBuildingMap.end()) {
			/* if a area code is inserted for the first time */
			BuildingTypeMap buildingTypeMap;
			std::vector<CityBuilding*> building(1, B);
			buildingTypeMap[type] = building;

			areaBuildingMap[area] = buildingTypeMap;
		} else {
			/* if a area code already exists */
			typeIter = areaIter->second.find(type);
			if (typeIter == areaIter->second.end()) {
				/* if a type is inserted for the first time */
				std::vector<CityBuilding*> building(1, B);
				areaIter->second[type] = building;
			} else {
				/* if a building type already exists */
				typeIter->second.push_back(B);
			}
		}

		/* step2: construct building map for all zones */
	#if 0
		zone = B->getZoneCode();
		zoneIter = zoneBuildingMap.find(zone);
		if (zoneIter == zoneBuildingMap.end()) {
			/* if a zone code is inserted for the first time */
			BuildingTypeMap buildingTypeMap;
			std::vector<CityBuilding*> building(1, B);
			buildingTypeMap[type] = building;

			zoneBuildingMap[zone] = buildingTypeMap;
		} else {
			/* if a zone code already exists */
			typeIter = zoneIter->second.find(type);
			if (typeIter == zoneIter->second.end()) {
				/* if a type is inserted for the first time */
				std::vector<CityBuilding*> building(1, B);
				zoneIter->second[type] = building;
			} else {
				/* if a building type already exists */
				typeIter->second.push_back(B);
			}
		}
	#else
		std::vector<int> zones;
		zones = B->getZoneCodes();
		for(auto zone : zones) {
			if(zone == 1000022) {
				printf("building: type(%d)\n", type);
			}

			zoneIter = zoneBuildingMap.find(zone);
			if (zoneIter == zoneBuildingMap.end()) {
				/* if a zone code is inserted for the first time */
				BuildingTypeMap buildingTypeMap;
				std::vector<CityBuilding*> building(1, B);
				buildingTypeMap[type] = building;

				zoneBuildingMap[zone] = buildingTypeMap;
			} else {
				/* if a zone code already exists */
				typeIter = zoneIter->second.find(type);
				if (typeIter == zoneIter->second.end()) {
					/* if a type is inserted for the first time */
					std::vector<CityBuilding*> building(1, B);
					zoneIter->second[type] = building;
				} else {
					/* if a building type already exists */
					typeIter->second.push_back(B);
				}
			}
		}
	#endif
	}
}

std::vector<CityBuilding*> CityBuildingMap::getBuildingsByArea(int area)
{
	std::vector<CityBuilding*> buildings;

	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter != areaBuildingMap.end()) {
		for(auto typeBuildings : areaIter->second) {
			for(auto B : typeBuildings.second) {
				buildings.push_back(B);
			}
		}
	}

	return buildings;
}

std::map<int, std::vector<CityBuilding*>>* CityBuildingMap::getBuildingsPerTypeByArea(int area)
{
	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter != areaBuildingMap.end())
		return &areaIter->second;

	return nullptr;
}

std::vector<CityBuilding*> CityBuildingMap::getBuildingsByArea(int area, int type)
{
	std::vector<CityBuilding*> buildings;

	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter != areaBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = areaIter->second.find(type);
		if (typeIter != areaIter->second.end()) {
			for(auto B : typeIter->second) {
				buildings.push_back(B);
			}
		}
	}

	return buildings;
}

std::vector<CityBuilding*> CityBuildingMap::getBuildingsByZone(int zone)
{
	std::vector<CityBuilding*> buildings;

	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter != zoneBuildingMap.end()) {
		for(auto typeBuildings : zoneIter->second) {
			for(auto B : typeBuildings.second) {
				buildings.push_back(B);
			}
		}
	}

	return buildings;
}

std::map<int, std::vector<CityBuilding*>>* CityBuildingMap::getBuildingsPerTypeByZone(int zone)
{
	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter != zoneBuildingMap.end())
		return &zoneIter->second;

	return nullptr;
}

std::vector<CityBuilding*> CityBuildingMap::getBuildingsByZone(int zone, int type)
{
	std::vector<CityBuilding*> buildings;

	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter != zoneBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = zoneIter->second.find(type);
		if (typeIter != zoneIter->second.end()) {
			for(auto B : typeIter->second) {
				buildings.push_back(B);
			}
		}
	}

	return buildings;
}

CityBuilding* CityBuildingMap::getRandomBuildingByArea(int area, int type)
{
	CityBuilding* building = nullptr;

	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter != areaBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = areaIter->second.find(type);
		if (typeIter != areaIter->second.end()) {
			int pos = std::rand() % typeIter->second.size();
			building = typeIter->second[pos];
		}
	}

	return building;
}

CityBuilding* CityBuildingMap::getRandomBuildingByZone(int zone, int type)
{
	CityBuilding* building = nullptr;

	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter != zoneBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = zoneIter->second.find(type);
		if (typeIter != zoneIter->second.end()) {
			int pos = std::rand() % typeIter->second.size();
			building = typeIter->second[pos];
		}
	}

	return building;
}

CityBuilding* CityBuildingMap::getNearestBuildingByArea(double x, double y, int area, int type)
{
	CityBuilding* building = nullptr;

	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter != areaBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = areaIter->second.find(type);
		if (typeIter != areaIter->second.end()) {
			double distMin = 0xffffff;
			for (CityBuilding* B : typeIter->second) {
				double bx, by;
				B->getCoordinate(bx, by);

				double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
				if (dist < distMin) {
					building = B;
					distMin = dist;
				}
			}
		}
	}

	return building;
}

CityBuilding* CityBuildingMap::getNearestBuildingByArea(double x, double y, int area, int type, int limit)
{
	CityBuilding* building = nullptr;

	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter != areaBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = areaIter->second.find(type);
		if (typeIter != areaIter->second.end()) {
			double distMin = 0xffffff;
			int bsize = (int)typeIter->second.size();

			if ((limit == 0) || (bsize <= limit)) {
				for (CityBuilding* B : typeIter->second) {
					double bx, by;
					B->getCoordinate(bx, by);

					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			} else {
				for (int i = 0; i < limit; i++) {
					int pos = std::rand() % bsize;
					CityBuilding* B = typeIter->second[pos];

					double bx, by;
					B->getCoordinate(bx, by);
					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			}
		}
	}

	return building;
}

CityBuilding* CityBuildingMap::getNearestBuildingByArea(double x, double y, int area, int type, double probability)
{
	CityBuilding* building = nullptr;

	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter != areaBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = areaIter->second.find(type);
		if (typeIter != areaIter->second.end()) {
			double distMin = 0xffffff;
			int bsize = (int)typeIter->second.size();
			int limit = 100; 	/* min value */

			if ((limit == 0) || (bsize <= limit)) {
				for (CityBuilding* B : typeIter->second) {
					double bx, by;
					B->getCoordinate(bx, by);

					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			} else {
				limit = (int)(typeIter->second.size() * probability);
				for (int i = 0; i < limit; i++) {
					int pos = std::rand() % bsize;
					CityBuilding* B = typeIter->second[pos];

					double bx, by;
					B->getCoordinate(bx, by);

					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			}
		}
	}

	return building;
}

CityBuilding* CityBuildingMap::getNearestBuildingByZone(double x, double y, int zone, int type)
{
	CityBuilding* building = nullptr;

	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter != zoneBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = zoneIter->second.find(type);
		if (typeIter != zoneIter->second.end()) {
			double distMin = 0xffffff;
			for (CityBuilding* B : typeIter->second) {
				double bx, by;
				B->getCoordinate(bx, by);

				double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
				if (dist < distMin) {
					building = B;
					distMin = dist;
				}
			}
		}
	}

	return building;
}

CityBuilding* CityBuildingMap::getNearestBuildingByZone(double x, double y, int zone, int type, int limit)
{
	CityBuilding* building = nullptr;

	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter != zoneBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = zoneIter->second.find(type);
		if (typeIter != zoneIter->second.end()) {
			double distMin = 0xffffff;
			int bsize = (int)typeIter->second.size();

			if ((limit == 0) || (bsize <= limit)) {
				for (CityBuilding* B : typeIter->second) {
					double bx, by;
					B->getCoordinate(bx, by);

					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			} else {
				for (int i = 0; i < limit; i++) {
					int pos = std::rand() % bsize;
					CityBuilding* B = typeIter->second[pos];

					double bx, by;
					B->getCoordinate(bx, by);
					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			}
		}
	}

	return building;
}

CityBuilding* CityBuildingMap::getNearestBuildingByZone(double x, double y, int zone, int type, double probability)
{
	CityBuilding* building = nullptr;

	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter != zoneBuildingMap.end()) {
		BuildingTypeMapIterator typeIter = zoneIter->second.find(type);
		if (typeIter != zoneIter->second.end()) {
			double distMin = 0xffffff;
			int bsize = (int)typeIter->second.size();
			int limit = 100; 	/* min value */

			if ((limit == 0) || (bsize <= limit)) {
				for (CityBuilding* B : typeIter->second) {
					double bx, by;
					B->getCoordinate(bx, by);

					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			} else {
				limit = (int)(typeIter->second.size() * probability);
				for (int i = 0; i < limit; i++) {
					int pos = std::rand() % bsize;
					CityBuilding* B = typeIter->second[pos];

					double bx, by;
					B->getCoordinate(bx, by);

					double dist = std::sqrt(std::pow(x-bx,2) + std::pow(y-by,2));
					if (dist < distMin) {
						building = B;
						distMin = dist;
					}
				}
			}
		}
	}

	return building;
}

int CityBuildingMap::getBuildingCountByArea(int area)
{
	int count = 0;

	/* find typeMap for this area */
	AreaBuildingMapIterator areaIter = areaBuildingMap.find(area);
	if (areaIter == areaBuildingMap.end()) {
		return 0;
	}

	/* add size of building vector for each type */
	for(auto buildingType : areaIter->second) {
		count += buildingType.second.size();
	}

	return count;
}

int CityBuildingMap::getBuildingCountByZone(int zone)
{
	int count = 0;

	/* find typeMap for this zone */
	ZoneBuildingMapIterator zoneIter = zoneBuildingMap.find(zone);
	if (zoneIter == zoneBuildingMap.end()) {
		return 0;
	}

	/* add size of building vector for each type */
	for(auto buildingType : zoneIter->second) {
		count += buildingType.second.size();
	}

	return count;
}

} /* namespace dtsim */
