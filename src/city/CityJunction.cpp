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
 * CityJunction.cpp
 *
 * $Revision: 778 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "CityJunction.h"

namespace dtsim {

CityJunction::CityJunction(double id) : junctionId(id)
{}

std::unique_ptr<CityCoordinate> CityJunction::getCoordinate() const
{
	return getGeometry()->getCoordinate();
}

void CityJunction::getCoordinate(CityCoordinate& c) const
{
	getGeometry()->getCoordinate(c);
}

void CityJunction::getCoordinate(double& x, double& y) const
{
	getGeometry()->getCoordinate(x, y);
}

std::size_t CityJunction::getRoadSize() const
{
	return roadMap.size();
}

CityRoad* CityJunction::getRoad(double destJunctionId) const
{
	RoadMapIter iter = roadMap.find(destJunctionId);
	if (iter != roadMap.end())
		return iter->second;
	else
		return nullptr;
}

void CityJunction::getRoads(std::vector<double>& junctionIds, std::vector<CityRoad*>& roads) const
{
	RoadMapIter iter = roadMap.begin();
	while (iter != roadMap.end()) {
		junctionIds.push_back(iter->first);
		roads.push_back(iter->second);
		iter++;
	}
}

void CityJunction::addRoad(double destJunctionId, CityRoad* road)
{
	RoadMapIter iter = roadMap.find(destJunctionId);
	if (iter == roadMap.end())
		roadMap[destJunctionId] = road;
}

} /* namespace dtsim */
