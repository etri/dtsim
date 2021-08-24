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
 * CityBusStop.cpp
 *
 * $Revision$
 * $LastChangedDate$
 */

#include "CityBusStop.h"
#include "CityBusLine.h"

namespace dtsim {

CityBusStop::CityBusStop(double stopId, std::string stopName)
: busStopId(stopId), busStopName(stopName)
{}

std::pair<int, int> CityBusStop::getBusStopOrder(double lineId) const
{
	std::pair<BusLineMapConstIterator, BusLineMapConstIterator> range = busLines.equal_range(lineId);

	BusLineMapConstIterator iter = range.first;
	BusLineMapConstIterator iterEnd = range.second;

	if (iter != iterEnd) {
		int first, second;

		first = second = iter->second.second;
		iter++;

		if (iter != iterEnd)
			second = iter->second.second;

		return std::make_pair(first, second);
	}

	return std::make_pair(-1, -1);
}

CityBusLine* CityBusStop::getBusLine(double lineId) const
{
	BusLineMapConstIterator iter = busLines.find(lineId);
	if (iter != busLines.end())
		return iter->second.first;

	return nullptr;
}

void CityBusStop::getBusLines(std::vector<CityBusLine*> filterLines, std::vector<CityBusLine*>& lines) const
{
	BusLineMapConstIterator iter = busLines.begin();
	while (iter != busLines.end()) {
		CityBusLine* busLine = iter->second.first;

		int i = 0;
		for (i = 0; i < filterLines.size(); i++) {
			CityBusLine* F = filterLines[i];
			if (F->getBusLineId() == iter->first)
				break;
		}
		if (i < filterLines.size())
			lines.push_back(iter->second.first);

		lines.push_back(iter->second.first);
		iter++;
	}
}

void CityBusStop::getBusLines(std::vector<CityBusLine*>& lines) const
{ 
	BusLineMapConstIterator iter = busLines.begin(); 
	while (iter != busLines.end()) { 
		CityBusLine* busLine = iter->second.first; 
		lines.push_back(iter->second.first); 
		iter++; 
	}
}

void CityBusStop::getBusLines(std::vector<std::pair<CityBusLine*, int>>& lines) const
{
	BusLineMapConstIterator iter = busLines.begin();
	while (iter != busLines.end()) {
		lines.push_back(std::make_pair(iter->second.first, iter->second.second));
		iter++;
	}
}

bool CityBusStop::hasBusLine(double lineId) const
{
	BusLineMapConstIterator iter = busLines.find(lineId);
	if (iter == busLines.end())
		return false;

	return true;
}

void CityBusStop::addBusLine(CityBusLine* busLine, int order)
{
	double lineId = busLine->getBusLineId();

	std::pair<BusLineMapIterator, BusLineMapIterator> range = busLines.equal_range(lineId);

	if (order < 0) {
		std::cout << "[" << order << "] BusStop[" << busStopId;
		std::cout << "-" << busStopName << "] lines=" << busLines.size() << std::endl;
	}

	BusLineMapIterator iter = range.first;
	BusLineMapIterator iterEnd = range.second;

	while (iter != iterEnd) {
		if (iter->second.second == order)
			return;
		iter++;
	}

	busLines.emplace(lineId, std::make_pair(busLine, order));
}

void CityBusStop::removeBusLine(double lineId)
{
	std::pair<BusLineMapIterator, BusLineMapIterator> range = busLines.equal_range(lineId);

	BusLineMapIterator iter = range.first;
	BusLineMapIterator iterEnd = range.second;

	while (iter != iterEnd)
		iter = busLines.erase(iter);
}

std::unique_ptr<CityCoordinate> CityBusStop::getCoordinate() const
{
	return getGeometry()->getCoordinate();
}

void CityBusStop::getCoordinate(CityCoordinate& c) const
{
	getGeometry()->getCoordinate(c);
}

void CityBusStop::getCoordinate(double& x, double& y) const
{
	getGeometry()->getCoordinate(x, y);
}

} /* namespace dtsim */
