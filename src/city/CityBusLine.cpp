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
 * CityBusLine.cpp
 *
 * $Revision$
 * $LastChangedDate$
 */

#include <algorithm>

#include "CityBusLine.h"
#include "CityBusStop.h"
#include "CityGeodeticCalculator.h"
#include "CityException.h"

namespace dtsim {

CityBusLineSection::CityBusLineSection(double fromStopId, double toStopId, CityGeometry* geom)
: fromBusStopId(fromStopId), toBusStopId(toStopId), geometry(geom), length(0)
{
	CityCoordinateSequence seq;
	geometry->getCoordinates(seq);

	std::size_t count = seq.getSize() - 1;
	double x1, y1, x2, y2;
	double meter;

	for (std::size_t i = 0; i < count; i++) {
		seq.getAt(i, x1, y1);
		seq.getAt(i+1, x2, y2);

		meter = CityGeodeticCalculator::instance()->getDistance(x1, y1, x2, y2);
		length += meter;
		distances.push_back(meter);
	}
}

CityBusLineSection::~CityBusLineSection()
{
	delete geometry;
}

std::unique_ptr<CityCoordinateSequence> CityBusLineSection::getCoordinates() const
{
	return geometry->getCoordinates();
}

void CityBusLineSection::getCoordinates(CityCoordinateSequence& seq) const
{
	geometry->getCoordinates(seq);
}

/*
 * Constructor and Destructor
 */
CityBusLine::CityBusLine(double id, std::string name) : busLineId(id), busLineName(name)
{}

CityBusLine::~CityBusLine()
{
	BusLineSectionMapIterator iter = busLineSections.begin();
	while (iter != busLineSections.end()) {
		CityBusLineSection* section = iter->second;
		iter = busLineSections.erase(iter);
		delete section;
	}
}

CityBusStop* CityBusLine::getBusStop(double stopId) const
{
	BusStopMapConstIterator iter = busStops.find(stopId);
	if (iter != busStops.end())
		return iter->second.first;

	return nullptr;
}

void CityBusLine::getBusStops(std::vector<CityBusStop*>& stops) const
{
	if (!busLineSections.empty()) {
		BusStopMapConstIterator stopIter;
		double fromStopId;
		double toStopId;

		BusLineSectionMapConstIterator sectionIter = busLineSections.begin();
		while (sectionIter != busLineSections.end()) {
			fromStopId = sectionIter->second->getFromBusStopId();

			stopIter = busStops.find(fromStopId);
			if (stopIter == busStops.end()) 
				throw CityException(__func__,"not found bus stop id");

			stops.push_back(stopIter->second.first);
			sectionIter++;
		}

		toStopId = busLineSections.rbegin()->second->getToBusStopId();

		stopIter = busStops.find(toStopId);
		if (stopIter == busStops.end()) 
			throw CityException(__func__,"not found bus stop id");

		stops.push_back(stopIter->second.first);
	}
}

bool CityBusLine::hasBusStop(double stopId) const
{
	BusStopMapConstIterator iter = busStops.find(stopId);
	if (iter == busStops.end())
		return false;

	return true;
}

void CityBusLine::addBusStop(CityBusStop* stop, int sectionNum)
{
	double stopId = stop->getBusStopId();

	std::pair<BusStopMapIterator, BusStopMapIterator> range = busStops.equal_range(stopId);

	BusStopMapIterator iter = range.first;
	BusStopMapIterator iterEnd = range.second;

	while (iter != iterEnd) {
		if (iter->second.second == sectionNum)
			return;
		iter++;
	}

	busStops.emplace(stopId, std::make_pair(stop, sectionNum));
}

CityBusLineSection* CityBusLine::getBusLineSection(double fromStopId) const
{
	auto range = busStops.equal_range(fromStopId);
	int order = 0;
	for (auto iter = range.first; iter != range.second; iter++) {
		// choose smaller order
		if (order == 0 || order > iter->second.second)
			order = iter->second.second;
	}		

	BusLineSectionMapConstIterator sectionIter = busLineSections.find(order);
	if (sectionIter == busLineSections.end())
		return nullptr;

	return sectionIter->second;
}

double CityBusLine::getBusLineSections(std::vector<CityBusLineSection*>& sections) const
{
	double totalLength = 0; 
	BusLineSectionMapConstIterator sectionIter = busLineSections.begin(); 
	while (sectionIter != busLineSections.end()) { 
		totalLength += sectionIter->second->getLength(); 
		
		sections.push_back(sectionIter->second); 
		sectionIter++; 
	} 
	
	return totalLength;
}

double CityBusLine::getBusLineSections(double fromStopId, double toStopId, std::vector<CityBusLineSection*>& sections) const
{
	BusStopMapConstIterator stopIter;
	stopIter = busStops.find(fromStopId);
	if (stopIter == busStops.end())
		return 0;

	// choose smaller order
	auto fromRange = busStops.equal_range(fromStopId);
	int fromOrder = 0;
	for (auto iter = fromRange.first; iter != fromRange.second; iter++) {
		if (fromOrder == 0 || fromOrder > iter->second.second)
			fromOrder = iter->second.second;
	}

	stopIter = busStops.find(toStopId);
	if (stopIter == busStops.end())
		return 0;

	// circular line
	if (fromStopId == toStopId)
		return getBusLineSections(sections);

	auto toRange = busStops.equal_range(toStopId);
	int toOrder = 0;
	for (auto iter = toRange.first; iter != toRange.second; iter++) {
		// choose larger order than fromOrder
		if (toOrder == 0 || iter->second.second > fromOrder)
			toOrder = iter->second.second;
	}

	double totalLength = 0; 

	if (fromOrder < toOrder) {
		BusLineSectionMapConstIterator sectionIter = busLineSections.find(fromOrder);
		for (int i = fromOrder; i < toOrder; i++) {
			totalLength += sectionIter->second->getLength(); 
		
			sections.push_back(sectionIter->second);
			sectionIter++;
		}
	}

	return totalLength;
}

double CityBusLine::getBusLineSections(int fromOrder, int toOrder, std::vector<CityBusLineSection*>& sections) const
{
	double totalLength = 0;
	BusLineSectionMapConstIterator sectionIter = busLineSections.find(fromOrder);
	for (int i = fromOrder; i < toOrder; i++) {
		totalLength += sectionIter->second->getLength();

		sections.push_back(sectionIter->second);
		sectionIter++;
	}

	return totalLength;
}

void CityBusLine::addBusLineSection(double fromStopId, double toStopId, int sectionNum, CityGeometry* geom)
{
	if ((sectionNum - 1) != busLineSections.size())
		throw CityException(__func__,"not continuous bus stop id in this bus line");

	CityBusLineSection* section;

	if (!busLineSections.empty()) {
		section = busLineSections.rbegin()->second;
		if (section->getToBusStopId() != fromStopId)
			throw CityException(__func__,"not continuous bus stop id in this bus line");
	}

	section = new CityBusLineSection(fromStopId, toStopId, geom);
	busLineSections[sectionNum] = section;
}

void CityBusLine::removeBusLineSection(double fromStopId)
{
	BusStopMapConstIterator stopIter = busStops.find(fromStopId);
	if (stopIter == busStops.end())
		return;

	BusLineSectionMapConstIterator sectionIter = busLineSections.find(stopIter->second.second);
	if (sectionIter != busLineSections.end()) {
		CityBusLineSection* section = sectionIter->second;
		sectionIter = busLineSections.erase(sectionIter);
		delete section;
	}
}

// add transfer bus lines and stops
void CityBusLine::addXferBusLine(double xferLineId, CityBusStop* board, CityBusStop* xfer)
{
	XferBusLineMapIterator iter = xferBusLines.find(xferLineId);
	double boardStopId = board->getBusStopId();
	double xferStopId = xfer->getBusStopId();
	std::vector<std::pair<double, double>> adjStops;

	if (iter != xferBusLines.end()) {
		adjStops = iter->second;
		for (int i = 0; i < adjStops.size(); i++) {
			if (adjStops[i].first == boardStopId && adjStops[i].second == xferStopId)
				return;
		}

		iter->second.push_back(std::make_pair(boardStopId, xferStopId));
	} else {
		adjStops.push_back(std::make_pair(boardStopId, xferStopId));
		xferBusLines[xferLineId] = adjStops;
	}
}

void CityBusLine::getXferBusLines(std::vector<double>& out)
{
	XferBusLineMapIterator iter = xferBusLines.begin();
	while (iter != xferBusLines.end()) {
		double xferLineId = iter->first;
		out.push_back(xferLineId);
		iter++;
	}
}

void CityBusLine::getXferBusLines(std::vector<double> filters, std::vector<double>& out)
{
	XferBusLineMapIterator iter = xferBusLines.begin();
	while (iter != xferBusLines.end()) {
		double xferLineId = iter->first;
		
		int i = 0;
		while (i < filters.size()) {
			if (xferLineId == filters[i])
				break;
		}
		if (i < filters.size())
			continue;

		out.push_back(xferLineId);
		iter++;
	}
}

void CityBusLine::getXferStops(double xferLineId, double stopId, std::multimap<int, std::pair<double, double>>& out)
{
	XferBusLineMapIterator iter = xferBusLines.find(xferLineId); 
	if (iter == xferBusLines.end()) 
		return; 
		
	CityBusStop* startOnStop = getBusStop(stopId); 
	std::pair<int, int> startOnOrder = startOnStop->getBusStopOrder(busLineId); 
	if (startOnOrder.first < 0) 
		return; 
		
	std::vector<std::pair<double, double>> adjStops = iter->second; 
	
	for (int i = 0; i < adjStops.size(); i++) { 
		CityBusStop* startOffStop = getBusStop(adjStops[i].first); 
		std::pair<int, int> startOffOrder = startOffStop->getBusStopOrder(busLineId); 
		
		if (startOffOrder.first < 0 || startOnOrder.first > startOffOrder.first) 
			continue; 
			
		out.insert(std::make_pair(startOffOrder.first, std::make_pair(adjStops[i].first, adjStops[i].second)));
	} 
}

} /* namespace dtsim */
