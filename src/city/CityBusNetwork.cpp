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
 * CityBusNetwork.cpp
 *
 * $Revision$
 * $LastChangedDate$
 */

#include "CityGeographySpace.h"
#include "CityCoordinate.h"
#include "CityBusNetwork.h"
#include "CityBusLine.h"

namespace dtsim {

#define MAX_TRANSFER_LIMIT		2		/* max. numbers of transfers */
#define MAX_DIRECT_LIMIT		3		/* max. numbers of direct lines */
#define MAX_INDIRECT_LIMIT		3		/* max. mumbers of indirect lines */
#define MAX_DISTANCE_RATE		4		/* max. direct line distance */

#define NEIGHBOR_RANGE			200.0
#define ROUTE_RADIUS_LIMIT		500.0
#define ROUTE_RADIUS_BASIC		200.0
#define ROUTE_RADIUS_SCALE		100.0

#undef DEBUG
#undef UNLIMITED

CityBusNetwork::CityBusNetwork(std::string name, CityContext* context)
: CitySpace(name, context)
{}

CityBusNetwork::~CityBusNetwork()
{
	BusLineMapConstIterator iter = busLines.begin();
	while (iter != busLines.end()) {
		CityBusLine* busLine = iter->second;
		iter = busLines.erase(iter);
		delete busLine;
	}

	busStops.clear();
}

void CityBusNetwork::getBusLines(std::vector<CityBusLine*>& lines) const
{
	BusLineMapConstIterator iter = busLines.begin();
	while (iter != busLines.end()) {
		lines.push_back(iter->second);
		iter++;
	}
}

CityBusLine* CityBusNetwork::getBusLine(double busLineId) const
{
	BusLineMapConstIterator iter = busLines.find(busLineId);
	if (iter != busLines.end())
		return iter->second;

	return nullptr;
}

void CityBusNetwork::addBusLine(CityBusLine* busLine)
{
	BusLineMapIterator iter = busLines.find(busLine->getBusLineId());
	if (iter == busLines.end())
		busLines[busLine->getBusLineId()] = busLine;
}

void CityBusNetwork::removeBusLine(double busLineId)
{
	BusLineMapIterator iter = busLines.find(busLineId);
	if (iter != busLines.end()) {
		CityBusLine* busLine = iter->second;
		iter = busLines.erase(iter);
		delete busLine;
	}
}

void CityBusNetwork::addBusStop(CityBusStop* busStop)
{
	BusStopMapIterator iter = busStops.find(busStop->getBusStopId());
	if (iter == busStops.end())
		busStops[busStop->getBusStopId()] = busStop;
}

void CityBusNetwork::removeBusStop(double stopId)
{
	BusStopMapIterator iter = busStops.find(stopId);
	if (iter != busStops.end())
		busStops.erase(iter);
}

CityBusStop* CityBusNetwork::getBusStop(double stopId)
{
	BusStopMapIterator iter = busStops.find(stopId);
	if (iter != busStops.end())
		return iter->second;

	return nullptr;
}

/* Builds transfer bus line map */
void CityBusNetwork::buildTransferLines()
{
	CityGeographySpace* envSpace = City::instance()->getGeographySpace();

	std::vector<CityBusStop*> adjStops;
	std::vector<CityBusLine*> adjLines;
	std::vector<CityBusLine*> xferLines;

	BusStopMapIterator busStopsIter = busStops.begin();

	std::cout.precision(std::numeric_limits<double>::max_digits10);
	while (busStopsIter != busStops.end()) {
		CityBusStop* busStop = busStopsIter->second;

		busStop->getBusLines(xferLines);

		/* registers lines passing specified bus stop to each other's transfer lines */
		for (CityBusLine* X1 : xferLines)
			for (CityBusLine* X2 : xferLines) {
				if (X1->getBusLineId() == X2->getBusLineId()) 
					continue;

				X1->addXferBusLine(X2->getBusLineId(), busStop, busStop);
			}

		/* finds bus stops within 200m from this stop */
		CityCoordinate coord;
		busStop->getCoordinate(coord);
		envSpace->getAgentsWithin(coord.x, coord.y, NEIGHBOR_RANGE, adjStops);

		/* registers lines passing adjacent bus stops to this stop's transfer lines */
		for (CityBusStop* adjStop : adjStops) {
			if (busStop->getBusStopId() == adjStop->getBusStopId())
				continue;

			adjStop->getBusLines(adjLines);

			for (CityBusLine* X : xferLines)
				for (CityBusLine* A : adjLines) {
					if (X->getBusLineId() == A->getBusLineId())
						continue;
					X->addXferBusLine(A->getBusLineId(), busStop, adjStop);
				}

			adjLines.clear();
		}
		adjStops.clear();
		xferLines.clear();

		busStopsIter++;
	}

//	printTransferMap();
}

void CityBusNetwork::printTransferMap()
{
	std::cout << "[[[[ Transfer Line Map ]]]] " << std::endl;

	BusLineMapIterator busLineIter = busLines.begin();
	while (busLineIter != busLines.end()) {
		CityBusLine* busLine = busLineIter->second;

		typename std::unordered_map<double, std::vector<std::pair<double, double>>>::const_iterator xIter; 
		xIter = busLine->xferBusLine_begin();

		while (xIter != busLine->xferBusLine_end()) {
			double xferLineId = xIter->first;
			std::vector<std::pair<double, double>> xferStops = xIter->second;
				
			CityBusLine* xferLine = getBusLine(xferLineId);

			std::cout << "# LINE[" << busLine->getBusLineId() << "->" << xferLineId << "]";
			std::cout << " (" << busLine->getBusLineName() << "->" << xferLine->getBusLineName();
			std::cout << ") stops=" << xferStops.size() << std::endl; 

			for (int i = 0; i < xferStops.size(); i++) {
				double startStopId = xferStops[i].first;
				double xferStopId = xferStops[i].second;

				CityBusStop* startStop = getBusStop(startStopId);
				CityBusStop* xferStop = getBusStop(xferStopId);

				std::cout << "  [" << i << "] STOP[" << startStopId << "] " << startStop->getBusStopName();
				std::cout << "-> XFER[" << xferStopId << "] " << xferStop->getBusStopName() << std::endl; 
			}

			xferStops.clear();
			xIter++;
		}
		busLineIter++;
	}
}

/* Checks if specified bus line includes in direct lines */
bool CityBusNetwork::findLine(double lineId, std::vector<double>& lineIds)
{
	for (double busLineId : lineIds)
		if (lineId == busLineId)
			return true;

	return false;
}

bool CityBusNetwork::findLine(double lineId, std::vector<CityBusLine*>& busLines)
{
	for (CityBusLine* busLine : busLines)
		if (lineId == busLine->getBusLineId())
			return true;

	return false;
}

/* Checks if specified bus route has been already found */
bool CityBusNetwork::IsDirectRoute(double lineId, std::map<double, BusRouteInfo>& routeMap)
{
	typename std::map<double, BusRouteInfo>::iterator rIter;
	for (rIter = routeMap.begin(); rIter != routeMap.end(); rIter++) {
		BusRouteInfo S = rIter->second;
		if (S.busRoute.size() > 1) 
			continue;
		if (S.busRoute[0].first == lineId)
			return true;
	}

	return false;
}

bool CityBusNetwork::canStop(std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds)
{
	if (routeMap.size() == 0)
		return false;

	if (directLineIds.size() >= MAX_DIRECT_LIMIT) {
		if (routeMap.size() > MAX_DIRECT_LIMIT) {
			typename std::map<double, BusRouteInfo>::iterator rIter;
			if (routeMap.size() > MAX_DIRECT_LIMIT) {
				int i = 0;
				for (i = 0, rIter = routeMap.begin(); i < MAX_DIRECT_LIMIT; i++, rIter++);
				routeMap.erase(rIter, routeMap.end());
			}
		}
		return true;
	}

	if (directLineIds.size()) {
		if (routeMap.size()-directLineIds.size() >= MAX_INDIRECT_LIMIT) {
			typename std::map<double, BusRouteInfo>::iterator rIter;
			if (routeMap.size()-directLineIds.size() >= MAX_INDIRECT_LIMIT) {
				int i = 0;
				for (i = 0, rIter = routeMap.begin(); i < MAX_INDIRECT_LIMIT; i++, rIter++);
				routeMap.erase(rIter, routeMap.end());
			}
			return true;
		}
	} else {
		if (routeMap.size() >= MAX_INDIRECT_LIMIT) {
			typename std::map<double, BusRouteInfo>::iterator rIter;
			if (routeMap.size() >= MAX_INDIRECT_LIMIT) {
				int i = 0;
				for (i = 0, rIter = routeMap.begin(); i < MAX_INDIRECT_LIMIT; i++, rIter++);
				routeMap.erase(rIter, routeMap.end());
			}
			return true;
		}
	}

	return false;
}

/* Checks if from bus stop can travel to bus stop */
bool CityBusNetwork::canTransfer(std::pair<int, int> startOrder, std::pair<int, int> goalOrder, int& fromOrder, int& toOrder)
{
	if (startOrder.first < 0 || goalOrder.first < 0)
		return false;

	if (startOrder.first == startOrder.second) {
		if (goalOrder.first == goalOrder.second) {
			if (startOrder.first > goalOrder.first)
				return false;
			fromOrder = startOrder.first;
			toOrder = goalOrder.first;
			goto out;
		} else { 	
			/* goalOrder.first != goalOrder.second */
			if (startOrder.first > goalOrder.first) {
				if (startOrder.first > goalOrder.second)
					return false;
				fromOrder = startOrder.first;
				toOrder = goalOrder.second;
				goto out;
			} else {
				fromOrder = startOrder.first;
				toOrder = goalOrder.first;
				goto out;
			}
		}
	} else {	
		/* startOrder.first != startOrder.second */
		if (goalOrder.first == goalOrder.second) {
			if (startOrder.first > goalOrder.first)
				if (startOrder.second > goalOrder.first)
					return false;
				fromOrder = startOrder.second;
				toOrder = goalOrder.first;
				goto out;
		} else {
			/* goalOrder.first != goalOrder.second */
			if (startOrder.first > goalOrder.first) {
				if (startOrder.first > goalOrder.second) {
					if (startOrder.second > goalOrder.first) {
						if (startOrder.second > goalOrder.second)
							return false;
						fromOrder = startOrder.second;
						toOrder = goalOrder.second;
						goto out;
					} else {
						fromOrder = startOrder.second;
						toOrder = goalOrder.first;
						goto out;
					}
				} else {
					fromOrder = startOrder.first;
					toOrder = goalOrder.second;
					goto out;
				}
			} else {
				fromOrder = startOrder.first;
				toOrder = goalOrder.first;
				goto out;
			}
		}
	}

out:
	return true;
}

/* Returnes total direct bus lines consisting of bus line id and bus line sections */
void CityBusNetwork::getDirectLines(double startId, double goalId, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance)
{
	CityBusStop* startStop = getBusStop(startId);
	CityBusStop* goalStop = getBusStop(goalId);

	std::vector<CityBusLine*> startLines, goalLines;

	startStop->getBusLines(startLines);
	goalStop->getBusLines(goalLines);

	for (CityBusLine* startLine : startLines) {
		double startLineId = startLine->getBusLineId();

		for (CityBusLine* goalLine : goalLines) {
			double goalLineId = goalLine->getBusLineId();

			if (startLineId != goalLineId)
				continue;

			/* start line equals to goal line */
			std::pair<int, int> startOrder = startStop->getBusStopOrder(startLineId);
			std::pair<int, int> goalOrder = goalStop->getBusStopOrder(goalLineId);
			int fromOrder = 0, toOrder = 0;

			if (canTransfer(startOrder, goalOrder, fromOrder, toOrder) == false)
				continue;

			std::vector<CityBusLineSection*> sections;

			/* from startId to goalId */
			double length = startLine->getBusLineSections(fromOrder, toOrder, sections);

			if (sections.size() == 0)
				continue;

			if (length > maxDistance) {
				directLineIds.push_back(startLineId);
				sections.clear();
				continue;
			}

			/* filter out the same direct line */
			if (IsDirectRoute(startLineId, routeMap))
				continue;

			BusRouteInfo singleRoute;
			singleRoute.busRoute.push_back(std::make_pair(startLineId, sections));

			routeMap[length] = singleRoute;
			directLineIds.push_back(startLineId);

#ifndef UNLIMITED
			if (canStop(routeMap, directLineIds))
				return;
#endif

			sections.clear();
		}
	}
}

/* gets two or more transfers */
double CityBusNetwork::getRideStops(double curStartId, double goalId, CityBusLine* curStartLine, std::vector<double>& xferLine, std::vector<std::pair<double, double>>& routeStops, int xferLineIdx, std::vector<double>& directLineIds)
{
	int xLineIdx = xferLineIdx;
	double curStartLineId = curStartLine->getBusLineId();
	double adjLineId = xferLine[xLineIdx]; 

	if (findLine(adjLineId, directLineIds))
		return 0;

	CityBusLine* adjLine = getBusLine(adjLineId);

	/* sorts by bus stop order on bus line */
	std::multimap<int, std::pair<double, double>> adjStops;
	typename std::multimap<int, std::pair<double, double>>::iterator aIter;

	curStartLine->getXferStops(adjLineId, curStartId, adjStops);

	for (aIter = adjStops.begin(); aIter != adjStops.end(); aIter++) {
		double curOffStopId = (*aIter).second.first;
		double adjOnStopId = (*aIter).second.second;

		CityBusStop* onStop = getBusStop(curStartId);
		CityBusStop* offStop = getBusStop(curOffStopId);

		std::pair<int, int> startOrder = onStop->getBusStopOrder(curStartLineId);
		std::pair<int, int> goalOrder = offStop->getBusStopOrder(curStartLineId);
		int fromOrder, toOrder;

		if (canTransfer(startOrder, goalOrder, fromOrder, toOrder) == false)
			continue;

		/* this is a goal line */
		if (xLineIdx == xferLine.size() - 1) {
			onStop = getBusStop(adjOnStopId);
			offStop = getBusStop(goalId);

			startOrder = onStop->getBusStopOrder(adjLineId);
			goalOrder = offStop->getBusStopOrder(adjLineId);

			if (canTransfer(startOrder, goalOrder, fromOrder, toOrder) == false)
				continue;
		}

		routeStops.push_back(std::make_pair(curStartId, curOffStopId));

		CityBusLine* adjLine = getBusLine(adjLineId);

		if (xLineIdx == xferLine.size() - 1) {
			routeStops.push_back(std::make_pair(adjOnStopId, goalId));
			return goalId;
		}
		
		/* erases the last curStartId and curOffStopId */
		double lastStopId = getRideStops(adjOnStopId, goalId, adjLine, xferLine, routeStops, ++xLineIdx, directLineIds);

		if (lastStopId == 0)
			routeStops.erase(routeStops.end() - 1);
		else if (lastStopId == goalId)
			return goalId;
	}

	return 0;
}

void CityBusNetwork::recurTransferRoutes(double startId, double goalId, std::vector<double> xferLine, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance)
{
	int xferLineIdx = 0;
	std::vector<std::pair<double, double>> routeStops;
	CityBusLine* startLine = getBusLine(xferLine[xferLineIdx++]); // [0]: the first line
	double adjLineId = xferLine[xferLineIdx++]; // [1]: the second line

	if (findLine(adjLineId, directLineIds))
		return;

	CityBusLine* adjLine = getBusLine(adjLineId);

	std::multimap<int, std::pair<double, double>> adjStops;
	typename std::multimap<int, std::pair<double, double>>::iterator aIter;

	startLine->getXferStops(adjLineId, startId, adjStops);

	for (aIter = adjStops.begin(); aIter != adjStops.end(); aIter++) {
		double startOffStopId = (*aIter).second.first;
		double adjOnStopId = (*aIter).second.second;

		routeStops.push_back(std::make_pair(startId, startOffStopId));

		/* completes to find all on and off stops on xfer line */
		double lastStopId = getRideStops(adjOnStopId, goalId, adjLine, xferLine, routeStops, xferLineIdx, directLineIds);
		if (lastStopId == goalId)
			break;

		routeStops.clear();
	}

	if (routeStops.size() == 0)
		return;

	/* make routes */
	BusRouteInfo singleRoute;
	std::vector<CityBusLineSection*> sections;
	double length = 0;

	for (int i = 0; i < xferLine.size(); i++) {
		double onStopId = routeStops[i].first;
		double offStopId = routeStops[i].second;

		CityBusStop* onStop = getBusStop(onStopId);
		CityBusStop* offStop = getBusStop(offStopId);

		std::pair<int, int> startOrder = onStop->getBusStopOrder(xferLine[i]);
		std::pair<int, int> goalOrder = offStop->getBusStopOrder(xferLine[i]);
		int fromOrder, toOrder;

		if (canTransfer(startOrder, goalOrder, fromOrder, toOrder) == false)
			continue;

		/* from onStopId to offStopId */
		CityBusLine* xline = getBusLine(xferLine[i]);
		length += xline->getBusLineSections(fromOrder, toOrder, sections);

		if (sections.size() == 0)
			return;

		singleRoute.busRoute.push_back(std::make_pair(xferLine[i], sections));
		sections.clear();
	}

	if (length > maxDistance)
		return;

	if (singleRoute.busRoute.size() == 1) {
		if (IsDirectRoute(singleRoute.busRoute[0].first, routeMap))
			return;
		routeMap[length] = singleRoute;
		directLineIds.push_back(singleRoute.busRoute[0].first);
	}
	else {
		routeMap[length] = singleRoute;
	}
}

void CityBusNetwork::getTransferRoutes(double startId, double goalId, std::vector<std::vector<double>> xferRoutes, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance)
{
	for (int i = 0; i < xferRoutes.size(); i++) {
		std::vector<double> xferLine = xferRoutes[i];

		/* makes a route from start bus line to goal bus line */
		if (xferLine.size() < 3) {
			double startLineId = xferLine[0];
			double goalLineId = xferLine[1];
			CityBusLine* startLine = getBusLine(startLineId);
			CityBusLine* goalLine = getBusLine(goalLineId);

			std::multimap<int, std::pair<double, double>> adjStops;
			typename std::multimap<int, std::pair<double, double>>::iterator aIter;

			startLine->getXferStops(goalLineId, startId, adjStops);

			for (aIter = adjStops.begin(); aIter != adjStops.end(); aIter++) {
				double curOffStopId = (*aIter).second.first;
				double adjOnStopId = (*aIter).second.second;

				CityBusStop* startOnStop = getBusStop(startId);
				CityBusStop* startOffStop = getBusStop(curOffStopId);

				std::pair<int, int> startOrder = startOnStop->getBusStopOrder(startLineId);
				std::pair<int, int> goalOrder = startOffStop->getBusStopOrder(startLineId);
				int startFromOrder, startToOrder;

				if (canTransfer(startOrder, goalOrder, startFromOrder, startToOrder) == false)
					continue;

				CityBusStop* onStop = getBusStop(adjOnStopId);
				CityBusStop* offStop = getBusStop(goalId);

				startOrder = onStop->getBusStopOrder(goalLineId);
				goalOrder = offStop->getBusStopOrder(goalLineId);
				int goalFromOrder, goalToOrder;

				if (canTransfer(startOrder, goalOrder, goalFromOrder, goalToOrder) == false)
					continue;

				BusRouteInfo singleRoute;
				std::vector<CityBusLineSection*> startSections, goalSections;

				/* from startId to curOffStopId */
				double length = startLine->getBusLineSections(startFromOrder, startToOrder, startSections);

				/* from adjOnStopId to goalId */
				length += goalLine->getBusLineSections(goalFromOrder, goalToOrder, goalSections);

				if (startSections.size())
					singleRoute.busRoute.push_back(std::make_pair(startLineId, startSections));

				if (goalSections.size())
					singleRoute.busRoute.push_back(std::make_pair(goalLineId, goalSections));

				if (startSections.size() == 0 && goalSections.size() == 0)
					continue;

				if (length > maxDistance)
					continue;

				if (singleRoute.busRoute.size() == 1) {
					if (IsDirectRoute(singleRoute.busRoute[0].first, routeMap))
						break;
					routeMap[length] = singleRoute;
					directLineIds.push_back(singleRoute.busRoute[0].first);
				} else {
					routeMap[length] = singleRoute;
				}

				break;
			}
		} else { 
			/* two or more transfers */
			recurTransferRoutes(startId, goalId, xferRoutes[i], routeMap, directLineIds, maxDistance);
		}

		xferLine.clear();

#ifndef UNLIMITED
		if (canStop(routeMap, directLineIds))
			break;
#endif
	}
}

void CityBusNetwork::recurTransferLines(std::vector<CityBusLine*> startLines, CityBusLine* startLine, std::vector<CityBusLine*> goalLines, std::vector<double>& xferLine, std::vector<std::vector<double>>& xferRoutes)
{
	std::vector<double> backupLine = xferLine;
	std::vector<double> adjLineIds;

	startLine->getXferBusLines(adjLineIds);

	for (double adjLineId : adjLineIds) {
		if (adjLineId == startLine->getBusLineId())
			continue;

		for (CityBusLine* goalLine : goalLines) {
			if (adjLineId == goalLine->getBusLineId()) {
				if (xferLine.size() == 0)
					xferLine = backupLine;

				xferLine.push_back(goalLine->getBusLineId());
				xferRoutes.push_back(std::move(xferLine));
				xferLine.clear();
				return;
			}
		}

		if (xferLine.size() == 0)
			xferLine = backupLine;

		if (findLine(adjLineId, xferLine) || findLine(adjLineId, startLines))
			continue;

		xferLine.push_back(adjLineId);

		if (xferLine.size() > MAX_TRANSFER_LIMIT) {
			xferLine.clear();
			continue;
		}

		/* recursive call to complete xferLine from start to goal lines */
		CityBusLine* adjLine = getBusLine(adjLineId);

		recurTransferLines(startLines, adjLine, goalLines, xferLine, xferRoutes);
	}
}

void CityBusNetwork::getTransferLines(int startLineIdx, std::vector<CityBusLine*> startLines, std::vector<CityBusLine*> goalLines, std::vector<std::vector<double>>& xferRoutes)
{
	std::vector<double> xferLine;
	std::vector<double> adjLineIds;
	CityBusLine* startLine = startLines[startLineIdx];

	startLine->getXferBusLines(adjLineIds);

	for (double adjLineId : adjLineIds) {
		bool found = false;
		if (adjLineId == startLine->getBusLineId())
			continue;

	    for (CityBusLine* goalLine : goalLines) {
			if (adjLineId == goalLine->getBusLineId()) {
				CityBusLine* line = getBusLine(adjLineId);

				if (xferLine.size() == 0)
					xferLine.push_back(startLine->getBusLineId());

				xferLine.push_back(goalLine->getBusLineId());
				xferRoutes.push_back(xferLine);
				xferLine.clear();
				found = true;
				break;
			}
		}
		if (found)
			continue;

		if (findLine(adjLineId, startLines))
			continue;
			
		if (xferLine.size() == 0)
			xferLine.push_back(startLine->getBusLineId()); 

		xferLine.push_back(adjLineId);

		if (xferLine.size() > MAX_TRANSFER_LIMIT) {
			xferLine.clear();
			continue;
		}

		CityBusLine* adjLine = getBusLine(adjLineId);
		recurTransferLines(startLines, adjLine, goalLines, xferLine, xferRoutes);

		/* if the end line is not the same as the goal line, search fails */
		if (xferLine.size())
			xferLine.clear();
	}
}

bool CityBusNetwork::findDirects(std::vector<CityBusStop*> startStops, std::vector<CityBusStop*> goalStops, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance)
{
	for (CityBusStop* startStop : startStops) {
		double startStopId = startStop->getBusStopId();

		for (CityBusStop* goalStop : goalStops) {
			double goalStopId = goalStop->getBusStopId();

			if (startStopId == goalStopId)
				continue;

			getDirectLines(startStopId, goalStopId, routeMap, directLineIds, maxDistance);

#ifndef UNLIMITED
			if (canStop(routeMap, directLineIds))
				return true;
#endif
		}
	}

	return false;
}

bool CityBusNetwork::findTransfers(std::vector<CityBusStop*> startStops, std::vector<CityBusStop*> goalStops, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance)
{
	for (CityBusStop* startStop : startStops) {
		double startStopId = startStop->getBusStopId();

		for (CityBusStop* goalStop : goalStops) {
			double goalStopId = goalStop->getBusStopId();

			if (startStopId == goalStopId)
				continue;

			std::vector<CityBusLine*> startLines, goalLines;

			startStop->getBusLines(startLines);
			goalStop->getBusLines(startLines, goalLines);

			std::vector<std::vector<double>> xferRoutes;

			for (int i = 0; i < startLines.size(); i++) {
				if (findLine(startLines[i]->getBusLineId(), directLineIds))
					continue;

				getTransferLines(i, startLines, goalLines, xferRoutes);
			}

			getTransferRoutes(startStopId, goalStopId, xferRoutes, routeMap, directLineIds, maxDistance);

#ifndef UNLIMITED
			if (canStop(routeMap, directLineIds))
				return true;
#endif

			xferRoutes.clear();
			startLines.clear();
			goalLines.clear();
		}
	}

	return false;
}

bool CityBusNetwork::findTransfers(std::vector<CityBusStop*> startStops, std::vector<CityBusStop*> goalStops, int max_routes, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance)
{
	for (CityBusStop* startStop : startStops) {
		double startStopId = startStop->getBusStopId();

		for (CityBusStop* goalStop : goalStops) {
			double goalStopId = goalStop->getBusStopId();

			if (startStopId == goalStopId)
				continue;

			std::vector<CityBusLine*> startLines, goalLines;

			startStop->getBusLines(startLines);
			goalStop->getBusLines(startLines, goalLines);

			std::vector<std::vector<double>> xferRoutes;

			for (int i = 0; i < startLines.size(); i++) {
				if (findLine(startLines[i]->getBusLineId(), directLineIds))
					continue;

				getTransferLines(i, startLines, goalLines, xferRoutes);
			}

			getTransferRoutes(startStopId, goalStopId, xferRoutes, routeMap, directLineIds, maxDistance);

			if (routeMap.size() >= max_routes)
				return true;

			xferRoutes.clear();
			startLines.clear();
			goalLines.clear();
		}
	}

	return false;
}

void CityBusNetwork::getBusRoute(double startX, double startY, double goalX, double goalY, std::vector<BusRouteInfo>& routes, bool bShortest)
{
	double radius = ROUTE_RADIUS_BASIC;
	double distance = dtsim::CityGeodeticCalculator::instance()->getDistance(startX, startY, goalX, goalY);
	if (distance < radius) {
#ifdef DEBUG
		std::cout << ">>> Pedestrian route" << std::endl;
#endif
		return;
	}

	double maxDistance = distance * MAX_DISTANCE_RATE;
#ifdef DEBUG
	std::cout << "  Distance (" << distance << ") MaxDist (" << maxDistance << ")" << std::endl;
#endif

	CityGeographySpace* envSpace = City::instance()->getGeographySpace();

	std::vector<CityBusStop*> startStops;
	std::vector<CityBusStop*> goalStops;

	std::map<int, std::vector<CityBusStop*>> startStopSet;
	std::map<int, std::vector<CityBusStop*>> goalStopSet;
	typename std::map<int, std::vector<CityBusStop*>>::iterator iter;

	std::vector<double> directLineIds;
	std::map<double, BusRouteInfo> routeMap;

	/* finds direct bus lines */
	int Pos = 0;
	bool bStart, bGoal, bDone;
	do {
		envSpace->getAgentsWithin(startX, startY, radius, startStops);
		if (startStops.size() == 0)
			goto next_direct;
		startStopSet[Pos] = startStops;

		envSpace->getAgentsWithin(goalX, goalY, radius, goalStops);
		if (goalStops.size() == 0)
			goto next_direct;
		goalStopSet[Pos] = goalStops;

		if (startStopSet.size() > 1) {
			bStart = bGoal = true;
			int oldPos = startStopSet.size() - 2;
			iter = startStopSet.find(oldPos);
			if (iter != startStopSet.end() && iter->second.size() == startStops.size()) {
				for (int i = 0; i < startStops.size(); i++)
					if (iter->second[i] != startStops[i]) {
						bStart = false;
						break;
					}
			} else {
				bStart = false;
			}

			iter = goalStopSet.find(oldPos);
			if (iter != goalStopSet.end() && iter->second.size() == goalStops.size()) {
				for (int i = 0; i < goalStops.size(); i++)
					if (iter->second[i] != goalStops[i]) {
						bGoal = false;
						break;
					}
			} else {
				bGoal = false;
			}

			if (bStart && bGoal)
				goto next_direct;
		}

		bDone = findDirects(startStops, goalStops, routeMap, directLineIds, maxDistance);

		if (bDone && routeMap.size())
			goto done;

		startStops.clear();
		goalStops.clear();

next_direct:
		radius += ROUTE_RADIUS_SCALE;
		Pos++;
	} while (radius <= ROUTE_RADIUS_LIMIT);

	if (startStopSet.size() == 0 || goalStopSet.size() == 0)
		return;

	if (routeMap.size())
		goto done;

	/* finds transfer bus lines */
	radius = ROUTE_RADIUS_BASIC;
	Pos = 0;
	do {
		iter = startStopSet.find(Pos);
		if (iter == startStopSet.end() || iter->second.size() == 0)
			goto next_xfer;
		startStops = iter->second;

		iter = goalStopSet.find(Pos);
		if (iter == goalStopSet.end() || iter->second.size() == 0)
			goto next_xfer;
		goalStops = iter->second;

		if (Pos > 0) {
			bStart = bGoal = true;
			iter = startStopSet.find(Pos-1);
			if (iter != startStopSet.end() && iter->second.size() == startStops.size()) {
				for (int i = 0; i < startStops.size(); i++)
					if (iter->second[i] != startStops[i]) {
						bStart = false;
						break;
					}
			} else {
				bStart = false;
			}

			iter = goalStopSet.find(Pos-1);
			if (iter != goalStopSet.end() && iter->second.size() == goalStops.size()) {
				for (int i = 0; i < goalStops.size(); i++)
					if (iter->second[i] != goalStops[i]) {
						bGoal = false;
						break;
					}
			} else {
				bStart = false;
			}

			if (bStart && bGoal)
				goto next_xfer;
		}

		bDone = findTransfers(startStops, goalStops, routeMap, directLineIds, maxDistance);

		if (bDone && routeMap.size())
			goto done;

next_xfer:
		radius += ROUTE_RADIUS_SCALE;
		Pos++;
	} while (routeMap.size() == 0 && radius <= ROUTE_RADIUS_LIMIT);

done:
	if (routeMap.size()) {
		typename std::map<double, BusRouteInfo>::iterator rIter = routeMap.begin();
		if (bShortest)
			routes.push_back(rIter->second);
		else
			for (; rIter != routeMap.end(); rIter++)
				routes.push_back(rIter->second);
	}
}

void CityBusNetwork::getBusRoute(double startX, double startY, double goalX, double goalY, double max_radius, int max_routes, std::vector<BusRouteInfo>& routes, bool bDirect, bool bShortest)
{
	if (max_radius < ROUTE_RADIUS_BASIC || max_routes <= 0) {
		std::cout << "Please check max_radius(>= " << ROUTE_RADIUS_BASIC << " m) and max_routes(> 0)" << std::endl;
		return;
	}

	double distance = dtsim::CityGeodeticCalculator::instance()->getDistance(startX, startY, goalX, goalY);
	if (distance < ROUTE_RADIUS_BASIC) {
#ifdef DEBUG
		std::cout << ">>> Pedestrian route" << std::endl;
#endif
		return;
	}

	double maxDistance = distance * MAX_DISTANCE_RATE;
#ifdef DEBUG
	std::cout << "# Distance=" << (unsigned int)distance << ", MaxDist=" << (unsigned int)maxDistance << std::endl;
#endif

	CityGeographySpace* envSpace = City::instance()->getGeographySpace();

	std::vector<CityBusStop*> startStops;
	std::vector<CityBusStop*> goalStops;

	std::vector<double> directLineIds;
	std::map<double, BusRouteInfo> routeMap;

	/* finds direct bus lines */
	bool bStart, bGoal;
	envSpace->getAgentsWithin(startX, startY, max_radius, startStops);
	if (startStops.size() == 0) {
		return;
	}

	envSpace->getAgentsWithin(goalX, goalY, max_radius, goalStops);
	if (goalStops.size() == 0) {
		return;
	}

	if (bDirect) {
		findDirects(startStops, goalStops, routeMap, directLineIds, maxDistance);
		goto finish;
	}

	/* finds transfer bus lines */
	findTransfers(startStops, goalStops, max_routes, routeMap, directLineIds, maxDistance);

finish:
	if (routeMap.size()) {
		typename std::map<double, BusRouteInfo>::iterator rIter = routeMap.begin();
		if (bShortest) {
			routes.push_back(rIter->second);
		} else {
			for (; rIter != routeMap.end(); rIter++) {
				routes.push_back(rIter->second);
				if (routes.size() >= max_routes)
					break;
			}
		}
	}
}

void CityBusNetwork::getBusRoute(CityCoordinate start, CityCoordinate goal, std::vector<BusRouteInfo>& routes, bool bShortest)
{
	getBusRoute(start.x, start.y, goal.x, goal.y, routes, bShortest);
}

void CityBusNetwork::getBusRoute(double startId, double goalId, std::vector<BusRouteInfo>& routes, bool bShortest)
{
	if (startId == goalId)
		return;

	CityBusStop* start = getBusStop(startId);
	CityBusStop* goal = getBusStop(goalId);

	if (start == nullptr || goal == nullptr)
		return;

	CityCoordinate startCoord, goalCoord;
	start->getGeometry()->getCoordinate(startCoord);
	goal->getGeometry()->getCoordinate(goalCoord);

	getBusRoute(startCoord, goalCoord, routes, bShortest);
}

/* 
 * Print functions for debugging 
 */
void CityBusNetwork::printBusRoutes(std::vector<BusRouteInfo>& routes)
{
	std::cout << "# Total Routes = " << routes.size() << std::endl;

	for (int r = 0; r < routes.size(); r++) {
		BusRouteInfo singleRoute = routes[r];
		std::cout << std::endl << "## Route-" << r << " : XferLines=" << singleRoute.busRoute.size() << std::endl;

		double length = 0;
		for (int i = 0, j = 0; j < singleRoute.busRoute.size(); j++) {
			CityBusLine* line = getBusLine(singleRoute.busRoute[j].first);
			std::vector<CityBusLineSection*> sections = singleRoute.busRoute[j].second;;

#if 0 // test
			std::vector<CityBusLineSection*> sections2;
			double totalLen = line->getBusLineSections((double)293062010, (double)293062010, sections2);
			std::cout << "[1]: 293062010 -> 293062010 : line sections, totalLen=" << (unsigned int)totalLen << std::endl;
			for (int s=0; s<sections2.size(); s++) {
				std::cout << "  - from=" << (unsigned int)sections2[s]->getFromBusStopId();
				std::cout << ", to=" << (unsigned int)sections2[s]->getToBusStopId();
				std::cout << ", len=" << (unsigned int)sections2[s]->getLength() << std::endl;
			}
			sections2.clear();

			totalLen = line->getBusLineSections((double)293062010, (double)293064119, sections2);
			std::cout << "[1]: 293062010 -> 293064119 : line sections, totalLen=" << (unsigned int)totalLen << std::endl;
			for (int s=0; s<sections2.size(); s++) {
				std::cout << "  - from=" << (unsigned int)sections2[s]->getFromBusStopId();
				std::cout << ", to=" << (unsigned int)sections2[s]->getToBusStopId();
				std::cout << ", len=" << (unsigned int)sections2[s]->getLength() << std::endl;
			}
			sections2.clear();

			totalLen = line->getBusLineSections((double)293064119, (double)293062010, sections2);
			std::cout << "[1]: 293064119 -> 293062010 : line sections, totalLen=" << (unsigned int)totalLen << std::endl;
			for (int s=0; s<sections2.size(); s++) {
				std::cout << "  - from=" << (unsigned int)sections2[s]->getFromBusStopId();
				std::cout << ", to=" << (unsigned int)sections2[s]->getToBusStopId();
				std::cout << ", len=" << (unsigned int)sections2[s]->getLength() << std::endl;
			}
			sections2.clear();
#endif // test

			if (sections.size() == 0) {
				std::cout << "   X.Line = " <<  line->getBusLineName() << "[" << singleRoute.busRoute[j].first << "]" << std::endl;
				break;
			}
			std::cout << "[" << j << "] Line=" <<  line->getBusLineName() << "[" << singleRoute.busRoute[j].first << "] " << std::endl;

			double preStopId = 0;
			for (CityBusLineSection* S : sections) {
				double fromStopId = S->getFromBusStopId();
				double toStopId = S->getToBusStopId();
				CityBusStop* fromStop = getBusStop(fromStopId);
				CityBusStop* toStop = getBusStop(toStopId);

				CityCoordinate coord;

				if (preStopId == 0 || preStopId != fromStopId) {
					std::cout << "  Stop " << i++ << ". " << fromStop->getBusStopName() << "[" << fromStopId << "]" << std::endl;
					std::cout << "  Stop " << i++ << ". " << toStop->getBusStopName() << "[" << toStopId << "]" << std::endl;
				} else if (preStopId == fromStopId) {
					std::cout << "  Stop " << i++ << ". " << toStop->getBusStopName() << "[" << toStopId << "]" << std::endl;
				}

				preStopId = toStopId;
				length += S->getLength();
			}
		}
		std::cout << "-------- Length = " << (unsigned long)length << " meter" << std::endl;
	}
}

void CityBusNetwork::printBusStops() 
{
	std::cout.precision(std::numeric_limits<double>::max_digits10); 
	BusStopMapIterator bIter = busStops.begin(); 
	while (bIter != busStops.end()) { 
		CityBusStop* stop = bIter->second; 
		std::cout << "Stop[" << stop->getBusStopId() << "] lines=" << stop->getBusLinesCount() << std::endl; 
		bIter++; 
	}
}

void CityBusNetwork::printBusLines() 
{
	std::cout.precision(std::numeric_limits<double>::max_digits10); 
	BusLineMapIterator bIter = busLines.begin(); 
	while (bIter != busLines.end()) { 
		CityBusLine* line = bIter->second; 
		std::cout << "# BusLine[" << line->getBusLineId() << "] " << line->getBusLineName() << std::endl; 

		std::vector<CityBusLineSection*> sections;
		line->getBusLineSections(sections);
		CityBusStop *from, *to;
		for (int i = 0; i < sections.size(); i++) {
			if (!i) {
				from = getBusStop(sections[i]->getFromBusStopId());
				std::cout << "[" << i+1 << "] BusStop[" << from->getBusStopId();
				std::cout << "-" << from->getBusStopName() << "] lines=" << from->getBusLinesCount() << std::endl;
			}
			to = getBusStop(sections[i]->getToBusStopId());
			std::cout << "[" << i+2 << "] BusStop[" << to->getBusStopId();
			std::cout << "-" << to->getBusStopName() << "] lines=" << to->getBusLinesCount() << std::endl;
		}
		sections.clear();
		bIter++; 
	}
}

} /* namespace dtsim */
