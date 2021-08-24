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
 * CityBusNetwork.h
 *
 * $Revision$
 * $LastChangedDate$
 */

#ifndef CITY_BUS_NETWORK_H
#define CITY_BUS_NETWORK_H

#include <string>
#include <unordered_map>

#include "CitySpace.h"
#include "CityBusLine.h"
#include "CityBusStop.h"

namespace dtsim {

/**
 * \class CityBusNetwork
 *
 * \brief
 * Bus network implementation
 */

typedef struct _Bus_Route_info {
	std::vector<std::pair<double, std::vector<CityBusLineSection*>>> busRoute;
} BusRouteInfo;

class CityBusNetwork: public CitySpace {

private:
	/// bus line map (bus line id, bus line object)
	typedef typename std::unordered_map<double, CityBusLine*> BusLineMap;
	typedef typename BusLineMap::iterator BusLineMapIterator;
	BusLineMap busLines;

	/// bus stop map (bus stop id, bus stop object)
	typedef typename std::unordered_map<double, CityBusStop*> BusStopMap;
	typedef typename BusStopMap::iterator BusStopMapIterator;
	BusStopMap busStops;

private:
	bool findLine(double lineId, std::vector<double>& lineIds);

	bool findLine(double lineId, std::vector<CityBusLine*>& lines);

	bool canTransfer(std::pair<int, int> startOrder, std::pair<int, int> goalOrder, int& fromOrder, int& toOrder);

	bool IsDirectRoute(double lineId, std::map<double, BusRouteInfo>& routeMap);

	bool canStop(std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds);

	void getDirectLines(double startId, double goalId, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance);

	void recurTransferRoutes(double startId, double goalId, std::vector<double> xferLine, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance);

	void getTransferRoutes(double startId, double goalId, std::vector<std::vector<double>> xferRoutes, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance);

	bool findDirects(std::vector<CityBusStop*> startStops, std::vector<CityBusStop*> goalStops, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance);

	bool findTransfers(std::vector<CityBusStop*> startStops, std::vector<CityBusStop*> goalStops, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance);

	bool findTransfers(std::vector<CityBusStop*> startStops, std::vector<CityBusStop*> goalStops, int max_routes, std::map<double, BusRouteInfo>& routeMap, std::vector<double>& directLineIds, double maxDistance);

	double getRideStops(double curStartId, double goalId, CityBusLine* curStartLine, std::vector<double>& xferLine, std::vector<std::pair<double, double>>& routeStops, int xlineIdx, std::vector<double>& directLineIds);

	void recurTransferLines(std::vector<CityBusLine*> startLines, CityBusLine* startLine, std::vector<CityBusLine*> goalLines, std::vector<double>& xferLine, std::vector<std::vector<double>>& xferRoutes);

	void getTransferLines(int startLineIdx, std::vector<CityBusLine*> startLines, std::vector<CityBusLine*> goalLines, std::vector<std::vector<double>>& xferRoutes);

	void printTransferMap();

public:
	typedef typename BusLineMap::const_iterator BusLineMapConstIterator;

	CityBusNetwork(std::string name, CityContext* context);
	virtual ~CityBusNetwork();

	/**
	 * Gets the start of iterator over the bus lines in this bus network
	 * The iterator->first is bus line id
	 * The iterator->second is bus line object
	 *
	 * @return the start of iterator over the bus lines in this bus network
	 */
	BusLineMapConstIterator bus_line_begin() const {
		return BusLineMapConstIterator(busLines.begin());
	}

	/**
	 * Gets the end of iterator over the bus lines in this bus network
	 * The iterator->first is bus line id
	 * The iterator->second is bus line object
	 *
	 * @return the end of iterator over the bus lines in this bus network
	 */
	BusLineMapConstIterator bus_line_end() const {
		return BusLineMapConstIterator(busLines.end());
	}

	/// Returns bus lines
	void getBusLines(std::vector<CityBusLine*>& lines) const;

	/// Returns bus line object
	CityBusLine* getBusLine(double busLineId) const;

	/// Add bus line object to bus network
	void addBusLine(CityBusLine* busLine);

	/// Remove bus line object from bus network
	void removeBusLine(double busLineId);

	/// Add bus stop object to bus network
	void addBusStop(CityBusStop* busStop);

	/// Remove bus stop object from bus network
	void removeBusStop(double stopId);

	/// Returns bus stop object
	CityBusStop* getBusStop(double stopId);

	/// Builds transfer bus lines
	void buildTransferLines();

	/// Returns bus route from start to goal
	void getBusRoute(CityCoordinate start, CityCoordinate goal, std::vector<BusRouteInfo>& routes, bool bShortest = false);

	void getBusRoute(double startX, double startY, double goalX, double goalY, std::vector<BusRouteInfo>& routes, bool bShortest = false);

	void getBusRoute(double startId, double goalId, std::vector<BusRouteInfo>& routes, bool bShortest = false);

	void getBusRoute(double startX, double startY, double goalX, double goalY, double max_radius, int max_routes, std::vector<BusRouteInfo>& routes, bool bDirect, bool bShortest = false);

	// Debug functions
	void printBusRoutes(std::vector<BusRouteInfo>& route);
	void printBusStops();
	void printBusLines();
	void getBusInfo(int& lines, int& stops) {
		lines = busLines.size();
		stops = busStops.size();
	}
};

} /* namespace dtsim */

#endif /* CITY_BUS_NETWORK_H */
