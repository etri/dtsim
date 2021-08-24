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
 * CityBusStop.h
 *
 * $Revision$
 * $LastChangedDate$
 */

#ifndef CITY_BUS_STOP_H
#define CITY_BUS_STOP_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <utility>

#include "CityAgent.h"

namespace dtsim {

class CityBusLine;

/**
 * \class CityBusStop
 *
 * \brief
 * Bus stop agent implementation
 */
class CityBusStop: public CityAgent {
protected:
	/// bus stop identifier
	double busStopId;

	/// bus stop name
	std::string busStopName;

	/// bus line map through this bus stop (bus line id, (bus line object, bus stop order))
	/// bus stop can have multiple order on a bus line. (ex. first and last bus stop)
	typedef typename std::unordered_multimap<double, std::pair<CityBusLine*, int>> BusLineMap;
	typedef typename BusLineMap::iterator BusLineMapIterator;
	typedef typename BusLineMap::const_iterator BusLineMapConstIterator;
	BusLineMap busLines;

public:
	CityBusStop(double stopId, std::string stopName);

	virtual ~CityBusStop() {}

	/// Returns a bus stop identifier
	double getBusStopId() const {
		return busStopId;
	}

	/// Returns a bus stop name
	std::string getBusStopName() const {
		return busStopName;
	}

	/// Returns a bus stop order on bus line
	std::pair<int, int> getBusStopOrder(double lineId) const;

	/// Returns a bus line object of specified bus line id
	CityBusLine* getBusLine(double lineId) const;

	/// Returns bus lines going through this bus stop
	void getBusLines(std::vector<CityBusLine*>& lines) const;

	void getBusLines(std::vector<CityBusLine*> filterLines, std::vector<CityBusLine*>& lines) const;

	/// Returns bus lines and orders through this bus stop
	void getBusLines(std::vector<std::pair<CityBusLine*, int>>& lines) const;

	int getBusLinesCount() const {
		return busLines.size();
	}

	/// check if specified line go through this bus stop
	bool hasBusLine(double lineId) const;

	/// Add bus line through this bus stop
	void addBusLine(CityBusLine* busLine, int order);

	/// Remove bus line
	void removeBusLine(double lineId);

	/// Returns a coordinate of a bus stop geometry
	std::unique_ptr<CityCoordinate> getCoordinate() const;

	/// Pushes a coordinate into the provided CityCoordinate
	void getCoordinate(CityCoordinate& c) const;

	/// Pushes a coordinate into the provided variables
	void getCoordinate(double& x, double& y) const;
};

} /* namespace dtsim */

#endif /* CITY_BUS_STOP_H */
