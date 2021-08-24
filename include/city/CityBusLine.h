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
 * CityBusLine.h
 *
 * $Revision$
 * $LastChangedDate$
 */

#ifndef CITY_BUS_LINE_H
#define CITY_BUS_LINE_H

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include <utility>

#include "CityGeometry.h"

namespace dtsim {

class CityBusStop;

class CityBusLineSection {
private:
	double fromBusStopId;
	double toBusStopId;

	CityGeometry* geometry;
	double length;
	std::vector<double> distances;

public:
	CityBusLineSection(double fromStopId, double toStopId, CityGeometry* geom);

	virtual ~CityBusLineSection();

	double getFromBusStopId() const {
		return fromBusStopId;
	}

	double getToBusStopId() const {
		return toBusStopId;
	}

	double getLength() const {
		return length;
	}

	std::vector<double> getDistances() const {
		return distances;
	}

	/// Returns a coordinate sequence of a building geometry
	std::unique_ptr<CityCoordinateSequence> getCoordinates() const;

	/// Pushes a coordinate sequence into the provided CityCoordinateSequence
	void getCoordinates(CityCoordinateSequence& seq) const;
};

/**
 * \class CityBusLine
 *
 * \brief
 * Bus line implementation
 */
class CityBusLine {
private:
	/// bus line identifier
	double busLineId;

	/// bus line name
	std::string busLineName;

	/// bus stop map (bus stop id, (bus stop object, bus stop order))
	typedef typename std::unordered_multimap<double, std::pair<CityBusStop*, int>> BusStopMap;
	typedef typename BusStopMap::iterator BusStopMapIterator;
	typedef typename BusStopMap::const_iterator BusStopMapConstIterator;
	BusStopMap busStops;

	/// bus line section map (bus line section order, bus line section object)
	typedef typename std::map<int, CityBusLineSection*> BusLineSectionMap;
	typedef typename BusLineSectionMap::iterator BusLineSectionMapIterator;
	typedef typename BusLineSectionMap::const_iterator BusLineSectionMapConstIterator;
	BusLineSectionMap busLineSections;

	/// transfer bus line map (transfer bus line id, adjacent bus line id and adjacent bus stops)
	typedef typename std::unordered_map<double, std::vector<std::pair<double, double>>> XferBusLineMap; 
	typedef typename XferBusLineMap::iterator XferBusLineMapIterator; 
	XferBusLineMap xferBusLines;

public:
	typedef typename XferBusLineMap::const_iterator XferBusLineMapConstIterator; 

	CityBusLine(double id, std::string name);

	virtual ~CityBusLine();

	XferBusLineMapConstIterator xferBusLine_begin() {
		return XferBusLineMapConstIterator(xferBusLines.begin());
	}

	XferBusLineMapConstIterator xferBusLine_end() const {
		return XferBusLineMapConstIterator(xferBusLines.end());
	}

	/// Returns a bus line identifier
	double getBusLineId() const {
		return busLineId;
	}

	/// Returns a bus line name
	std::string getBusLineName() const {
		return busLineName;
	}

	void addBusLineSection(CityBusStop* from, CityBusStop* to, int order, CityGeometry* geom);

	/// get Bus Stop object
	CityBusStop* getBusStop(double stopId) const;

	/// get all Bus Stop objects
	void getBusStops(std::vector<CityBusStop*>& stops) const;

	/// check if this bus line has a bus stop
	bool hasBusStop(double stopId) const;

	/// add bus stop
	void addBusStop(CityBusStop* stop, int sectionNum);

	/// get a bus line section starting from specified bus stop id
	CityBusLineSection* getBusLineSection(double fromStopId) const;

	/// get all sections in this bus line
	double getBusLineSections(std::vector<CityBusLineSection*>& sections) const;

	/// get sections from specified bus stop id to specified bus stop
	double getBusLineSections(double fromStopId, double toStopId, std::vector<CityBusLineSection*>& sections) const;

	/// get sections between specified orders
	double getBusLineSections(int fromOrder, int toOrder, std::vector<CityBusLineSection*>& sections) const;

	/// add a bus line section and bus stops
	void addBusLineSection(double fromStopId, double toStopId, int sectionNum, CityGeometry* geom);

	/// remove a bus line section starting from specified bus stop id
	void removeBusLineSection(double fromStopId);

	/// add a transfer bus line
	void addXferBusLine(double xferLineId, CityBusStop* board, CityBusStop* xfer);

	/// get transfer bus lines
	void getXferBusLines(std::vector<double>& out);

	/// get transfer bus lines excluding filters
	void getXferBusLines(std::vector<double> filters, std::vector<double>& out);

	/// get transfer bus stops after specified bus stop
	void getXferStops(double xferLineId, double stopId, std::multimap<int, std::pair<double, double>>& out);
};

} /* namespace dtsim */

#endif /* CITY_BUS_LINE_H */
