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
 * CityCoordinateSequence.h
 *
 * $Revision: 640 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_COORDINATE_SEQUENCE_H
#define CITY_COORDINATE_SEQUENCE_H

#include <iostream>
#include <string>
#include <vector>

#include "geos/geom/CoordinateSequence.h"
#include "geos/geom/CoordinateArraySequence.h"
#include "geos/geom/CoordinateSequenceFilter.h"
#include "CityCoordinate.h"

namespace dtsim {

/**
 * \class CityCoordinateSequence
 *
 * \brief
 * Coordinate sequence information
 */
class CityCoordinateSequence {
private:
	/// coordinate sequence
	std::vector<CityCoordinate> coordSeq;

public:
	CityCoordinateSequence() {}

	/// CityCoordinateSequence copy constructor
	CityCoordinateSequence(const CityCoordinateSequence& c);

	virtual ~CityCoordinateSequence() {}

	CityCoordinateSequence& operator=(const CityCoordinateSequence& c);

	/// Returns position of a coordinate, or numeric_limits<size_t>::max as not found value
	static std::size_t indexOf(const CityCoordinate* c, const CityCoordinateSequence* cs);

	/// Returns true if two coordinate sequences are identical
	static bool equals(const CityCoordinateSequence* s1, const CityCoordinateSequence* s2);

	/// Reverse Coordinate order in given CoordinateSequences
	static void reverse(CityCoordinateSequence* cs);

	/// Scroll given CoordinateSequence so to start with given Coordinate
	static void scroll(CityCoordinateSequence* cs, const CityCoordinate* firstCoord);

	/// Returns the number of coordinates
	std::size_t getSize() const;

	/// Returns true if not contains coordinates
	bool isEmpty() const;

	/// Pushes all coordinates of this sequence into the provided vector
	void toVector(std::vector<CityCoordinate>& coords) const;

	/// Returns a coordinate at specified position
	const CityCoordinate& getAt(std::size_t pos) const;

	/// Returns a coordinate at specified position to given coordinate
	void getAt(std::size_t pos, CityCoordinate& c) const;

	/// Returns a coordinate at specified position to given xy coordinate
	void getAt(std::size_t pos, double& x, double& y) const;

	/// Returns a coordinate at specified position
	//const CityCoordinate* operator[](std::size_t pos) const;
	const CityCoordinate& operator[](std::size_t pos) const;

	/// Copy a coordinate to specified position
	void setAt(const CityCoordinate& c, std::size_t pos);

	/// Copy xy coordinate to specified position
	void setAt(double x, double y, std::size_t pos);

	/// Substitute coordinate list with a copy of the given vector
	void setPoints(const std::vector<CityCoordinate>& v);

	/// Add a coordinate to coordinate sequence
	void add(const CityCoordinate& c);

	/// Add xy coordinate to coordinate sequence
	void add(double x, double y);

	/// Returns string representation of coordinate sequence
	std::string toString() const;
};

std::ostream& operator<<(std::ostream& os, const CityCoordinateSequence& cs);
bool operator==(const CityCoordinateSequence& s1, const CityCoordinateSequence& s2);
bool operator!=(const CityCoordinateSequence& s1, const CityCoordinateSequence& s2);

/**
 * \class CityCoordinateSequenceFilter
 *
 * \brief
 * Coordinate sequence filter to apply to coordinate sequence
 */
class CityCoordinateSequenceFilter: public geos::geom::CoordinateSequenceFilter {
private:
	geos::geom::CoordinateArraySequence geosCoordSeq;
	std::size_t size;
	bool done;
	bool changed;

public:
	CityCoordinateSequenceFilter(const CityCoordinateSequence& seq);
	virtual ~CityCoordinateSequenceFilter() {}

	bool isDone() const {
		return done;
	}

	bool isGeometryChanged() const {
		return changed;
	}

	void filter_rw(geos::geom::CoordinateSequence& seq, std::size_t n);
};

} /* namespace dtsim */

#endif /* CITY_COORDINATE_SEQUENCE_H */
