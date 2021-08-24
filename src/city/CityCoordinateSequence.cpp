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
 * CityCoordinateSequence.cpp
 *
 * $Revision: 640 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "CityCoordinateSequence.h"

namespace dtsim {

CityCoordinateSequence::CityCoordinateSequence(const CityCoordinateSequence& c)
{
	for (std::size_t i = 0, n = c.getSize(); i < n; i++)
		coordSeq.push_back(c.getAt(i));
}

CityCoordinateSequence& CityCoordinateSequence::operator=(const CityCoordinateSequence& c)
{
	if (this != &c) {
		for (std::size_t i = 0, n = c.getSize(); i < n; i++)
			coordSeq.push_back(c.getAt(i));
	}

	return *this;
}

std::size_t CityCoordinateSequence::indexOf(const CityCoordinate* c, const CityCoordinateSequence* cs)
{
	for (std::size_t i = 0, n = cs->getSize(); i < n; i++) {
		if ((*c) == cs->getAt(i))
			return i;
	}

	return std::numeric_limits<std::size_t>::max();
}

bool CityCoordinateSequence::equals(const CityCoordinateSequence* s1, const CityCoordinateSequence* s2)
{
	if (s1 == s2)
		return true;

	if ((s1 == nullptr) || (s2 == nullptr))
		return false;

	std::size_t n = s1->getSize();
	if (n != s2->getSize())
		return false;

	double x1, y1;
	double x2, y2;

	for (std::size_t i = 0; i < n; i++) {
		s1->getAt(i, x1, y1);
		s2->getAt(i, x2, y2);

		if (x1 != x2)
			return false;
		if (y1 != y2)
			return false;
	}

	return true;
}

void CityCoordinateSequence::reverse(CityCoordinateSequence* cs)
{
	std::size_t start = 0;
	std::size_t end = cs->getSize() - 1;
	double startX, startY;
	double endX, endY;

	while (start < end) {
		cs->getAt(start, startX, startY);
		cs->getAt(end, endX, endY);

		cs->setAt(endX, endY, start);
		cs->setAt(startX, startY, end);

		start++;
		end--;
	}
}

void CityCoordinateSequence::scroll(CityCoordinateSequence* cs, const CityCoordinate* firstCoord)
{
	std::size_t index = indexOf(firstCoord, cs);
	if (index < 1)
		return; // not found or already first

	std::size_t i, j = 0;
	std::size_t length = cs->getSize();
	std::vector<CityCoordinate> v(length);

	for (i = index; i < length; i++)
		v[j++] = cs->getAt(i);

	for (i = 0; i < index; i++)
		v[j++] = cs->getAt(i);

	cs->setPoints(v);
}

std::size_t CityCoordinateSequence::getSize() const
{
	return coordSeq.size();
}

bool CityCoordinateSequence::isEmpty() const
{
	return coordSeq.empty();
}

void CityCoordinateSequence::toVector(std::vector<CityCoordinate>& coords) const
{
	coords.insert(coords.end(), coordSeq.begin(), coordSeq.end());
}

const CityCoordinate& CityCoordinateSequence::getAt(std::size_t pos) const
{
	return coordSeq[pos];
}

void CityCoordinateSequence::getAt(std::size_t pos, CityCoordinate& c) const
{
	c = coordSeq[pos];
}

void CityCoordinateSequence::getAt(std::size_t pos, double& x, double& y) const
{
	x = coordSeq[pos].x;
	y = coordSeq[pos].y;
}

const CityCoordinate& CityCoordinateSequence::operator[](std::size_t pos) const
{
	return getAt(pos);
}

void CityCoordinateSequence::setAt(const CityCoordinate& c, std::size_t pos)
{
	coordSeq[pos].x = c.x;
	coordSeq[pos].y = c.y;
}

void CityCoordinateSequence::setAt(double x, double y, std::size_t pos)
{
	coordSeq[pos].x = x;
	coordSeq[pos].y = y;
}

void CityCoordinateSequence::setPoints(const std::vector<CityCoordinate>& v)
{
	coordSeq.assign(v.begin(), v.end());
}

void CityCoordinateSequence::add(const CityCoordinate& c)
{
	coordSeq.push_back(c);
}

void CityCoordinateSequence::add(double x, double y)
{
	coordSeq.push_back(CityCoordinate(x, y));
}

std::string CityCoordinateSequence::toString() const
{
	std::string result("(");
	if (getSize() > 0) {
		for (std::size_t i = 0, n = coordSeq.size(); i < n; i++) {
			if (i)
				result.append(", ");
			result.append(coordSeq[i].toString());
		}
	}
	result.append(")");

	return result;
}

std::ostream& operator<<(std::ostream& os, const CityCoordinateSequence& cs)
{
	os << "(";
	for (std::size_t i = 0, n = cs.getSize(); i < n; ++i) {
		if (i)
			os << ", ";
		os << cs.getAt(i);
	}
	os << ")";

	return os;
}

bool operator==(const CityCoordinateSequence& s1, const CityCoordinateSequence& s2)
{
	return CityCoordinateSequence::equals(&s1, &s2);
}

bool operator!=(const CityCoordinateSequence& s1, const CityCoordinateSequence& s2)
{
	return !CityCoordinateSequence::equals(&s1, &s2);
}

CityCoordinateSequenceFilter::CityCoordinateSequenceFilter(const CityCoordinateSequence& seq)
: done(false), changed(false)
{
	size = seq.getSize();

	double x, y;
	for (std::size_t i = 0; i < size; i++) {
		seq.getAt(i, x, y);

		geosCoordSeq.add(geos::geom::Coordinate(x, y));
	}
}

void CityCoordinateSequenceFilter::filter_rw(geos::geom::CoordinateSequence& seq, size_t n)
{
	const geos::geom::Coordinate coord1 = geosCoordSeq.getAt(n);
	const geos::geom::Coordinate coord2 = seq.getAt(n);

	if (!changed && !coord1.equals(coord2))
		changed = true;

	seq.setAt(coord1, n);

	if (n == size)
		done = true;
}

} /* namespace dtsim */
