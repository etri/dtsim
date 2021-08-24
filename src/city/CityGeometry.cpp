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
 * CityGeometry.cpp
 *
 * $Revision: 640 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "CityGeometryFactory.h"
#include "CityGeometry.h"

namespace dtsim {

std::unique_ptr<CityCoordinate> CityGeometry::getCoordinate() const
{
	double x = geosGeom->getCoordinate()->x;
	double y = geosGeom->getCoordinate()->y;
	std::unique_ptr<CityCoordinate> coord(new CityCoordinate(x, y));

	return coord;
}

void CityGeometry::getCoordinate(CityCoordinate& coord) const
{
	coord.x = geosGeom->getCoordinate()->x;
	coord.y = geosGeom->getCoordinate()->y;
}

void CityGeometry::getCoordinate(double& x, double& y) const
{
	x = geosGeom->getCoordinate()->x;
	y = geosGeom->getCoordinate()->y;
}

std::unique_ptr<CityCoordinate> CityGeometry::getCoordinate(std::size_t i) const
{
	if (geosGeom->getNumPoints() <= i)
		return nullptr;

	double x = geosGeom->getCoordinates()->getAt(i).x;
	double y = geosGeom->getCoordinates()->getAt(i).y;
	std::unique_ptr<CityCoordinate> coord(new CityCoordinate(x, y));

	return coord;
}

void CityGeometry::getCoordinate(std::size_t i, CityCoordinate& coord) const
{
	std::unique_ptr<geos::geom::CoordinateSequence> geosCoordSeq = geosGeom->getCoordinates();

	if (geosGeom->getNumPoints() <= i)
		return;

	coord.x = geosCoordSeq->getAt(i).x;
	coord.y = geosCoordSeq->getAt(i).y;
}

void CityGeometry::getCoordinate(std::size_t i, double& x, double& y) const
{
	std::unique_ptr<geos::geom::CoordinateSequence> geosCoordSeq = geosGeom->getCoordinates();

	if (geosGeom->getNumPoints() <= i)
		return;

	x = geosCoordSeq->getAt(i).x;
	y = geosCoordSeq->getAt(i).y;
}

std::unique_ptr<CityCoordinateSequence> CityGeometry::getCoordinates() const
{
	std::unique_ptr<CityCoordinateSequence> coordSeq(new CityCoordinateSequence());

	std::unique_ptr<geos::geom::CoordinateSequence> geosCoordSeq = geosGeom->getCoordinates();
	double x, y;

	for (std::size_t i = 0, n = geosCoordSeq->getSize(); i < n; i++) {
		x = geosCoordSeq->getAt(i).x;
		y = geosCoordSeq->getAt(i).y;
		coordSeq->add(x, y);
	}

	return coordSeq;
}

void CityGeometry::getCoordinates(CityCoordinateSequence& coordSeq) const
{
	std::unique_ptr<geos::geom::CoordinateSequence> geosCoordSeq = geosGeom->getCoordinates();
	double x, y;

	for (std::size_t i = 0, n = geosCoordSeq->getSize(); i < n; i++) {
		x = geosCoordSeq->getAt(i).x;
		y = geosCoordSeq->getAt(i).y;
		coordSeq.add(x, y);
	}
}

bool CityGeometry::getCentroid(CityCoordinate& coord) const
{
	geos::geom::Coordinate geomCoord;

	if (geosGeom->getCentroid(geomCoord)) {
		coord.x = geomCoord.x;
		coord.y = geomCoord.y;
		return true;
	}

	return false;
}

bool CityGeometry::getCentroid(double& x, double& y) const
{
	geos::geom::Coordinate geomCoord;

	if (geosGeom->getCentroid(geomCoord)) {
		x = geomCoord.x;
		y = geomCoord.y;
		return true;
	}

	return false;
}

CityGeometry* CityGeometry::getCentroid() const
{
	geos::geom::Coordinate geosCoord;

	if (geosGeom->getCentroid(geosCoord)) {
		CityGeometry* point = CityGeometryFactory::instance()->createPoint(geosCoord.x, geosCoord.y);
		return point;
	}

	return nullptr;
}

} /* namespace dtsim */
