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
 * CityGeometryFactory.cpp
 *
 * $Revision: 975 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "geos/geom/CoordinateArraySequenceFactory.h"
#include "geos/geom/LinearRing.h"
#include "geos/geom/Point.h"
#include "geos/geom/Polygon.h"
#include "CityGeometryFactory.h"

namespace dtsim {

CityGeometryFactory::CityGeometryFactory()
{
	geosGeomFactory = geos::geom::GeometryFactory::getDefaultInstance();
	geosGeomShapeFactory = new geos::util::GeometricShapeFactory(geosGeomFactory);
}

CityGeometryFactory::~CityGeometryFactory()
{
	delete geosGeomShapeFactory;
	_instance = 0;
}

CityGeometryFactory* CityGeometryFactory::_instance = 0;

CityGeometryFactory* CityGeometryFactory::instance()
{
	if (_instance == 0)
		_instance = new CityGeometryFactory();

	return _instance;
}

CityGeometry* CityGeometryFactory::createPoint(double x, double y)
{
	geos::geom::Coordinate geosCoord(x, y);
	geos::geom::Point* point = geosGeomFactory->createPoint(geosCoord);

	CityGeometry* cityGeom = new CityGeometry(point);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPoint(std::vector<double>& coord)
{
	return createPoint(coord[0], coord[1]);
}

CityGeometry* CityGeometryFactory::createPoint(CityCoordinate& coord)
{
	return createPoint(coord.x, coord.y);
}

CityGeometry* CityGeometryFactory::createArc(double baseX, double baseY, double width, double height, int numPoints, double startAng, double angExtent)
{
	geosGeomShapeFactory->setBase(geos::geom::Coordinate(baseX, baseY));
	geosGeomShapeFactory->setWidth(width);
	geosGeomShapeFactory->setHeight(height);
	geosGeomShapeFactory->setNumPoints(numPoints);

	std::unique_ptr<geos::geom::LineString> arc = geosGeomShapeFactory->createArc(startAng, angExtent);

	CityGeometry* cityGeom = new CityGeometry(arc.release());

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createArc(std::vector<double>& xCoords, std::vector<double>& yCoords)
{
	if (xCoords.empty() || yCoords.empty())
		return nullptr;

	if (xCoords.size() != yCoords.size())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();

	for (std::size_t i = 0, n = xCoords.size(); i < n; i++)
		geosCoordSeq->add(geos::geom::Coordinate(xCoords[i], yCoords[i]));

	geos::geom::LineString* arc = geosGeomFactory->createLineString(geosCoordSeq);

	CityGeometry* cityGeom = new CityGeometry(arc);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createArc(std::vector<double>& coords)
{
	if (coords.empty())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();

	for (std::size_t i = 0, n = coords.size(); i < n; i += 2)
		geosCoordSeq->add(geos::geom::Coordinate(coords[i], coords[i+1]));

	geos::geom::LineString* arc = geosGeomFactory->createLineString(geosCoordSeq);

	CityGeometry* cityGeom = new CityGeometry(arc);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createCircle(double centerX, double centerY, int numPoints, double radius)
{
	geosGeomShapeFactory->setCentre(geos::geom::Coordinate(centerX, centerY));
	geosGeomShapeFactory->setSize(radius);
	geosGeomShapeFactory->setNumPoints(numPoints);

	std::unique_ptr<geos::geom::Polygon> circle = geosGeomShapeFactory->createCircle();

	CityGeometry* cityGeom = new CityGeometry(circle.release());

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createEllipse(double centerX, double centerY, double width, double height, int numPoints)
{
	geosGeomShapeFactory->setCentre(geos::geom::Coordinate(centerX, centerY));
	geosGeomShapeFactory->setWidth(width);
	geosGeomShapeFactory->setHeight(height);
	geosGeomShapeFactory->setNumPoints(numPoints);

	std::unique_ptr<geos::geom::Polygon> ellipse = geosGeomShapeFactory->createCircle();

	CityGeometry* cityGeom = new CityGeometry(ellipse.release());

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createRectangle(double baseX, double baseY, double width, double height)
{
	geosGeomShapeFactory->setBase(geos::geom::Coordinate(baseX, baseY));
	geosGeomShapeFactory->setWidth(width);
	geosGeomShapeFactory->setHeight(height);
	geosGeomShapeFactory->setNumPoints(4);

	std::unique_ptr<geos::geom::Polygon> rectangle = geosGeomShapeFactory->createRectangle();

	CityGeometry* cityGeom = new CityGeometry(rectangle.release());

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createArcPolygon(double centerX, double centerY,
													double radius, int numArcPoints,
													double startAng, double angExtent)
{
	geos::geom::Coordinate center(centerX, centerY);

	geosGeomShapeFactory->setCentre(center);
	geosGeomShapeFactory->setSize(radius);
	geosGeomShapeFactory->setNumPoints(numArcPoints);

	std::unique_ptr<geos::geom::LineString> line = geosGeomShapeFactory->createArc(startAng, angExtent);

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();

	geosCoordSeq->add(center);
	for (std::size_t i = 0; i < numArcPoints; i++)
		geosCoordSeq->add(line->getCoordinateN(i));
	geosCoordSeq->add(center);

	geos::geom::LinearRing* ring = geosGeomFactory->createLinearRing(geosCoordSeq);
	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(ring, nullptr);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(double centerX, double centerY,
												 double radius, int numPoints,
												 std::vector<double>& xCoords,
												 std::vector<double>& yCoords)
{
	if (xCoords.empty() || yCoords.empty())
		return nullptr;

	if (xCoords.size() != yCoords.size())
		return nullptr;

	geosGeomShapeFactory->setCentre(geos::geom::Coordinate(centerX, centerY));
	geosGeomShapeFactory->setSize(radius);
	geosGeomShapeFactory->setNumPoints(numPoints);

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();

	for (std::size_t i = 0; i < numPoints; i++)
		geosCoordSeq->add(geos::geom::Coordinate(xCoords[i], yCoords[i]));

	geos::geom::LinearRing* ring = geosGeomFactory->createLinearRing(geosCoordSeq);
	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(ring, nullptr);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(double centerX, double centerY,
												 double radius, int numPoints,
												 std::vector<double>& coords)
{
	if (coords.empty())
		return nullptr;

	geosGeomShapeFactory->setCentre(geos::geom::Coordinate(centerX, centerY));
	geosGeomShapeFactory->setSize(radius);
	geosGeomShapeFactory->setNumPoints(numPoints);

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();

	for (std::size_t i = 0; i < numPoints; i += 2)
		geosCoordSeq->add(geos::geom::Coordinate(coords[i], coords[i+1]));

	geos::geom::LinearRing* ring = geosGeomFactory->createLinearRing(geosCoordSeq);
	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(ring, nullptr);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(std::vector<double>& xCoords, std::vector<double>& yCoords)
{
	if (xCoords.empty() || yCoords.empty())
		return nullptr;

	if (xCoords.size() != yCoords.size())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();

	for (std::size_t i = 0, n = xCoords.size(); i < n; i++)
		geosCoordSeq->add(geos::geom::Coordinate(xCoords[i], yCoords[i]));

	geos::geom::LinearRing* ring = geosGeomFactory->createLinearRing(geosCoordSeq);
	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(ring, nullptr);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(std::vector<double>& coords)
{
	if (coords.empty())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();

	for (std::size_t i = 0, n = coords.size(); i < n; i += 2)
		geosCoordSeq->add(geos::geom::Coordinate(coords[i], coords[i+1]));

	geos::geom::LinearRing* ring = geosGeomFactory->createLinearRing(geosCoordSeq);
	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(ring, nullptr);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(std::vector<std::vector<double>>& partXcoords,
												 std::vector<std::vector<double>>& partYcoords)
{
	if (partXcoords.empty() || partYcoords.empty())
		return nullptr;

	if (partXcoords.size() || partYcoords.size())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq;

	geosCoordSeq = new geos::geom::CoordinateArraySequence();
	for (std::size_t i = 0, n = partXcoords[0].size(); i < n; i++) {
		geosCoordSeq->add(geos::geom::Coordinate(partXcoords[0][i], partYcoords[0][i]));
	}
	geos::geom::LinearRing* shell = geosGeomFactory->createLinearRing(geosCoordSeq);

	std::vector<geos::geom::LinearRing*>* holes = nullptr;
	geos::geom::LinearRing* hole;
	if (partXcoords.size() > 1) {
		holes = new std::vector<geos::geom::LinearRing*>;

		for (std::size_t i = 1, n1 = partXcoords.size()-1; i < n1; i++) {
			geosCoordSeq = new geos::geom::CoordinateArraySequence();
			for (std::size_t j = 0, n2 = partXcoords[i].size(); j < n2; j++) {
				geosCoordSeq->add(geos::geom::Coordinate(partXcoords[i][j], partYcoords[i][j]));
			}
			hole = geosGeomFactory->createLinearRing(geosCoordSeq);
			holes->push_back(hole);
		}
	}

	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(shell, holes);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(std::vector<std::vector<double>>& partCoords)
{
	if (partCoords.empty())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq;
	double x, y;

	geosCoordSeq = new geos::geom::CoordinateArraySequence();
	for (std::size_t i = 0, n = partCoords[0].size(); i < n; i += 2) {
		x = partCoords[0][i];
		y = partCoords[0][i+1];
		geosCoordSeq->add(geos::geom::Coordinate(x, y));
	}
	geos::geom::LinearRing* shell = geosGeomFactory->createLinearRing(geosCoordSeq);

	std::vector<geos::geom::LinearRing*>* holes = nullptr;
	geos::geom::LinearRing* hole;
	if (partCoords.size() > 1) {
		holes = new std::vector<geos::geom::LinearRing*>;

		for (std::size_t i = 1, n1 = partCoords.size()-1; i < n1; i++) {
			geosCoordSeq = new geos::geom::CoordinateArraySequence();
			for (std::size_t j = 0, n2 = partCoords[i].size(); j < n2; j += 2) {
				x = partCoords[i][j];
				y = partCoords[i][j+1];
				geosCoordSeq->add(geos::geom::Coordinate(x, y));
			}
			hole = geosGeomFactory->createLinearRing(geosCoordSeq);
			holes->push_back(hole);
		}
	}

	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(shell, holes);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(CityCoordinateSequence& coordSeq)
{
	if (coordSeq.isEmpty())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq = new geos::geom::CoordinateArraySequence();
	double x, y;

	for (std::size_t i = 0, n = coordSeq.getSize(); i < n; i++) {
		coordSeq.getAt(i, x, y);
		geosCoordSeq->add(geos::geom::Coordinate(x, y));
	}

	geos::geom::LinearRing* ring = geosGeomFactory->createLinearRing(geosCoordSeq);
	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(ring, nullptr);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

CityGeometry* CityGeometryFactory::createPolygon(std::vector<CityCoordinateSequence>& partCoordSeq)
{
	if (partCoordSeq.empty())
		return nullptr;

	geos::geom::CoordinateArraySequence* geosCoordSeq;
	double x, y;

	geosCoordSeq = new geos::geom::CoordinateArraySequence();
	for (std::size_t i = 0, n = partCoordSeq[0].getSize(); i < n; i++) {
		partCoordSeq[0].getAt(i, x, y);
		geosCoordSeq->add(geos::geom::Coordinate(x, y));
	}
	geos::geom::LinearRing* shell = geosGeomFactory->createLinearRing(geosCoordSeq);

	std::vector<geos::geom::LinearRing*>* holes = nullptr;
	geos::geom::LinearRing* hole;
	if (partCoordSeq.size() > 1) {
		holes = new std::vector<geos::geom::LinearRing*>;

		for (std::size_t i = 1, n1 = partCoordSeq.size()-1; i < n1; i++) {
			geosCoordSeq = new geos::geom::CoordinateArraySequence();
			for (std::size_t j = 0, n2 = partCoordSeq[i].getSize(); j < n2; j++) {
				partCoordSeq[i].getAt(j, x, y);
				geosCoordSeq->add(geos::geom::Coordinate(x, y));
			}
			hole = geosGeomFactory->createLinearRing(geosCoordSeq);
			holes->push_back(hole);
		}
	}

	geos::geom::Polygon* polygon = geosGeomFactory->createPolygon(shell, holes);

	CityGeometry* cityGeom = new CityGeometry(polygon);

	return cityGeom;
}

} /* namespace dtsim */
