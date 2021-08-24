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
 * CityGeometryFactory.h
 *
 * $Revision: 975 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_GEOMETRY_FACTORY_H
#define CITY_GEOMETRY_FACTORY_H

#include <vector>

#include "geos/geom/GeometryFactory.h"
#include "geos/util/GeometricShapeFactory.h"
#include "CityGeometry.h"
#include "CityCoordinate.h"
#include "CityCoordinateSequence.h"

namespace dtsim {

/**
 * \class CityGeometryFactory
 *
 * \brief
 * Creates CityGeometry objects
 *
 * Creates CityGeometry objects using GEOS GeometryFactory and GeometricShapeFactory.
 * To use CityGeometryFactory, you can use instance() static function.
 */
class CityGeometryFactory {
private:
	/// GEOS GeometryFactory object
	const geos::geom::GeometryFactory* geosGeomFactory;

	/// GEOS GeometricShapeFactory object
	geos::util::GeometricShapeFactory* geosGeomShapeFactory;

	static CityGeometryFactory* _instance;

public:
	CityGeometryFactory();
	virtual ~CityGeometryFactory();

	static CityGeometryFactory* instance();

	/**
	 * Creates point
	 *
	 * @param x x-coordinate
	 * @param y y-coordinate
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPoint(double x, double y);

	/**
	 * Creates point
	 *
	 * @param coord vector of coordinate
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPoint(std::vector<double>& coord);

	/**
	 * Creates point
	 *
	 * @param coord CityCoordinate object
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPoint(CityCoordinate& coord);

	/**
	 * Creates an elliptical arc, as a linestring
	 * The arc is always created in a counter-clockwise direction
	 *
	 * @param baseX the location of the shape(in most cases is the lower left point of the envelope containing the shape)
	 * @param baseY the location of the shape(in most cases is the lower left point of the envelope containing the shape)
	 * @param width the width of the shape
	 * @param height the height of the shape
	 * @param numPoints
	 * @param startAng start angle in radians
	 * @param angExtent size of angle in radians
	 * @return returns CityGeometry object
	 */
	CityGeometry* createArc(double baseX, double baseY, double width, double height, int numPoints, double startAng, double angExtent);

	/**
	 * Creates an elliptical arc, as a linestring
	 * The arc is always created in a counter-clockwise direction
	 *
	 * @param xCoords vector of x-coordinates
	 * @param yCoords vector of y-coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createArc(std::vector<double>& xCoords, std::vector<double>& yCoords);

	/**
	 * Creates an elliptical arc, as a linestring
	 * The arc is always created in a counter-clockwise direction
	 *
	 * @param coords vector of coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createArc(std::vector<double>& coords);

	/**
	 * Creates a circular polygon
	 *
	 * @param centerX the centre X of the shape's bouding box
	 * @param centerY the centre Y of the shape's bouding box
	 * @param numPoints
	 * @param radius the size of the extent of the shape in both x and y directions
	 * @return returns CityGeometry object
	 */
	CityGeometry* createCircle(double centerX, double centerY, int numPoints, double radius);

	/**
	 * Creates an elliptical polygon
	 *
	 * @param centerX the centre X of the shape's bouding box
	 * @param centerY the centre Y of the shape's bouding box
	 * @param width the width of the shape
	 * @param height the height of the shape
	 * @param numPoints
	 * @return returns CityGeometry object
	 */
	CityGeometry* createEllipse(double centerX, double centerY, double width, double height, int numPoints);

	/**
	 * Creates a rectangular polygon
	 *
	 * @param baseX the location of the shape(in most cases is the lower left point of the envelope containing the shape)
	 * @param baseY the location of the shape(in most cases is the lower left point of the envelope containing the shape)
	 * @param width the width of the shape
	 * @param height the height of the shape
	 * @return returns CityGeometry object
	 */
	CityGeometry* createRectangle(double baseX, double baseY, double width, double height);

	/**
	 * Creates an elliptical arc polygon
	 *
	 * @param centerX the centre X of the shape's bouding box
	 * @param centerY the centre Y of the shape's bouding box
	 * @param radius the size of the extent of the shape in both x and y directions
	 * @param numArcPoints
	 * @param startAng start angle in radians
	 * @param angExtent size of angle in radians
	 * @return returns CityGeometry object
	 */
	CityGeometry* createArcPolygon(double centerX, double centerY, double radius, int numArcPoints, double startAng, double angExtent);

	/**
	 * Create polygon
	 *
	 * @param centerX the centre X of the shape's bouding box
	 * @param centerY the centre Y of the shape's bouding box
	 * @param radius the size of the extent of the shape in both x and y directions
	 * @param numPoints
	 * @param xCoords vector of x-coordinates
	 * @param yCoords vector of y-coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(double centerX, double centerY, double radius, int numPoints, std::vector<double>& xCoords, std::vector<double>& yCoords);

	/**
	 * Create polygon
	 *
	 * @param centerX the centre X of the shape's bouding box
	 * @param centerY the centre Y of the shape's bouding box
	 * @param radius the size of the extent of the shape in both x and y directions
	 * @param numPoints
	 * @param coords vector of coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(double centerX, double centerY, double radius, int numPoints, std::vector<double>& coords);

	/**
	 * Create polygon
	 *
	 * @param xCoords vector of x-coordinates
	 * @param yCoords vector of y-coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(std::vector<double>& xCoords, std::vector<double>& yCoords);

	/**
	 * Create polygon
	 *
	 * @param coords vector of coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(std::vector<double>& coords);

	/**
	 * Create polygon
	 *
	 * @param partXcoords vector of vector of x-coordinates
	 * @param partYcoords vector of vector of y-coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(std::vector<std::vector<double>>& partXcoords, std::vector<std::vector<double>>& partYcoords);

	/**
	 * Create polygon
	 *
	 * @param partCoords vector of vector of coordinates
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(std::vector<std::vector<double>>& partCoords);

	/**
	 * Create polygon
	 *
	 * @param coordSeq CityCoordinateSequence object
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(CityCoordinateSequence& coordSeq);

	/**
	 * Create polygon
	 *
	 * @param partCoordSeq vector of CityCoordinateSequence object
	 * @return returns CityGeometry object
	 */
	CityGeometry* createPolygon(std::vector<CityCoordinateSequence>& partCoordSeq);
};

} /* namespace dtsim */

#endif /* CITY_GEOMETRY_FACTORY_H */
