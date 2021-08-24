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
 * CityGeometry.h
 *
 * $Revision: 630 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_GEOMETRY_H
#define CITY_GEOMETRY_H

#include "geos/geom/Geometry.h"
#include "CityCoordinate.h"
#include "CityCoordinateSequence.h"

namespace dtsim {

/**
 * \class CityGeometry
 *
 * \brief
 * Geometry implementation
 *
 * CityGeometry is constructed by CityGeometryFactory.
 * GEOS(Geometry Engine - Open Source) is a C++ port of the JTS Topology Suite(JTS).
 * https://trac.osgeo.org/geos
 */
class CityGeometry {
private:
	/// GEOS geometry object
	geos::geom::Geometry* geosGeom;

public:
	CityGeometry(geos::geom::Geometry* geom) : geosGeom(geom) {}

	virtual ~CityGeometry() { delete geosGeom; }

	/// Non User API
	geos::geom::Geometry* getGEOSgeometry() const {
		return geosGeom;
	}

	/// Non User API
	void setGEOSgeometry(geos::geom::Geometry* geom) {
		geosGeom = geom;
	}

	bool isEmpty() const {
		return geosGeom->isEmpty();
	}

	/// Returns the count of this geometrys vertices.
	std::size_t getNumPoints() const {
		return geosGeom->getNumPoints();
	}

	/// Returns the length of this geometry.
	double getLength() const {
		return geosGeom->getLength();
	}

	/// Returns the minimum distance between this and geom.
	double distance(const CityGeometry* geom) const {
		return geosGeom->distance(geom->getGEOSgeometry());
	}

	/// Returns coordinate of this geometry. Caller takes ownership of the returned object.
	std::unique_ptr<CityCoordinate> getCoordinate() const;

	/// Returns coordinate of this geometry.
	void getCoordinate(CityCoordinate& coord) const;

	/// Returns XY coordinate of this geometry.
	void getCoordinate(double& x, double& y) const;

	/// Returns coordinate at position i of this geometry. Caller takes ownership of the returned object.
	std::unique_ptr<CityCoordinate> getCoordinate(std::size_t i) const;

	/// Returns coordinate at position i of this geometry.
	void getCoordinate(std::size_t i, CityCoordinate& coord) const;

	/// Returns XY coordinate at position i of this geometry.
	void getCoordinate(std::size_t i, double& x, double& y) const;

	/// Returns coordinate sequence of this geometry. Caller takes ownership of the returned object.
	std::unique_ptr<CityCoordinateSequence> getCoordinates() const;

	/// Returns coordinate sequence of this geometry.
	void getCoordinates(CityCoordinateSequence& coordSeq) const;

	/// Compute the centroid of this geometry as a coordinate.
	/// Returns false if centroid cannot be computed(empty geometry)
	bool getCentroid(CityCoordinate& coord) const;

	/// Compute the centroid of this geometry as a xy coordinate.
	/// Returns false if centroid cannot be computed(empty geometry)
	bool getCentroid(double& x, double& y) const;

	/// Compute the centroid of this geometry and returns geometry of point type.
	CityGeometry* getCentroid() const;

	/// Returns true if other.within(this) returns true.
	bool contains(const CityGeometry* g) const {
		return geosGeom->contains(g->getGEOSgeometry());
	}

	/**
	 * Tests whether this geometry crosses the specified geometry.
	 *
	 * The crosses predicate has the following equivalent definitions:
	 *
	 *   - The geometries have some but not all interior points in common.
	 *   - The DE-9IM Intersection Matrix for the two geometries matches
	 *     [T*T******] (for P/L, P/A, and L/A situations)
	 *     [T*****T**] (for L/P, A/P, and A/L situations)
	 *     [0********] (for L/L situations) For any other combination of dimensions this predicate returns false.
	 * The SFS defined this predicate only for P/L, P/A, L/L, and L/A situations.
	 * JTS extends the definition to apply to L/P, A/P and A/L situations as well,
	 * in order to make the relation symmetric.
	 */
	bool crosses(const CityGeometry* g) const {
		return geosGeom->crosses(g->getGEOSgeometry());
	}

	/**
	 * Tests whether this geometry is disjoint from the specified geometry.
	 *
	 * The disjoint predicate has the following equivalent definitions:
	 *
	 *   - The two geometries have no point in common
	 *   - The DE-9IM Intersection Matrix for the two geometries matches [FF*FF****]
	 *   - ! g.intersects(this) (disjoint is the inverse of intersects)
	 */
	bool disjoint(const CityGeometry* g) const {
		return geosGeom->disjoint(g->getGEOSgeometry());
	}

	/// Returns true if disjoint returns false.
	bool intersects(const CityGeometry* g) const {
		return geosGeom->intersects(g->getGEOSgeometry());
	}

	/// Returns true if the DE-9IM intersection matrix for the two Geometrys is
	/// T*T***T** (for two points or two surfaces) 1*T***T** (for two curves)
	bool overlaps(const CityGeometry* g) const {
		return geosGeom->overlaps(g->getGEOSgeometry());
	}

	/// Returns true if the DE-9IM intersection matrix for the two Geometrys is
	/// FT*******, F**T***** or F***T****.
	bool touches(const CityGeometry* g) const {
		return geosGeom->touches(g->getGEOSgeometry());
	}

	/// Returns true if the DE-9IM intersection matrix for the two Geometrys is T*F**F***.
	bool within(const CityGeometry* g) const {
		return geosGeom->within(g->getGEOSgeometry());
	}
};

} /* namespace dtsim */

#endif /* CITY_GEOMETRY_H */
