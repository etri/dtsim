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
 * CityGeodeticCalculator.h
 *
 * $Revision: 625 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_GEODETIC_CALCULATOR_H
#define CITY_GEODETIC_CALCULATOR_H

#include "CityCoordinate.h"

namespace dtsim {

/// Geodetic formula type
enum GeodeticFormula {
	GEODETIC_HAVERSINE = 1,
	GEODETIC_VINCENTY = 2
};

/**
 * \class CityGeodeticCalculator
 *
 * \brief
 * Calculates the distance, bearing and more between a pair of points specified
 * as latitude and longitude.
 *
 * This class supports Haversine and Vincenty formula. Which formula to use can be
 * specified in model.props file.
 * https://www.movable-type.co.uk/scripts/latlong.html
 * https://www.movable-type.co.uk/scripts/latlong-vincenty.html
 * The WGS-84 is a geocentric datum, based on ellipsoid with:
 *  - Semi-major axis      a = 6378137.0      metres
 *  - Semi-minor axis      b = 6356752.314245 metres
 *  - Inverse flattening 1/f = 298.257223563
 */
class CityGeodeticCalculator {
private:
	static const double WGS84_a;
	static const double WGS84_b;
	static const double WGS84_f;
	static const double RADIUS_EARTH;

	GeodeticFormula formula;

	void haversine_getDestinationPoint(double startLon, double startLat,
									   double distance, double azimuth,
									   double& destLon, double& destLat);
	void vincenty_getDestinationPoint(double startLon, double startLat,
									  double distance, double azimuth,
									  double& destLon, double& destLat);
	double haversine_getDistance(double startLon, double startLat,
								 double destLon, double destLat);
	double vincenty_getDistance(double startLon, double startLat,
								double destLon, double destLat, double& azimuth);
	double haversine_getAzimuth(double startLon, double startLat,
								double destLon, double destLat);
	double vincenty_getAzimuth(double startLon, double startLat,
							   double destLon, double destLat);

	static CityGeodeticCalculator* _instance;

public:
	CityGeodeticCalculator();
	virtual ~CityGeodeticCalculator();

	static CityGeodeticCalculator* instance();

	/**
	 * Returns the destination point from start point having travelled the given
	 * distance on the given azimuth
	 *
	 * @param startLon starting longitude
	 * @param startLat starting latitude
	 * @param distance
	 * @param azimuth
	 * @param destLon returned destination longitude
	 * @param destLat returned destination latitude
	 * @return destination longitude, latitude
	 */
	void getDestinationPoint(double startLon, double startLat, double distance, double azimuth,
							 double& destLon, double& destLat);

	/**
	 * Returns the destination point from start point having travelled the given
	 * distance on the given azimuth
	 *
	 * @param startPoint starting point(longitude, latitude) coordinate
	 * @param distance
	 * @param azimuth
	 * @param destPoint returned destination point(longitude, latitude) coordinate
	 * @return destination point(longitude, latitude)
	 */
	void getDestinationPoint(CityCoordinate& startPoint, double distance, double azimuth,
							 CityCoordinate& destPoint);

	/**
	 * Returns the midpoint between start point and destination point
	 *
	 * @param startLon starting longitude
	 * @param startLat starting latitude
	 * @param destLon destination longitude
	 * @param destLat destination latitude
	 * @param midLon returned longitude
	 * @param midLat returned latitude
	 * @return mid longitude and latitude
	 */
	void getMidPoint(double startLon, double startLat, double destLon, double destLat,
					 double& midLon, double& midLat);

	/**
	 * Returns the midpoint between start point and destination point
	 *
	 * @param startPoint starting point(longitude, latitude) coordinate
	 * @param destPoint destination point(longitude, latitude) coordinate
	 * @param midPoint return mid point(longitude, latitude) coordinate
	 * @return mid point(longitude, latitude)
	 */
	void getMidPoint(CityCoordinate& startPoint, CityCoordinate& destPoint,
					 CityCoordinate& midPoint);

	/**
	 * Returns the point at given fraction between start point and destination point
	 *
	 * @param startLon starting longitude
	 * @param startLat starting latitude
	 * @param fraction fraction between the two coordinates(0 = this point, 1 = specified point)
	 * @param destLon destination longitude
	 * @param destLat destination latitude
	 * @param intermediateLon returned intermediate longitude
	 * @param intermediateLat returned intermediate latitude
	 * @return intermediate longitude and latitude
	 */
	void getIntermediatePoint(double startLon, double startLat, double fraction,
							  double destLon, double destLat,
							  double& intermediateLon, double& intermediateLat);

	/**
	 * Returns the point at given fraction between start point and destination point
	 *
	 * @param startPoint starting point(longitude, latitude) coordinate
	 * @param fraction fraction between the two coordinates(0 = this point, 1 = specified point)
	 * @param destPoint destination point(longitude, latitude) coordinate
	 * @param intermediatePoint returned intermediate point(longitude, latitude) coordinate
	 * @return intermediate point between this point and destination point
	 */
	void getIntermediatePoint(CityCoordinate& startPoint, double fraction,
							  CityCoordinate& destPoint,
							  CityCoordinate& intermediatePoint);

	/**
	 * Returns the distance along the surface of the earth from start point and destination point
	 * Use haversine or vincenty formula
	 *
	 * @param startLon starting longitude
	 * @param startLat starting latitude
	 * @param destLon destination longitude
	 * @param destLat destination latitude
	 * @return distance(meters)
	 */
	double getDistance(double startLon, double startLat, double destLon, double destLat);

	/**
	 * Returns the distance along the surface of the earth from start point and destination point
	 * Use haversine or vincenty formula
	 *
	 * @param startPoint starting point(longitude, latitude) coordinate
	 * @param destPoint destination point(longitude, latitude) coordinate
	 * @return distance(meters)
	 */
	double getDistance(CityCoordinate& startPoint, CityCoordinate& destPoint);

	/**
	 * Returns the azimuth from start point and destination point
	 *
	 * @param startLon starting longitude
	 * @param startLat starting latitude
	 * @param destLon destination longitude
	 * @param destLat destination latitude
	 * @return azimuth(0 ~ 360 degrees)
	 */
	double getAzimuth(double startLon, double startLat, double destLon, double destLat);

	/**
	 * Returns the azimuth from start point and destination point
	 *
	 * @param startPoint starting point(longitude, latitude)
	 * @param destPoint destination point(longitude, latitude)
	 * @return azimuth(0 ~ 360 degrees)
	 */
	double getAzimuth(CityCoordinate& startPoint, CityCoordinate& destPoint);
};

} /* namespace dtsim */

#endif /* CITY_GEODETIC_CALCULATOR_H */
