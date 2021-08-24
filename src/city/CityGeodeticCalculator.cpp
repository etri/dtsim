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
 * CityGeodeticCalculator.cpp
 *
 * $Revision: 625 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <boost/math/constants/constants.hpp>
#include <string>
#include <vector>
#include <cmath>
#include "City.h"
#include "CityGeodeticCalculator.h"
#include "CityUtils.h"

namespace dtsim {

const double CityGeodeticCalculator::WGS84_a = 6378137;
const double CityGeodeticCalculator::WGS84_b = 6356752.314245;
const double CityGeodeticCalculator::WGS84_f = 1/298.257223563;
const double CityGeodeticCalculator::RADIUS_EARTH = 6371000.0;

CityGeodeticCalculator::CityGeodeticCalculator()
{
	if (City::instance()->containProperty("geodetic.distance.formula")) {
		std::string formulaStr = City::instance()->getStringProperty("geodetic.distance.formula");

		if ((formulaStr.compare("haversine") == 0) || (formulaStr.compare("Haversine") == 0))
			formula = GEODETIC_HAVERSINE;
		else if ((formulaStr.compare("vincenty") == 0) || (formulaStr.compare("Vincenty") == 0))
			formula = GEODETIC_VINCENTY;
		else
			formula = GEODETIC_HAVERSINE;
	} else {
		formula = GEODETIC_HAVERSINE;
	}
}

CityGeodeticCalculator::~CityGeodeticCalculator()
{
	_instance = 0;
}

CityGeodeticCalculator* CityGeodeticCalculator::_instance = 0;

CityGeodeticCalculator* CityGeodeticCalculator::instance()
{
	if (_instance == 0)
		_instance = new CityGeodeticCalculator();

	return _instance;
}

void CityGeodeticCalculator::haversine_getDestinationPoint(double startLon, double startLat,
														   double distance, double azimuth,
														   double& destLon, double& destLat)
{
	double delta = distance / RADIUS_EARTH;
	double theta = degreesToRadians(azimuth);

	double lambda1 = degreesToRadians(startLon);
	double phi1 = degreesToRadians(startLat);

	double sinPhi2 = (std::sin(phi1) * std::cos(delta)) +
					 (std::cos(phi1) * std::sin(delta) * std::cos(theta));
	double phi2 = std::asin(sinPhi2);

	double x = std::cos(delta) - (std::sin(phi1) * sinPhi2);
	double y = std::sin(theta) * std::sin(delta) * std::cos(phi1);
	double lambda2 = lambda1 + std::atan2(y, x);

	destLon = radiansToDegrees(lambda2);
	destLat = radiansToDegrees(phi2);
}

void CityGeodeticCalculator::vincenty_getDestinationPoint(double startLon, double startLat,
														  double distance, double azimuth,
														  double& destLon, double& destLat)
{
	double lambda1 = degreesToRadians(startLon);
	double phi1 = degreesToRadians(startLat);
	double alpha1 = degreesToRadians(azimuth);
	double s = distance;

	double sinAlpha1 = std::sin(alpha1);
	double cosAlpha1 = std::cos(alpha1);

	double tanU1 = (1 - WGS84_f) * std::tan(phi1);
	double cosU1 = 1 / std::sqrt((1 + tanU1*tanU1));
	double sinU1 = tanU1 * cosU1;

	double sigma1 = std::atan2(tanU1, cosAlpha1);
	double sinAlpha = cosU1 * sinAlpha1;
	double cosSqAlpha = 1 - sinAlpha*sinAlpha;
	double uSq = cosSqAlpha * (WGS84_a*WGS84_a - WGS84_b*WGS84_b) / (WGS84_b*WGS84_b);
	double A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
	double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));

	double cos2SigmaM, sinSigma, cosSigma, deltaSigma;

	double sigma = s / (WGS84_b*A);
	double sigmaP;
	int iterations = 0;
	do {
		cos2SigmaM = std::cos(2*sigma1 + sigma);
		sinSigma = std::sin(sigma);
		cosSigma = std::cos(sigma);
		deltaSigma = B * sinSigma * (cos2SigmaM + B/4*(cosSigma*(-1 + 2*cos2SigmaM*cos2SigmaM)-
					 B / 6*cos2SigmaM*(-3 + 4*sinSigma*sinSigma)*(-3 + 4*cos2SigmaM*cos2SigmaM)));
		sigmaP = sigma;
		sigma = s / (WGS84_b*A) + deltaSigma;
	} while ((std::abs(sigma - sigmaP) > 1e-12) && (++iterations < 1000));

	double chi = sinU1*sinSigma - cosU1*cosSigma*cosAlpha1;
	double phi2 = std::atan2(sinU1*cosSigma + cosU1*sinSigma*cosAlpha1, (1-WGS84_f)*std::sqrt(sinAlpha*sinAlpha + chi*chi));
	double lambda = std::atan2(sinSigma*sinAlpha1, cosU1*cosSigma - sinU1*sinSigma*cosAlpha1);
	double C = WGS84_f/16*cosSqAlpha*(4 + WGS84_f*(4 - 3*cosSqAlpha));
	double L = lambda - (1-C) * WGS84_f * sinAlpha * (sigma + C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));
	double lambda2 = lambda1 + L;

	double alpha2 = std::atan2(sinAlpha, -chi);

	destLon = radiansToDegrees(lambda2);
	destLat = radiansToDegrees(phi2);
}

void CityGeodeticCalculator::getDestinationPoint(double startLon, double startLat,
												 double distance, double azimuth,
												 double& destLon, double& destLat)
{
	if (formula == GEODETIC_HAVERSINE)
		haversine_getDestinationPoint(startLon, startLat, distance, azimuth, destLon, destLat);
	else
		vincenty_getDestinationPoint(startLon, startLat, distance, azimuth, destLon, destLat);
}

void CityGeodeticCalculator::getDestinationPoint(CityCoordinate& startPoint,
												 double distance, double azimuth,
												 CityCoordinate& destPoint)
{
	getDestinationPoint(startPoint.x, startPoint.y, distance, azimuth, destPoint.x, destPoint.y);
}

void CityGeodeticCalculator::getMidPoint(double startLon, double startLat,
										 double destLon, double destLat,
										 double& midLon, double& midLat)
{
	double lambda1 = degreesToRadians(startLon);
	double phi1 = degreesToRadians(startLat);
	double phi2 = degreesToRadians(destLat);

	double deltaLambda = degreesToRadians(destLon - startLon);

	std::vector<double> A;
	std::vector<double> B;
	std::vector<double> C;
	double x, y, z;

	x = std::cos(phi1);
	y = 0;
	z = std::sin(phi1);
	A.push_back(x);
	A.push_back(y);
	A.push_back(z);

	x = std::cos(phi2) * std::cos(deltaLambda);
	y = std::cos(phi2) * std::sin(deltaLambda);
	z = std::sin(phi2);
	B.push_back(x);
	B.push_back(y);
	B.push_back(z);

	x = A[0] + B[0];
	y = A[1] + B[1];
	z = A[2] + B[2];
	C.push_back(x);
	C.push_back(y);
	C.push_back(z);

	double phiM = std::atan2(C[2], std::sqrt((C[0]*C[0]) + (C[1]*C[1])));
	double lambdaM = lambda1 + std::atan2(C[1], C[0]);

	midLon = radiansToDegrees(lambdaM);
	midLat = radiansToDegrees(phiM);
}

void CityGeodeticCalculator::getMidPoint(CityCoordinate& startPoint, CityCoordinate& destPoint, 
										 CityCoordinate& midPoint)
{
	getMidPoint(startPoint.x, startPoint.y, destPoint.x, destPoint.y, midPoint.x, midPoint.y);
}

void CityGeodeticCalculator::getIntermediatePoint(double startLon, double startLat, double fraction,
												  double destLon, double destLat,
												  double& intermediateLon, double& intermediateLat)
{
	if ((startLon == destLon) && (startLat == destLat)) {
		intermediateLon = startLon;
		intermediateLat = startLat;
		return;
	}

	double lambda1 = degreesToRadians(startLon);
	double phi1 = degreesToRadians(startLat);
	double lambda2 = degreesToRadians(destLon);
	double phi2 = degreesToRadians(destLat);

	double deltaPhi = phi2 - phi1;
	double deltaLambda = lambda2 - lambda1;

	double a = (std::sin(deltaPhi*0.5)*std::sin(deltaPhi*0.5)) +
			   (std::cos(phi1)*std::cos(phi2)*std::sin(deltaLambda*0.5)*std::sin(deltaLambda*0.5));
	double delta = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

	double A = std::sin((1-fraction)*delta) / std::sin(delta);
	double B = std::sin(fraction*delta) / std::sin(delta);

	double x = (A * std::cos(phi1) * std::cos(lambda1)) + (B * std::cos(phi2) * std::cos(lambda2));
	double y = (A * std::cos(phi1) * std::sin(lambda1)) + (B * std::cos(phi2) * std::sin(lambda2));
	double z = (A * std::sin(phi1)) + (B * std::sin(phi2));

	double phi3 = std::atan2(z, std::sqrt((x*x) + (y*y)));
	double lambda3 = std::atan2(y, x);

	intermediateLon = radiansToDegrees(lambda3);
	intermediateLat = radiansToDegrees(phi3);
}

void CityGeodeticCalculator::getIntermediatePoint(CityCoordinate& startPoint, double fraction,
												  CityCoordinate& destPoint,
												  CityCoordinate& intermediatePoint)
{
	getIntermediatePoint(startPoint.x, startPoint.y, fraction, destPoint.x, destPoint.y,
						 intermediatePoint.x, intermediatePoint.y);
}

double CityGeodeticCalculator::haversine_getDistance(double startLon, double startLat,
													 double destLon, double destLat)
{
	double lambda1 = degreesToRadians(startLon);
	double phi1 = degreesToRadians(startLat);
	double lambda2 = degreesToRadians(destLon);
	double phi2 = degreesToRadians(destLat);

	double deltaPhi = phi2 - phi1;
	double deltaLambda = lambda2 - lambda1;

	double a = (std::sin(deltaPhi*0.5) * std::sin(deltaPhi*0.5)) +
			   (std::cos(phi1)*std::cos(phi2)*std::sin(deltaLambda*0.5)*std::sin(deltaLambda*0.5));
	double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1-a));

	return RADIUS_EARTH * c;
}

double CityGeodeticCalculator::vincenty_getDistance(double startLon, double startLat,
													double destLon, double destLat, double& azimuth)
{
	double lambda1 = degreesToRadians(startLon);
	double phi1 = degreesToRadians(startLat);
	double lambda2 = degreesToRadians(destLon);
	double phi2 = degreesToRadians(destLat);

	double L = lambda2 - lambda1;
	double tanU1 = (1 - WGS84_f) * std::tan(phi1);
	double cosU1 = 1 / std::sqrt((1 + tanU1*tanU1));
	double sinU1 = tanU1 * cosU1;
	double tanU2 = (1 - WGS84_f) * std::tan(phi2);
	double cosU2 = 1 / std::sqrt((1 + tanU2*tanU2));
	double sinU2 = tanU2 * cosU2;

	double sinLambda, cosLambda, sinSigma = 0, cosSigma = 0, sinAlpha;
	double sinSqSigma, cosSqAlpha = 0, cos2SigmaM = 0, sigma = 0, C;

	double lambda = L;
	double lambdaP;
	int iterations = 0;

	bool antimeridian = std::abs(L) > boost::math::constants::pi<double>();

	do {
		sinLambda = std::sin(lambda);
		cosLambda = std::cos(lambda);

		sinSqSigma = (cosU2*sinLambda)*(cosU2*sinLambda) +
					 (cosU1*sinU2 - sinU1*cosU2*cosLambda) *
					 (cosU1*sinU2 - sinU1*cosU2*cosLambda);

		if (std::abs(sinSqSigma) < std::numeric_limits<double>::epsilon())
			break;

		sinSigma = std::sqrt(sinSqSigma);
		cosSigma = sinU1*sinU2 + cosU1*cosU2*cosLambda;

		sigma = std::atan2(sinSigma, cosSigma);
		sinAlpha = cosU1*cosU2*sinLambda / sinSigma;
		cosSqAlpha = 1 - sinAlpha*sinAlpha;
		cos2SigmaM = (cosSqAlpha != 0) ? (cosSigma - 2*sinU1*sinU2/cosSqAlpha) : 0;

		C = WGS84_f/16*cosSqAlpha*(4 + WGS84_f*(4 - 3*cosSqAlpha));
		lambdaP = lambda;
		lambda = L + (1-C)*WGS84_f*sinAlpha*(sigma + C*sinSigma*(cos2SigmaM + C*cosSigma*(-1 + 2*cos2SigmaM*cos2SigmaM)));
	} while ((std::abs(lambda - lambdaP) > 1e-12) && (++iterations < 1000));

	double uSq = cosSqAlpha * (WGS84_a*WGS84_a - WGS84_b*WGS84_b) / (WGS84_b*WGS84_b);
	double A = 1 + uSq/16384*(4096 + uSq*(-768 + uSq*(320  -175*uSq)));
	double B = uSq/1024 * (256 + uSq*(-128 + uSq*(74 - 47*uSq)));
	double deltaSigma = B*sinSigma*(cos2SigmaM + B/4*(cosSigma*(-1 + 2*cos2SigmaM*cos2SigmaM) -
						B/6*cos2SigmaM*(-3 + 4*sinSigma*sinSigma)*(-3 + 4*cos2SigmaM*cos2SigmaM)));

	double distance = WGS84_b*A*(sigma - deltaSigma);

	double alpha = std::atan2(cosU2*sinLambda, cosU1*sinU2 - sinU1*cosU2*cosLambda);
	double degrees = radiansToDegrees(alpha);
	if ((0 > degrees) || (degrees >= 360))
		degrees = std::fmod((std::fmod(degrees, 360.0) + 360.0), 360.0);

	azimuth = std::abs(distance) < std::numeric_limits<double>::epsilon() ? NAN : degrees;

	return distance;
}

double CityGeodeticCalculator::getDistance(double startLon, double startLat,
										   double destLon, double destLat)
{
	if (formula == GEODETIC_HAVERSINE) {
		return haversine_getDistance(startLon, startLat, destLon, destLat);
	} else {
		double azimuth;
		return vincenty_getDistance(startLon, startLat, destLon, destLat, azimuth);
	}
}

double CityGeodeticCalculator::getDistance(CityCoordinate& startPoint, CityCoordinate& destPoint)
{
	return getDistance(startPoint.x, startPoint.y, destPoint.x, destPoint.y);
}

double CityGeodeticCalculator::haversine_getAzimuth(double startLon, double startLat,
													double destLon, double destLat)
{
	double phi1 = degreesToRadians(startLat);
	double phi2 = degreesToRadians(destLat);

	double deltaLambda = degreesToRadians(destLon - startLon);

	double x = (std::cos(phi1) * std::sin(phi2)) -
			   (std::sin(phi1) * std::cos(phi2) * std::cos(deltaLambda));
	double y = std::sin(deltaLambda) * std::cos(phi2);
	double theta = std::atan2(y, x);

	double azimuth = radiansToDegrees(theta);

	if ((0 > azimuth) || (azimuth >= 360))
		azimuth = std::fmod((std::fmod(azimuth, 360.0) + 360.0), 360.0);

	return azimuth;
}

double CityGeodeticCalculator::vincenty_getAzimuth(double startLon, double startLat,
												   double destLon, double destLat)
{
	double azimuth;
	vincenty_getDistance(startLon, startLat, destLon, destLat, azimuth);

	return azimuth;
}

double CityGeodeticCalculator::getAzimuth(double startLon, double startLat,
										  double destLon, double destLat)
{
	if ((startLon == destLon) && (startLat == destLat))
		return NAN;

	if (formula == GEODETIC_HAVERSINE)
		return haversine_getAzimuth(startLon, startLat, destLon, destLat);
	else
		return vincenty_getAzimuth(startLon, startLat, destLon, destLat);
}

double CityGeodeticCalculator::getAzimuth(CityCoordinate& startPoint, CityCoordinate& destPoint)
{
	return getAzimuth(startPoint.x, startPoint.y, destPoint.x, destPoint.y);
}

} /* namespace dtsim */
