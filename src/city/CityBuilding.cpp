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
 * CityBuilding.cpp
 *
 * $Revision$
 * $LastChangedDate$
 */

#include "CityBuilding.h"

namespace dtsim {

CityBuilding::CityBuilding(double id, double type) : buildingId(id), buildingType(type)
{}

// tmp
CityBuilding::CityBuilding(double id, double type, std::string name) : 
	buildingId(id), buildingType(type), buildingName(name)
{}

std::unique_ptr<CityCoordinateSequence> CityBuilding::getCoordinates() const
{
	return getGeometry()->getCoordinates();
}

void CityBuilding::getCoordinates(CityCoordinateSequence& seq) const
{
	getGeometry()->getCoordinates(seq);
}

std::unique_ptr<CityCoordinate> CityBuilding::getCoordinate() const
{
	double x, y;
	getGeometry()->getCentroid(x, y);

	std::unique_ptr<CityCoordinate> coord(new CityCoordinate(x, y));
	return coord;
}

void CityBuilding::getCoordinate(CityCoordinate& c) const
{
	getGeometry()->getCentroid(c);
}

void CityBuilding::getCoordinate(double& x, double& y) const
{
	getGeometry()->getCentroid(x, y);
}

} /* namespace dtsim */
