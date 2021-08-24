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
 * CityRoad.cpp
 *
 * $Revision$
 * $LastChangedDate$
 */

#include "CityRoad.h"
#include "CityGeodeticCalculator.h"

namespace dtsim {

CityRoad::CityRoad(double roadId_, double startJunctionId_, double endJunctionId_, double lanes_, double maxSpeed_, double length_)
: roadId(roadId_), startJunctionId(startJunctionId_), endJunctionId(endJunctionId_), lanes(lanes_), maxSpeed(maxSpeed_), length(length_)
{}

std::unique_ptr<CityCoordinateSequence> CityRoad::getCoordinates() const
{
	return getGeometry()->getCoordinates();
}

void CityRoad::getCoordinates(CityCoordinateSequence& seq) const
{
	getGeometry()->getCoordinates(seq);
}

void CityRoad::makeDistances(const CityCoordinateSequence* seq)
{
	std::size_t count = seq->getSize() - 1;
	double x1, y1, x2, y2;
	double meter;

	for (std::size_t i = 0; i < count; i++) {
		seq->getAt(i, x1, y1);
		seq->getAt(i+1, x2, y2);

		meter = CityGeodeticCalculator::instance()->getDistance(x1, y1, x2, y2);
		distances.push_back(meter);
	}
}

} /* namespace dtsim */
