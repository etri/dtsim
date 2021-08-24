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
 * CityGridSpace.cpp
 *
 * $Revision: 664 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "City.h"
#include "CityGridSpace.h"
#include "CityException.h"

namespace dtsim {

CityGridSpaceAdder::CityGridSpaceAdder() : grid(0), center(0.0, 0.0)
{}

CityGridSpaceAdder::~CityGridSpaceAdder()
{}

void CityGridSpaceAdder::init(repast::GridDimensions gd, repast::Grid<CityAgent, double>* grid_)
{
	double x = gd.origin(0) + gd.extents(0) / 2.0;
	double y = gd.origin(1) + gd.extents(1) / 2.0;
	center = repast::Point<double>(x, y);
	grid = grid_;
}

bool CityGridSpaceAdder::add(boost::shared_ptr<CityAgent> agent)
{
	return grid->moveTo(agent->getId(), center);
}

CityGridSpace::CityGridSpace(std::string& name, CityContext* context)
: CitySpace(name, context)
{
	try {
		if (!City::instance()->containProperty("grid.min.x"))
			throw CityException(__func__, "not found 'grid.min.x' in property file");
		if (!City::instance()->containProperty("grid.min.y"))
			throw CityException(__func__, "not found 'grid.min.y' in property file");
		if (!City::instance()->containProperty("grid.max.x"))
			throw CityException(__func__, "not found 'grid.max.x' in property file");
		if (!City::instance()->containProperty("grid.proc.per.x"))
			throw CityException(__func__, "not found 'grid.proc.per.x' in property file");
		if (!City::instance()->containProperty("grid.proc.per.y"))
			throw CityException(__func__, "not found 'grid.proc.per.y' in property file");
		if (!City::instance()->containProperty("grid.buffer"))
			throw CityException(__func__, "not found 'grid.buffer' in property file");

		double minX = City::instance()->getDoubleProperty("grid.min.x");
		double minY = City::instance()->getDoubleProperty("grid.min.y");
		double maxX = City::instance()->getDoubleProperty("grid.max.x");
		double maxY = City::instance()->getDoubleProperty("grid.max.y");
		int procPerX = City::instance()->getIntProperty("grid.proc.per.x");
		int procPerY = City::instance()->getIntProperty("grid.proc.per.y");
		buffer = City::instance()->getIntProperty("grid.buffer");

		repast::Point<double> originPoint(minX, minY);
		repast::Point<double> extentPoint(maxX-minX+1, maxY-minY+1);
		repast::GridDimensions dims(originPoint, extentPoint);
		std::vector<int> procDims = {procPerX, procPerY};
		boost::mpi::communicator* comm = City::instance()->getCommunicator();

		std::vector<double> origin = {minX, minY};
		std::vector<double> extent = {maxX-minX+1, maxY-minY+1};
		gridDims = {origin, extent};

		gridSpace = new repast::SharedContinuousSpace<CityAgent, repast::StickyBorders, CityGridSpaceAdder>(name, dims, procDims, buffer, comm);

		context->addRepastProjection(gridSpace);
	} catch (CityException& e) {
		throw e;
	}
}

CityGridSpace::~CityGridSpace()
{}

double CityGridSpace::getDistance(const std::vector<double>& point1, const std::vector<double>& point2) const
{
	repast::Point<double> pt1(point1);
	repast::Point<double> pt2(point2);

	return gridSpace->getDistance(pt1, pt2);
}

double CityGridSpace::getDistanceSq(const std::vector<double>& point1, const std::vector<double>& point2) const
{
	repast::Point<double> pt1(point1);
	repast::Point<double> pt2(point2);

	return gridSpace->getDistanceSq(pt1, pt2);
}

} /* namespace dtsim */
