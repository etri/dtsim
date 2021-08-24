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
 * CityGridSpace.h
 *
 * $Revision: 664 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_GRID_SPACE_H
#define CITY_GRID_SPACE_H

#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include "repast_hpc/Grid.h"
#include "repast_hpc/SharedContinuousSpace.h"
#include "repast_hpc/GridDimensions.h"
#include "repast_hpc/GridComponents.h"
#include "repast_hpc/MultipleOccupancy.h"
#include "repast_hpc/AgentId.h"
#include "CityContext.h"
#include "CitySpace.h"
#include "CityGridDimensions.h"
#include "CityAgent.h"

namespace dtsim {

class City;

class CityGridSpaceAdder {
private:
	repast::Grid<CityAgent, double>* grid;
	repast::Point<double> center;

public:
	CityGridSpaceAdder();
	virtual ~CityGridSpaceAdder();

	void init(repast::GridDimensions gd, repast::Grid<CityAgent, double>* grid_);
	bool add(boost::shared_ptr<CityAgent> agent);
};

/**
 * \class CityGridSpace
 *
 * \brief
 * Grid Space implementation
 */
class CityGridSpace: public CitySpace {
private:
	/// RepastHPC Continuous SharedBaseGrid space
	repast::SharedContinuousSpace<CityAgent, repast::StickyBorders, CityGridSpaceAdder>* gridSpace;

	/// Dimensions of the entire pan-process grid
	CityGridDimensions gridDims;

	/// Size of the buffer between this part of the pan-process grid and its neighbors
	int buffer;

public:
	/**
	 * Creates a CityGridSpace
	 *
	 * @param name grid space name
	 * @param context CityContext
	 */
	CityGridSpace(std::string& name, CityContext* context);
	virtual ~CityGridSpace();

	/// Add agent to grid space
	template<typename T>
	T* addAgent(T* agent);

	/// Remove agent from grid space
	template<typename T>
	void removeAgent(T* agent);

	template<typename T>
	void moveAgent(const T* agent, const std::vector<double>& location);

	template<typename T>
	void moveAgentByDisplacement(const T* agent, const std::vector<double>& displacement);

	template<typename T>
	void moveAgentByVector(const T* agent, double distance, const std::vector<double>& anglesInRadians);

	template<typename T>
	bool contains(const T* agent);

	template<typename T>
	T* getAgentAt(const std::vector<double>& point) const;

	template<typename T>
	void getAgentsAt(const std::vector<double>& point, std::vector<T*>& out) const;

	template<typename T>
	void getAgentsWithin(const std::vector<double> point1, const std::vector<double> point2, std::vector<T*>& agents) const;

	template<typename T>
	void getAllAgents(std::vector<T*>& agents) const;

	template<typename T>
	bool getLocation(const T* agent, std::vector<double>& location) const;

	/// Gets the distance between the two grid points
	double getDistance(const std::vector<double>& point1, const std::vector<double>& point2) const;

	/// Gets the square of the distance between the two grid points
	double getDistanceSq(const std::vector<double>& point1, const std::vector<double>& point2) const;

	/// Gets the dimensions of this grid space
	CityGridDimensions getDimensions() const {
		return gridDims;
	}

	/// Gets the number of agents in this grid space
	size_t getSize() const {
		return gridSpace->size();
	}
};

template<typename T>
T* CityGridSpace::addAgent(T* agent)
{
	return context->addAgent(agent);
}

template<typename T>
void CityGridSpace::removeAgent(T* agent)
{
	context->removeAgent(agent);
}

/**
 * Moves the specified agent to the specified location
 *
 * @param agent the agent to move
 * @param location the location to move to
 */
template<typename T>
void CityGridSpace::moveAgent(const T* agent, const std::vector<double>& location)
{
	gridSpace->moveTo(agent->getId(), location);
}

/**
 * Moves the specified agent from its current location by the specified amount
 *
 * @param agent the agent to move
 * @param displacement the amount to move the agent
 */
template<typename T>
void CityGridSpace::moveAgentByDisplacement(const T* agent, const std::vector<double>& displacement)
{
	gridSpace->moveByDisplacement(agent, displacement);
}

/**
 * Moves the specified agent the specified distance from its current position along the specified angle
 *
 * @param agent the agent to move
 * @param distance the distance to move
 * @param anglesInRadians the angle to move along in radians
 */
template<typename T>
void CityGridSpace::moveAgentByVector(const T* agent, double distance, const std::vector<double>& anglesInRadians)
{
	gridSpace->moveByVector(agent, distance, anglesInRadians);
}

/**
 * Gets whether or not this grid contains the specified agent
 */
template<typename T>
bool CityGridSpace::contains(const T* agent)
{
	return gridSpace->contains(agent->getId());
}

/**
 * Gets the first agent found at the specified point,
 * or NUll if there is no such agent.
 */
template<typename T>
T* CityGridSpace::getAgentAt(const std::vector<double>& point) const
{
	repast::Point<double> pt(point);
	T* agent = gridSpace->getObjectAt(pt);

	return static_cast<T*>(agent);
}

/**
 * Gets all the agents found at the specified point
 */
template<typename T>
void CityGridSpace::getAgentsAt(const std::vector<double>& point, std::vector<T*>& out) const
{
	repast::Point<double> pt(point);
	gridSpace->getObjectsAt(pt, out);
}

/**
 * Gets the agents between point1 and point2 in this grid
 */
template<typename T>
void CityGridSpace::getAgentsWithin(std::vector<double> point1, std::vector<double> point2, std::vector<T*>& agents) const
{
	if (point1.size() != point2.size())
		throw CityException(__func__, "not equal dimensions");

	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	typename repast::BaseGrid<CityAgent, repast::MultipleOccupancy<CityAgent, double>, repast::StickyBorders, CityGridSpaceAdder, double>::const_iterator iter = gridSpace->begin();
	typename repast::BaseGrid<CityAgent, repast::MultipleOccupancy<CityAgent, double>, repast::StickyBorders, CityGridSpaceAdder, double>::const_iterator iterEnd = gridSpace->end();
	while (iter != iterEnd) {
		T* agent = iter->get();

		if (agent->getId().agentType() == type) {
			std::vector<double> location;
			getLocation(agent, location);

			if (point1.size() != location.size())
				throw CityException(__func__, "not equal dimensions");

			for (int i = 0, n = location.size(); i < n; i++) {
				double start = point1[i];
				double end = point2[i];
				if (point1[i] > point2[i]) {
					start = point2[i];
					end = point1[i];
				}

				if ((start >= location[i]) && (location[i] < end))
					agents.push_back(static_cast<T*>(agent));
			}
		}

		iter++;
	}
}

/**
 * Gets all agents in grid space
 *
 * @param [out] agents agents in grid space
 */
template<typename T>
void CityGridSpace::getAllAgents(std::vector<T*>& agents) const
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	typename repast::BaseGrid<CityAgent, repast::MultipleOccupancy<CityAgent, double>, repast::StickyBorders, CityGridSpaceAdder, double>::const_iterator iter = gridSpace->begin();
	typename repast::BaseGrid<CityAgent, repast::MultipleOccupancy<CityAgent, double>, repast::StickyBorders, CityGridSpaceAdder, double>::const_iterator iterEnd = gridSpace->end();
	while (iter != iterEnd) {
		if (iter->get()->getId().agentType() == type)
			agents.push_back(static_cast<T*>(iter->get()));

		iter++;
	}
}

/**
 * Gets the location of this agent and puts it in the specified vector
 *
 * @param agent the agent whose location we want to get
 * @param [out] location vector where the agents location will be put
 * @return true if the location was successfully found, otherwise false
 */
template<typename T>
bool CityGridSpace::getLocation(const T* agent, std::vector<double>& location) const
{
	return gridSpace->getLocation(agent->getId(), location);
}

} /* namespace dtsim */

#endif /* CITY_GRID_SPACE_H */
