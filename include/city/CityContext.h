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
 * CityContext.h
 *
 * $Revision: 865 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_CONTEXT_H
#define CITY_CONTEXT_H

#include <boost/mpi/communicator.hpp>
#include <string>
#include <vector>
#include <unordered_map>

#include "repast_hpc/SharedContext.h"
#include "repast_hpc/Projection.h"
#include "repast_hpc/AgentId.h"
#include "City.h"
#include "CityAgent.h"
#include "CityAgentFactory.h"

namespace dtsim {

class CitySpace;

/**
 * \class CityContext
 *
 * \brief
 * Collection of city agents. CityContext is used internally.
 */
class CityContext {
private:
	repast::SharedContext<CityAgent>* repastContext;

	typedef typename std::unordered_map<std::string, CitySpace*> SpaceMap;
	typedef typename SpaceMap::iterator SpaceMapIterator;
	SpaceMap spaceMap;

public:
	typedef typename repast::Context<CityAgent>::const_iterator AgentMapConstIterator;

	CityContext(boost::mpi::communicator* comm);
	virtual ~CityContext();

	/// Non User API
	void addRepastProjection(repast::Projection<CityAgent>* space);

	/// Non User API
	repast::Projection<CityAgent>* getRepastProjection(const std::string& name);

	/// Adds space to the context
	void addSpace(CitySpace* space);

	/// Returns space from this context
	CitySpace* getSpace(std::string name);

	/**
	 * Gets the start of iterator over the agents in this context
	 * The iterator dereferences into shared_ptr<CityAgent>.
	 * The actual agent pointer can be accessed by iter->get().
	 *
	 * @return the start of iterator over the agents in this context
	 */
	AgentMapConstIterator begin() const {
		return AgentMapConstIterator(repastContext->begin());
	}

	/**
	 * Gets the end of iterator over the agents in this context
	 * The iterator dereferences into shared_ptr<CityAgent>.
	 * The actual agent pointer can be accessed by iter->get().
	 *
	 * @return the end of iterator over the agents in this context
	 */
	AgentMapConstIterator end() const {
		return AgentMapConstIterator(repastContext->end());
	}

	template<typename T>
	T* addAgent(T* agent);

	template<typename T>
	void removeAgent(T* agent);

	/// Returns all agents in this context
	void getAllAgents(std::vector<CityAgent*>& agents);
};

/**
 * Adds the agent to the context
 *
 * @param agent agent to add
 * @return agent object pointer
 */
template<typename T>
T* CityContext::addAgent(T* agent)
{
	CityAgent* cityAgent = static_cast<CityAgent*>(agent);

	if (cityAgent->getAgentId() < 0) {
		int type = CityAgentFactory::instance()->getAgentType<T>();
		int id = CityAgentFactory::instance()->getAgentId<T>();
		int startProc = City::instance()->getCommunicator()->rank();
		int currentProc = City::instance()->getCommunicator()->rank();

		repast::AgentId agentId(id, startProc, type, currentProc);
		cityAgent->setId(agentId);
	}

	repastContext->addAgent(cityAgent);
	return agent;
}

/**
 * Removes the specified agent from this context
 *
 * @param agent agent to remove
 */
template<typename T>
void CityContext::removeAgent(T* agent)
{
	repastContext->removeAgent(static_cast<CityAgent*>(agent));
}

} /* namespace dtsim */

#endif /* CITY_CONTEXT_H */
