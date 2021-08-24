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
 * CityContext.cpp
 *
 * $Revision: 865 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "CityContext.h"
#include "CitySpace.h"
#include "CityException.h"

namespace dtsim {

CityContext::CityContext(boost::mpi::communicator* comm)
{
	repastContext = new repast::SharedContext<CityAgent>(comm);
}

CityContext::~CityContext()
{
	SpaceMapIterator iter = spaceMap.begin();
	while (iter != spaceMap.end()) {
		CitySpace* space = iter->second;
		iter = spaceMap.erase(iter);
		delete space;
	}

	delete repastContext;
}

void CityContext::addRepastProjection(repast::Projection<CityAgent>* space)
{
	repastContext->addProjection(space);
}

repast::Projection<CityAgent>* CityContext::getRepastProjection(const std::string& name)
{
	return repastContext->getProjection(name);
}

void CityContext::addSpace(CitySpace* space)
{
	std::string name = space->getName();

	SpaceMapIterator iter = spaceMap.find(name);
	if (iter != spaceMap.end())
		throw CityException(__func__, "space already exist");

	spaceMap[name]= space;
}

CitySpace* CityContext::getSpace(std::string name)
{
	SpaceMapIterator iter = spaceMap.find(name);
	if (iter != spaceMap.end())
		return iter->second;
	else
		return nullptr;
}

void CityContext::getAllAgents(std::vector<CityAgent*>& agents)
{
	AgentMapConstIterator iter = begin();
	while (iter != end()) {
		agents.push_back(iter->get());
		iter++;
	}
}

} /* namespace dtsim */
