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
 * CityNetworkSpace.cpp
 *
 * $Revision: 664 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "City.h"
#include "CityNetworkSpace.h"

namespace dtsim {

/**
 * CityLink
 */
CityLink::CityLink()
: repast::RepastEdge<CityAgent>()
{}

CityLink::CityLink(CityAgent* source, CityAgent* target)
: repast::RepastEdge<CityAgent>(source, target)
{}

CityLink::CityLink(CityAgent* source, CityAgent* target, double weight)
: repast::RepastEdge<CityAgent>(source, target, weight)
{}

CityLink::CityLink(boost::shared_ptr<CityAgent> source, boost::shared_ptr<CityAgent> target)
: repast::RepastEdge<CityAgent>(source, target)
{}

CityLink::CityLink(boost::shared_ptr<CityAgent> source, boost::shared_ptr<CityAgent> target, double weight)
: repast::RepastEdge<CityAgent>(source, target, weight)
{}

CityLink::CityLink(const CityLink& edge)
: repast::RepastEdge<CityAgent>(edge)
{}

CityLink::~CityLink()
{}


/**
 * CityLinkContent
 */
CityLinkContent::CityLinkContent()
{}

CityLinkContent::CityLinkContent(CityLink* link)
: repast::RepastEdgeContent<CityAgent>(link)
{}

CityLinkContent::~CityLinkContent()
{}


/**
 * CityLinkContentManager
 */
CityLinkContentManager::CityLinkContentManager()
{}

CityLinkContentManager::~CityLinkContentManager()
{}

CityLink* CityLinkContentManager::createEdge(CityLinkContent& content, repast::Context<CityAgent>* context)
{
	return new CityLink(context->getAgent(content.source), context->getAgent(content.target), content.weight);
}

CityLinkContent* CityLinkContentManager::provideEdgeContent(CityLink* edge)
{
	return new CityLinkContent(edge);
}


/**
 * CityNetworkSpace
 */
CityNetworkSpace::CityNetworkSpace(std::string& name, CityContext* context)
: CitySpace(name, context)
{
	directed = false;
	lcm = new CityLinkContentManager();

	networkSpace = new repast::SharedNetwork<CityAgent, CityLink, CityLinkContent, CityLinkContentManager>(name, directed, lcm);

	context->addRepastProjection(networkSpace);
}

CityNetworkSpace::~CityNetworkSpace()
{
	delete lcm;
}

void CityNetworkSpace::showEdges()
{
	networkSpace->showEdges();
}

int CityNetworkSpace::edgeCount() const
{
	return networkSpace->edgeCount();
}

int CityNetworkSpace::vertexCount() const
{
	return networkSpace->vertexCount();
}

} /* namespace dtsim */
