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
 * CityNetworkSpace.h
 *
 * $Revision: 625 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_NETWORK_SPACE_H
#define CITY_NETWORK_SPACE_H

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include "repast_hpc/SharedNetwork.h"
#include "repast_hpc/Edge.h"
#include "CityContext.h"
#include "CitySpace.h"
#include "CityAgent.h"

namespace dtsim {

class CityGeographySpace;

/**
 * \class CityLink
 *
 * \brief
 * Network link implementation
 *
 * CityLink inherits RepastHPC RepastEdge of CityAgent
 */
class CityLink: public repast::RepastEdge<CityAgent> {
public:
	CityLink();
	virtual ~CityLink();

	CityLink(CityAgent* source, CityAgent* target);

	CityLink(CityAgent* source, CityAgent* target, double weight);

	CityLink(boost::shared_ptr<CityAgent> source, boost::shared_ptr<CityAgent> target);

	CityLink(boost::shared_ptr<CityAgent> source, boost::shared_ptr<CityAgent> target, double weight);

	CityLink(const CityLink& edge);
};

/**
 * \class CityLinkContent
 *
 * \brief
 * Serializable; also, does not include agent content, only agent IDs
 *
 * CityLinkContent inherits RepastHPC RepastEdgeContent of CityAgent
 */
class CityLinkContent: public repast::RepastEdgeContent<CityAgent> {
public:
	CityLinkContent();
	CityLinkContent(CityLink* link);
	virtual ~CityLinkContent();
};

/**
 * \class CityLinkContentManager
 *
 * \brief
 * Creates CityLink from CityLinkContent, and vice versa
 */
class CityLinkContentManager {
public:
	CityLinkContentManager();
	virtual ~CityLinkContentManager();

	CityLink* createEdge(CityLinkContent& content, repast::Context<CityAgent>* context);

	CityLinkContent* provideEdgeContent(CityLink* edge);
};


/**
 * \class CityNetworkSpace
 *
 * \brief
 * Network Space implementation
 */
class CityNetworkSpace: public CitySpace {
private:
	/// RepastHPC SharedNetwork space
	repast::SharedNetwork<CityAgent, CityLink, CityLinkContent, CityLinkContentManager>* networkSpace;

	/// Class that is capable of transforming a CityLink into CityLinkContent and vice versa
	CityLinkContentManager* lcm;

	/// if true the network will be directed, otherwise not
	bool directed;

public:
	/**
	 * Creates a CityNetworkSpace
	 *
	 * @param name network space name
	 * @param context CityContext
	 */
	CityNetworkSpace(std::string& name, CityContext* context);
	virtual ~CityNetworkSpace();

	template<typename T>
	void addLink(T* source, T* target);

	template<typename T>
	void addLink(T* source, T* target, double weight);

	template<typename T>
	void removeLink(T* source, T* target);

	template<typename T>
	boost::shared_ptr<CityLink> findLink(T* source, T* target);

	template<typename T>
	void successors(T* vertex, std::vector<T*>& out);

	template<typename T>
	void predecessors(T* vertex, std::vector<T*>& out);

	template<typename T>
	void adjacent(T* vertex, std::vector<T*>& out);

	template<typename T>
	int inDegree(T* vertex);

	template<typename T>
	int outDegree(T* vertex);

	template<typename T>
	void getVertices(std::vector<T*>& vertices);

	void showEdges();
	int edgeCount() const;
	int vertexCount() const;
};

template<typename T>
void CityNetworkSpace::addLink(T* source, T* target)
{
	CityLink* link = new CityLink(static_cast<CityAgent*>(source), static_cast<CityAgent*>(target), 1);
	boost::shared_ptr<CityLink> ptr = boost::shared_ptr<CityLink>(link);

	networkSpace->addEdge(ptr);
}

template<typename T>
void CityNetworkSpace::addLink(T* source, T* target, double weight)
{
	CityLink* link = new CityLink(static_cast<CityAgent*>(source), static_cast<CityAgent*>(target), weight);
	boost::shared_ptr<CityLink> ptr = boost::shared_ptr<CityLink>(link);

	networkSpace->addEdge(ptr);
}

template<typename T>
void CityNetworkSpace::removeLink(T* source, T* target)
{
	networkSpace->removeEdge(static_cast<CityAgent*>(source), static_cast<CityAgent*>(target));
}

template<typename T>
boost::shared_ptr<CityLink> CityNetworkSpace::findLink(T* source, T* target)
{
	return networkSpace->findEdge(static_cast<CityAgent*>(source), static_cast<CityAgent*>(target));
}

template<typename T>
void CityNetworkSpace::successors(T* vertex, std::vector<T*>& out)
{
	std::vector<CityAgent*> cityAgents;

	networkSpace->successors(static_cast<CityAgent*>(vertex), cityAgents);
	for (CityAgent* v : cityAgents)
		out.push_back(static_cast<T*>(v));
}

template<typename T>
void CityNetworkSpace::predecessors(T* vertex, std::vector<T*>& out)
{
	std::vector<CityAgent*> cityAgents;

	networkSpace->predecessors(static_cast<CityAgent*>(vertex), cityAgents);
	for (CityAgent* v : cityAgents)
		out.push_back(static_cast<T*>(v));
}

template<typename T>
void CityNetworkSpace::adjacent(T* vertex, std::vector<T*>& out)
{
	std::vector<CityAgent*> cityAgents;

	networkSpace->adjacent(static_cast<CityAgent*>(vertex), cityAgents);
	for (CityAgent* v : cityAgents)
		out.push_back(static_cast<T*>(v));
}

template<typename T>
int CityNetworkSpace::inDegree(T* vertex)
{
	return networkSpace->inDegree(static_cast<CityAgent*>(vertex));
}

template<typename T>
int CityNetworkSpace::outDegree(T* vertex)
{
	return networkSpace->outDegree(static_cast<CityAgent*>(vertex));
}

template<typename T>
void CityNetworkSpace::getVertices(std::vector<T*>& vertices)
{
	typename repast::Graph<CityAgent, CityLink, CityLinkContent, CityLinkContentManager>::vertex_iterator iter = networkSpace->verticesBegin();
	typename repast::Graph<CityAgent, CityLink, CityLinkContent, CityLinkContentManager>::vertex_iterator iterEnd = networkSpace->verticesEnd();

	while (iter != iterEnd) {
		vertices.push_back(static_cast<T*>(*iter));
		iter++;
	}
}

} /* namespace dtsim */

#endif /* CITY_NETWORK_SPACE_H */
