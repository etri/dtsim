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

/* 
 * CityRouter.h
 *
 * $Revision: $
 * $LastChangedDate: $
 */

#ifndef CITY_ROUTER_H_
#define CITY_ROUTER_H_

#include <string>
#include <vector>
#include <unordered_map>

#include "CityRouteTopology.h"
#include "CityException.h"

namespace dtsim {

/**
 * \brief
 * CityRouter is a common object to perform all algorithms.
 *
 * It defines the template variables that make up the routing network.
 * This object will be created as many as the number of dynamic agents define in the model.
 *
 * Here, the following search algorithms are supported: 
 *   - A* search algorithm
 *   - BFS(Best-First Serach) search algorithm
 *   - Dijkstra's search algorithm
 *   - BiDijkstra(Bidirectional Dijkstra)'s search algorithm
 *
 * @tparam V the vertex type in this route network
 * @tparam E the edge type in this route network
 */
template<typename V, typename E>
class CityRouter {
private:
	CityRouteTopology<V, E>* routeTopology;

public:
	/**
	 * Creates a CityRouter
	 * It is called within the CityRouteNetwork constructor
	 *
	 * @param nodes the vertices map
	 * @param search_algorithm the specified search algorithm
	 */
	CityRouter(std::unordered_map<double, V*>& nodes, std::string searchType);

	~CityRouter();

	/// Calls the getShortestPath() function of each algorithm
	double getShortestPath(double start, double goal, std::vector<double>& out);

	void updateEdge(double edgeId, double newWeight);
};

template<typename V, typename E>
CityRouter<V, E>::CityRouter(std::unordered_map<double, V*>& nodes, std::string searchType)
: routeTopology(nullptr)
{
	if (searchType.compare("astar") == 0)
		routeTopology = new AStar<V, E>(nodes, -1, -1);
	else if (searchType.compare("bfs") == 0)
		routeTopology = new BFS<V, E>(nodes, -1, -1);
	else if (searchType.compare("dijkstra") == 0)
		routeTopology = new Dijkstra<V, E>(nodes, -1, -1);
	else if (searchType.compare("bidijkstra") == 0)
		routeTopology = new BiDijkstra<V, E>(nodes, -1, -1);
	else
		throw CityException(__func__, "not supported search type");
}

template<typename V, typename E>
CityRouter<V, E>::~CityRouter() 
{
	delete routeTopology;
}

template<typename V, typename E>
double CityRouter<V, E>::getShortestPath(double start, double goal, std::vector<double>& out)
{
	return routeTopology->getShortestPath(start, goal, out);
}

template<typename V, typename E>
void CityRouter<V, E>::updateEdge(double edgeId, double newWeight)
{
	routeTopology->updateEdge(edgeId, newWeight);
}

} /* namespace dtsim */

#endif /* CITY_ROUTER_H_ */
