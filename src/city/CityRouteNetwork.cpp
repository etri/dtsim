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
 * CityRouteNetwork.cpp
 *
 *  $Revision:$
 *  $LastChangedDate:$
 */

#include <vector>
#include <unordered_map>
#include <algorithm>

#include "City.h"
#include "CityRouteNetwork.h"
#include "CityTimer.h"
#include "CityCoordinate.h"

namespace dtsim {

Vertex::Vertex(double nodeid, double latitude, double longitude)
: _nodeid(nodeid), _latitude(latitude), _longitude(longitude), _visited(false), _moved(false), _passed(false)
{}

void Vertex::getSuccessors(std::vector<Vertex*>& out) const
{
	for (Vertex* v : _successors)
		out.push_back(v);
}

void Vertex::getPredecessors(std::vector<Vertex*>& out) const
{
	for (Vertex* v : _predecessors)
		out.push_back(v);
}

Edge* Vertex::findEdge(Vertex* target) 
{
	for (Edge* e : _edges) {
		if (e->target()->nodeid() == target->nodeid())
			return e;
	}

	return nullptr;
}

void Vertex::doAddEdge(Edge* e) 
{
	_edges.push_back(e);

	if (e->source() == this) {
		putSuccessors(e->target());
	} else if (e->target() == this) {
		putPredecessors(e->source());
	}
}

Edge::Edge(Vertex* source, Vertex* target, double edgeId)
: _source(source), _target(target), _edgeid(edgeId), _weight(1), _visited(false)
{}

Edge::Edge(Vertex* source, Vertex* target, double edgeId, double weight)
: _source(source), _target(target), _edgeid(edgeId), _weight(weight), _visited(false)
{}

std::string CityRouteNetwork::DEFAULT_SEARCH_ALGORITHM = "astar";

/*
 * CityRouteNetwork Member Functions
 */
CityRouteNetwork::CityRouteNetwork(std::string& name, CityContext* context)
: CitySpace(name, context), cacheMap(nullptr)
{}

CityRouteNetwork::~CityRouteNetwork() 
{
	VerticesMapIterator vIter = vertices.begin();
	while (vIter != vertices.end()) {
		Vertex* v = vIter->second.first;
		vIter = vertices.erase(vIter);
		delete v;
	}

	std::vector<Edge*>::iterator eIter = edges.begin();
	while (eIter != edges.end()) {
		Edge* e = *eIter;
		eIter = edges.erase(eIter);
		delete e;
	}
	edgeMap.clear();

	delete cacheMap;

	std::vector<CityRouter<Vertex, Edge>*>::iterator rIter = routers.begin();
	while (rIter != routers.end()) {
		CityRouter<Vertex, Edge>* r = *rIter;
		rIter = routers.erase(rIter);
		delete r;
	}
}

void CityRouteNetwork::build(std::vector<CityJunction*>& junctions, std::vector<CityRoad*>& roads)
{
	std::unordered_map<double, Vertex*> out;
	Vertex* v;
	double junctionId;
	double x, y;
	for (CityJunction* junction : junctions) {
		junction->getCoordinate(x, y);
		junctionId = junction->getJunctionId();

		v = new Vertex(junctionId, x, y);

		vertices[junctionId] = std::make_pair(v, junction);
		out[junctionId] = v;
	}

	VerticesMapIterator iter;
	double startJunctionId, endJunctionId;
	Vertex* source;
	Vertex* target;
	Edge* e;
	for (CityRoad* road : roads) {
		startJunctionId = road->getStartJunctionId();
		endJunctionId = road->getEndJunctionId();
		if ((startJunctionId == 0) || (endJunctionId == 0))
			continue;

		iter = vertices.find(startJunctionId);
		if (iter == vertices.end())
			continue;
		source = iter->second.first;

		iter = vertices.find(endJunctionId);
		if (iter == vertices.end())
			continue;
		target = iter->second.first;

		e = source->findEdge(target);
		if (e == nullptr) {
			e = new Edge(source, target, road->getRoadId(), road->getLength());

			source->doAddEdge(e);
			target->doAddEdge(e);

			edges.push_back(e);
			edgeMap[road->getRoadId()] = e;
		}
	}

	std::cout << "The number of Junctions : " << vertices.size() << std::endl;
	std::cout << "The number of Roads : " << edgeMap.size() << std::endl;

	if (City::instance()->containProperty("routecache.file.path")) {
		std::string cacheFilePath = City::instance()->getStringProperty("routecache.file.path");

		std::fstream mfile(cacheFilePath, std::ios::in | std::ios::binary); 
		if (!mfile.fail()) { 
			mfile.close();

			cacheMap = new CityRouteCache(cacheFilePath);
		}
	}

	std::string search_algorithm;
	if (City::instance()->containProperty("pathfinding.algorithm"))
		search_algorithm = City::instance()->getStringProperty("pathfinding.algorithm");
	else
		search_algorithm = DEFAULT_SEARCH_ALGORITHM;

	int numRouters = 1;
	if (City::instance()->containProperty("schedule.worker.threads"))
		numRouters = City::instance()->getIntProperty("schedule.worker.threads");

	for (int i = 0; i < numRouters; i++) {
		CityRouter<Vertex, Edge>* r = new CityRouter<Vertex, Edge>(out, search_algorithm);
		routers.push_back(r);
		waitQ.push(r);
	}
}

void CityRouteNetwork::updateEdge(double edgeId, double newWeight)
{
	typename std::unordered_map<double, Edge*>::iterator eIter = edgeMap.find(edgeId);
	if (eIter != edgeMap.end()) {
		Edge* e = eIter->second;
		e->weight(newWeight); 
			
		for (CityRouter<Vertex, Edge>* r : routers) { 
			r->updateEdge(edgeId, newWeight); 
		} 

		if (cacheMap) {
			delete cacheMap;
			cacheMap = nullptr;
		}
	}
}

Edge* CityRouteNetwork::findEdge(CityJunction* source, CityJunction* target)
{ 
	VerticesMapIterator iter; 
	
	iter = vertices.find(source->getJunctionId()); 
	if (iter == vertices.end()) 
		return nullptr; 
	Vertex* start = iter->second.first; 
	
	iter = vertices.find(target->getJunctionId()); 
	if (iter == vertices.end()) 
		return nullptr; 
	Vertex* goal = iter->second.first; 
	
	return start->findEdge(goal); 
}

CityRouter<Vertex, Edge>* CityRouteNetwork::getRouter()
{
	std::lock_guard<std::mutex> guard(qLock);
	CityRouter<Vertex, Edge>* r = waitQ.front();
	waitQ.pop();
	return r;
}

void CityRouteNetwork::putRouter(CityRouter<Vertex, Edge>* r)
{
	std::lock_guard<std::mutex> guard(qLock);
	waitQ.push(r);
}

void CityRouteNetwork::printPath(double start, double goal, std::vector<CityJunction*>& paths)
{
	std::cout.precision(std::numeric_limits<double>::max_digits10);
	for (CityJunction* j : paths) {
		double nodeId = j->getJunctionId();
		if (nodeId == start) { 
       		std::cout << "" << nodeId << "-";
        } else if(nodeId == goal) {
       		std::cout << "" << nodeId;
        } else {
       		std::cout << "" << nodeId << "-";
        }
	}
	std::cout << std::endl;
}

void CityRouteNetwork::getShortestPath(double startJunctionId, double endJunctionId, std::vector<CityJunction*>& junctions)
{
	if (startJunctionId == endJunctionId)
		return;

	if ((vertices.find(startJunctionId) == vertices.end()) || (vertices.find(endJunctionId) == vertices.end()))
		return;

	VerticesMapIterator vMapIter;
	std::vector<double> nodes;
	double distance = 0;

	if (cacheMap) {
		if (cacheMap->getPath(startJunctionId, endJunctionId, nodes) > 0) {
			for (std::size_t n = nodes.size(), i = 0, j = 1; j < n; i++, j++) {
				vMapIter = vertices.find(nodes[i]);
				Vertex* vSource = vMapIter->second.first;

				vMapIter = vertices.find(nodes[j]);
				Vertex* vTarget = vMapIter->second.first;

				Edge* e = vSource->findEdge(vTarget);
				distance += e->weight();
			}
		}
	}

	CityRouter<Vertex, Edge>* Router = nullptr;

	if (distance == 0) {
		Router = getRouter();
		distance = Router->getShortestPath(startJunctionId, endJunctionId, nodes);
	}

	if (distance > 0) {
		for (std::size_t n = nodes.size(), i = 0; i < n; i++) {
			vMapIter = vertices.find(nodes[i]);
			junctions.push_back(vMapIter->second.second);
		}
	}

	if (Router)
		putRouter(Router);

	if (distance <= 0)
		return;

	/* can use for debug */
	//printPath(startJunctionId, endJunctionId, junctions);
}

void CityRouteNetwork::getShortestPath(double startJunctionId, double endJunctionId, std::vector<CityRoad*>& roads)
{
	std::vector<CityJunction*> junctions;

	getShortestPath(startJunctionId, endJunctionId, junctions);

	if (junctions.size()) {
		for (std::size_t n = junctions.size(), i = 0, j = 1; j < n; i++, j++) {
			CityRoad* road = junctions[i]->getRoad(junctions[j]->getJunctionId());
			if (road == nullptr)
				continue;

			roads.push_back(road);
		}
	}
}

void CityRouteNetwork::getShortestPath(double startJunctionId, double endJunctionId, std::vector<std::unique_ptr<CityCoordinateSequence>>& roadCoordSeqs, std::vector<std::vector<double>>& roadDistances)
{
	std::vector<CityRoad*> roads;

	getShortestPath(startJunctionId, endJunctionId, roads);

	if (roads.empty())
		return;

	std::vector<CityRoad*>::iterator roadsIter = roads.begin();

	if (roadsIter != roads.end()) {
		roadCoordSeqs.push_back(std::move((*roadsIter)->getCoordinates()));
		roadDistances.push_back((*roadsIter)->getDistances());
	}

	std::unique_ptr<CityCoordinateSequence> coordSeq;
	std::vector<double> distances;
	double startId;
	double endId;

	roadsIter = roads.begin();
	while (roadsIter != roads.end()) {
		endId = (*roadsIter)->getEndJunctionId();

		if (++roadsIter == roads.end())
			break;

		startId = (*roadsIter)->getStartJunctionId();

		coordSeq = (*roadsIter)->getCoordinates();
		distances = (*roadsIter)->getDistances();

		if (endId != startId) {
			CityCoordinateSequence::reverse(coordSeq.get());
			roadCoordSeqs.push_back(std::move(coordSeq));

			std::size_t start = 0;
			std::size_t end = distances.size() - 1; 
			double dist1, dist2;
			while (start < end) {
				dist1 = distances[start];
				dist2 = distances[end];

				distances[start] = dist2;
				distances[end] = dist1;

				start++;
				end--;
			}
			roadDistances.push_back(distances);
		} else {
			roadCoordSeqs.push_back(std::move(coordSeq));
			roadDistances.push_back(distances);
		}
		distances.clear();
	}
}

void CityRouteNetwork::getShortestPath(double startJunctionId, double endJunctionId, std::vector<std::pair<double, std::pair<std::unique_ptr<CityCoordinateSequence>, std::vector<double>>>>& roadDistances)
{
	std::vector<CityRoad*> roads;

	getShortestPath(startJunctionId, endJunctionId, roads);

	if (roads.empty())
		return;

	std::unique_ptr<CityCoordinateSequence> coordSeq;
	std::vector<double> distances;
	std::vector<CityRoad*>::iterator roadsIter = roads.begin();
	double roadId;

	if (roadsIter != roads.end()) {
		roadId = (*roadsIter)->getRoadId();
		coordSeq = (*roadsIter)->getCoordinates();
		distances = (*roadsIter)->getDistances();
		roadDistances.push_back(std::make_pair(roadId, std::make_pair(std::move(coordSeq), distances)));
	}

	double startId;
	double endId;

	roadsIter = roads.begin();
	while (roadsIter != roads.end()) {
		endId = (*roadsIter)->getEndJunctionId();

		if (++roadsIter == roads.end())
			break;

		roadId = (*roadsIter)->getRoadId();
		startId = (*roadsIter)->getStartJunctionId();

		coordSeq = (*roadsIter)->getCoordinates();
		distances = (*roadsIter)->getDistances();

		if (endId != startId) {
			CityCoordinateSequence::reverse(coordSeq.get());

			std::size_t start = 0;
			std::size_t end = distances.size() - 1; 
			double dist1, dist2;
			while (start < end) {
				dist1 = distances[start];
				dist2 = distances[end];

				distances[start] = dist2;
				distances[end] = dist1;

				start++;
				end--;
			}
			roadDistances.push_back(std::make_pair(roadId, std::make_pair(std::move(coordSeq), distances)));
		} else {
			roadDistances.push_back(std::make_pair(roadId, std::make_pair(std::move(coordSeq), distances)));
		}
		distances.clear();
	}
}

} /* namespace */
