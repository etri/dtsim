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
 * CityRouteTopology.h
 *
 *  $Revision: $
 *  $LastChangedDate: $
 */

#ifndef CITY_ROUTE_TOPOLOGY_H_
#define CITY_ROUTE_TOPOLOGY_H_

#include <unordered_map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <list>
#include <utility>
#include <cmath>
#include <time.h>
#include <limits>

#include "CityException.h"

namespace dtsim {

/**
 * \class CityRouteTopology
 *
 * \brief
 * CityRouteTopology is the base class of each search algorithm.
 *
 * @tparam V the vertex type in this road network
 * @tparam E the edge type in this road network
 */
template<typename V, typename E>
class CityRouteTopology {

protected:
	/// start node identifier in the shortest path
	double start_nodeid;

	/// goal node identifier in the shortest path
	double goal_nodeid;

	typedef typename std::unordered_map<double, V*> RouteVertexMap;
	typedef typename RouteVertexMap::iterator RouteVertexMapIterator;
	RouteVertexMap routeVertices;

	std::vector<E*> routeEdges;
	std::unordered_map<double, E*> routeEdgeMap;

	/// Clears the states of vertices and edges
	void resetStates();

	/// Returns a vertex with the specified node identifier
	V* getVertex(double nodeid);

public:
	/**
	 * Constructor
	 *
	 * @param nodes the vertex map in the route network
	 * @param start the start node identifier
	 * @param goal the goal node identifier
	 */
	CityRouteTopology(std::unordered_map<double, V*>& nodes, double start, double goal);

	virtual ~CityRouteTopology();

	virtual double getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag = true) = 0;

	/// Updates the weight of the specified edge
	virtual void updateEdge(double edgeId, double newWeight);
};

template<typename V, typename E>
CityRouteTopology<V, E>::CityRouteTopology(std::unordered_map<double, V*>& nodes, double start, double goal)
: start_nodeid(start), goal_nodeid(goal)
{
	RouteVertexMapIterator nIter, sIter, tIter;

	/* creats a vertex */
	for (nIter = nodes.begin(); nIter != nodes.end(); nIter++) {
		double nodeid = nIter->first;
		sIter = routeVertices.find(nodeid);
		if (sIter != routeVertices.end())
			continue;

		V* cur_node = nIter->second;
		V* vertex = new V(nodeid, cur_node->getLatitude(), cur_node->getLongitude());
		routeVertices[nodeid] = vertex;
	}

	/* creats an edge between source and target */
	for (nIter = nodes.begin(); nIter != nodes.end(); nIter++) {
		V* node = nIter->second;

		std::vector<E*> edges = node->edges();
		typename std::vector<E*>::iterator eIter;

		for (eIter = edges.begin(); eIter != edges.end(); eIter++) {
			sIter = routeVertices.find((*eIter)->source()->nodeid());
			tIter = routeVertices.find((*eIter)->target()->nodeid());

			V* vSource = sIter->second;
			V* vTarget = tIter->second;

			E* e = vSource->findEdge(vTarget);
			if (e != nullptr) {
				continue;
			}

			e = new E(vSource, vTarget, (*eIter)->getEdgeId(), (*eIter)->weight());

			routeEdges.push_back(e);
			routeEdgeMap[e->getEdgeId()] = e;

			vSource->doAddEdge(e);
			vTarget->doAddEdge(e);
		}
	}
}

template<typename V, typename E>
CityRouteTopology<V, E>::~CityRouteTopology()
{
	RouteVertexMapIterator vertexIter = routeVertices.begin();
	while (vertexIter != routeVertices.end()) {
		V* v = vertexIter->second;
		vertexIter = routeVertices.erase(vertexIter);
		delete v;
	}

	typename std::vector<E*>::iterator edgeIter = routeEdges.begin();
	while (edgeIter != routeEdges.end()) {
		E* e = *edgeIter;
		edgeIter = routeEdges.erase(edgeIter);
		delete e;
	}
	routeEdgeMap.clear();
}

template<typename V, typename E>
void CityRouteTopology<V, E>::resetStates()
{
	RouteVertexMapIterator iter = routeVertices.begin();
	while (iter != routeVertices.end()) {
		iter->second->visited(false);
		iter->second->moved(false);
		iter->second->passed(false);
		iter++;
	}

	for (E* e : routeEdges)
		e->visited(false);
}

template<typename V, typename E>
V* CityRouteTopology<V, E>::getVertex(double nodeid)
{
	RouteVertexMapIterator iter = routeVertices.find(nodeid);
	if (iter != routeVertices.end())
		return iter->second;

	return nullptr;
}

template<typename V, typename E>
void CityRouteTopology<V, E>::updateEdge(double edgeId, double newWeight)
{
	typename std::unordered_map<double, E*>::iterator eIter = routeEdgeMap.find(edgeId);
	if (eIter != routeEdgeMap.end()) {
		E* edge = eIter->second;
		edge->weight(newWeight);
	}
}

/**
 * \class Bidirectional Dijkstra
 *
 * \brief
 * Bidirectional Dijkstra search algorithm can solve single-source shortest path with non-negative edge weight.
 * It constructs the tree of minimum total length between the n nodes and 
 * finds the path of minimum total length between two given nodes P and Q.
 */

#define UNDEFINED   -1

template<typename V, typename E>
class BiDijkstra : public CityRouteTopology<V, E> {

private:
	typedef typename std::unordered_map<V*, double> VertexMap;
	typedef typename VertexMap::iterator VertexMapIterator;

	/// second value is the total distance from start to current in forward/backward direction
	VertexMap F_distances;
	VertexMap B_distances;

	/// second value is the previous nodeid in forward/backward direction
	VertexMap F_prevs;
	VertexMap B_prevs;

	V* mid_node;

	/// forward/backward direction
	std::vector<V*> F_queue;
	std::vector<V*> B_queue;

	double _distance;

	typedef typename dtsim::CityRouteTopology<V, E> RouteTopology;

public:
	BiDijkstra(std::unordered_map<double, V*>& nodes, double start, double goal = -1);

	~BiDijkstra() {}

	/// Gets a node with minimum total length from start to current from OPEN
	V* getNodeWithMinDistanceFromStart(double& min);
	V* getNodeWithMinDistanceFromTarget(double& min);

	/// Sets recomputed weight to current neighbors
	void setForwardDistance(V* current);
	void setBackwardDistance(V* current);

	/// Retruns the node identifier of the predecessor of target node 
	double getForwardPreviousNodeId(V* target);
	double getBackwardPreviousNodeId(V* target);

	/// Returns the predecessor of a node with the specified identifier
	V* findForwardPreviousVertex(double nodeid);
	V* findBackwardPreviousVertex(double nodeid);

	/// Calculates total distance from source to target
	double CalculateDistance(double source, double target);

	/// Runs shortest paths from start to all remaining nodes
	int run();

	/// Gets the shortest path from start to goal from PREVS and puts them into out
	virtual double getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag = true);
};

template<typename V, typename E>
BiDijkstra<V, E>::BiDijkstra(std::unordered_map<double, V*>& nodes, double start, double goal)
: RouteTopology(nodes, start, goal), _distance(std::numeric_limits<int>::max())
{}

template<typename V, typename E>
V* BiDijkstra<V, E>::getNodeWithMinDistanceFromStart(double& min)
{
	min = (double)std::numeric_limits<int>::max();
	V* target = nullptr;

	VertexMapIterator iter = F_distances.begin();
	VertexMapIterator iterEnd = F_distances.end();
	while (iter != iterEnd) {
		V* current = iter->first;

		if ((current->visited() == false) && (min > iter->second)) {
			min = iter->second;
			target = current;
		}

		iter++;
	}

	return target;
}

template<typename V, typename E>
V* BiDijkstra<V, E>::getNodeWithMinDistanceFromTarget(double& min)
{
	min = (double)std::numeric_limits<int>::max();
	V* source = nullptr;

	VertexMapIterator iter = B_distances.begin();
	VertexMapIterator iterEnd = B_distances.end();
	while (iter != iterEnd) {
		V* current = iter->first;

		if ((current->visited() == false) && (min > iter->second)) {
			min = iter->second;
			source = current;
		}

		iter++;
	}

	return source;
}

template<typename V, typename E>
void BiDijkstra<V, E>::setForwardDistance(V* current)
{
	std::vector<V*> out;
	current->getSuccessors(out);
	if (out.size() == 0) {
		return;
	}

	std::vector<E*> edges;
	for (V* node : out) {
    	E* edge = current->findEdge(node);
		if (edge) {
        	edges.push_back(edge);
		}
    } 

	double alt = 0;

	for (E* edge : edges) {
		V* vSource = RouteTopology::getVertex(edge->source()->nodeid());
		V* vTarget = RouteTopology::getVertex(edge->target()->nodeid());

		double length = edge->weight();

		if (current->nodeid() == vSource->nodeid()) {
			alt = F_distances[current] + length;
			if (F_distances[vTarget] == std::numeric_limits<int>::max()|| alt < F_distances[vTarget]) {
				F_distances[vTarget] = alt;
				F_prevs[vTarget] = current->nodeid();

				// set the true distance from start to target
				if (vTarget->visited()) {
					if (alt + B_distances[vTarget] < _distance) {
						_distance = alt + B_distances[vTarget];
						mid_node = vTarget;
					}
				}
			}
		} else if (current->nodeid() == vTarget->nodeid()) {
			alt = F_distances[current] + length;
			if (F_distances[vSource] == std::numeric_limits<int>::max()|| alt < F_distances[vSource]) {
				F_distances[vSource] = alt;
				F_prevs[vSource] = current->nodeid();
			} else  {
				if (F_distances[current] == std::numeric_limits<int>::max()|| alt < F_distances[current]) {
					F_distances[current] = alt;
					F_prevs[current] = vSource->nodeid();
				}
			}
		}
	}

	out.clear();
	edges.clear();
}

template<typename V, typename E>
void BiDijkstra<V, E>::setBackwardDistance(V* current)
{
	std::vector<V*> out;
	current->getPredecessors(out);
	if (out.size() == 0) {
		return;
	}

	std::vector<E*> edges;
	for (V* node : out) {
    	E* edge = node->findEdge(current);
		if (edge) {
        	edges.push_back(edge);
		}
    } 

	double alt = 0;

	for (E* edge : edges) {
		V* vSource = RouteTopology::getVertex(edge->source()->nodeid());
		V* vTarget = RouteTopology::getVertex(edge->target()->nodeid());

		double length = edge->weight();

		if (current->nodeid() == vSource->nodeid()) {
			alt = B_distances[current] + length;
			if (B_distances[vTarget] == std::numeric_limits<int>::max()|| alt < B_distances[vTarget]) {
				B_distances[vTarget] = alt;
				B_prevs[vTarget] = current->nodeid();
			}
		} else if (current->nodeid() == vTarget->nodeid()) {
			alt = B_distances[current] + length;
			if (B_distances[vSource] == std::numeric_limits<int>::max()|| alt < B_distances[vSource]) {
				B_distances[vSource] = alt;
				B_prevs[vSource] = current->nodeid();

				// set the true distance from target to current
				if (vSource->visited()) {
					if (alt + F_distances[vSource] < _distance) {
						_distance = alt + F_distances[vSource];
						mid_node = vSource;
					}
				}
			} else  {
				if (B_distances[current] == std::numeric_limits<int>::max()|| alt < B_distances[current]) {
					B_distances[current] = alt;
					B_prevs[current] = vSource->nodeid();
				}
			}
		}
	}

	out.clear();
	edges.clear();
}

template<typename V, typename E>
double BiDijkstra<V, E>::getForwardPreviousNodeId(V* target) 
{
    typename std::unordered_map<V*, double>::iterator pIter;
	for (pIter = F_prevs.begin(); pIter != F_prevs.end(); pIter++) {
		V* current = pIter->first;
		if (current->nodeid() == target->nodeid()) {
			double nodeid = pIter->second;
			return nodeid;
		}
	}

	return 0;
}

template<typename V, typename E>
double BiDijkstra<V, E>::getBackwardPreviousNodeId(V* target) 
{
    typename std::unordered_map<V*, double>::iterator pIter;
	for (pIter = B_prevs.begin(); pIter != B_prevs.end(); pIter++) {
		V* current = pIter->first;
		if (current->nodeid() == target->nodeid()) {
			double nodeid = pIter->second;
			return nodeid;
		}
	}

	return 0;
}

template<typename V, typename E>
V* BiDijkstra<V, E>::findForwardPreviousVertex(double nodeid) 
{
    typename std::unordered_map<V*, double>::iterator pIter;
	for (pIter = F_prevs.begin(); pIter != F_prevs.end(); pIter++) {
		V* current = pIter->first;
		if (current->nodeid() == nodeid) {
			return current;
		}
	}

	return nullptr;
}

template<typename V, typename E>
V* BiDijkstra<V, E>::findBackwardPreviousVertex(double nodeid) 
{
    typename std::unordered_map<V*, double>::iterator pIter;
	for (pIter = B_prevs.begin(); pIter != B_prevs.end(); pIter++) {
		V* current = pIter->first;
		if (current->nodeid() == nodeid) {
			return current;
		}
	}

	return nullptr;
}

template<typename V, typename E>
double BiDijkstra<V, E>::CalculateDistance(double source, double target)
{
	V* vSource = RouteTopology::getVertex(source);
	double weight = 0;

	std::vector<V*> out;
	vSource->getSuccessors(out);

	std::vector<E*> edges;
	for (V* node : out) {
    	E* edge = vSource->findEdge(node);
		if (edge) {
        	edges.push_back(edge);
		}
    }

	for (E* edge : edges) {
		if (edge->target()->nodeid() == target || edge->source()->nodeid() == target) {
			weight = edge->weight();
			break;
		}
	}

	out.clear();
	edges.clear();

	return weight;
}

template<typename V, typename E>
int BiDijkstra<V, E>::run()
{
	typename RouteTopology::RouteVertexMapIterator iter = RouteTopology::routeVertices.begin();
	typename RouteTopology::RouteVertexMapIterator iterEnd = RouteTopology::routeVertices.end();

	while (iter != iterEnd) {
		F_distances[iter->second] = std::numeric_limits<int>::max();
		B_distances[iter->second] = std::numeric_limits<int>::max();

		F_prevs[iter->second] = (double)UNDEFINED;
		B_prevs[iter->second] = (double)UNDEFINED;

		F_queue.push_back(iter->second);
		B_queue.push_back(iter->second);

		iter++;
	}

	V* vStart = RouteTopology::getVertex(RouteTopology::start_nodeid);
	V* vGoal = RouteTopology::getVertex(RouteTopology::goal_nodeid);

	if (vStart == nullptr || vGoal == nullptr) {
		F_queue.clear();
		B_queue.clear();
		return -1;
	}

	F_distances[vStart] = 0;
	B_distances[vGoal] = 0;

	double F_min = 0, B_min = 0, leastEdgeCost = 0;;
	while (!F_queue.empty() && !B_queue.empty()) {
		V* fCurrent = getNodeWithMinDistanceFromStart(F_min);
		if (fCurrent == nullptr) {
			break;
		}

		V* bCurrent = getNodeWithMinDistanceFromTarget(B_min);
		if (bCurrent == nullptr) {
			break;
		}

		if (F_min + B_min >= _distance) {
			break; 
		}

		fCurrent->visited(true);

		typename std::vector<V*>::iterator fqIter;
		for (fqIter = F_queue.begin(); fqIter != F_queue.end(); fqIter++) {
			if ((*fqIter)->nodeid() == fCurrent->nodeid()) {
				F_queue.erase(fqIter);
				break;
			}
		}

		bCurrent->visited(true);

		typename std::vector<V*>::iterator bqIter;
		for (bqIter = B_queue.begin(); bqIter != B_queue.end(); bqIter++) {
			if ((*bqIter)->nodeid() == bCurrent->nodeid()) {
				B_queue.erase(bqIter);
				break;
			}
		}

		setForwardDistance(fCurrent);
		setBackwardDistance(bCurrent);

		if (fCurrent->nodeid() == bCurrent->nodeid()) {
			mid_node = fCurrent;
			break;
		}
	}

	if (!F_queue.empty()) {
		F_queue.clear();
	}

	if (!B_queue.empty()) {
		B_queue.clear();
	}

	return 0;
}

template<typename V, typename E>
double BiDijkstra<V, E>::getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag)
{
	if (resetFlag) {
		RouteTopology::start_nodeid = start;
		RouteTopology::goal_nodeid = goal;
		RouteTopology::resetStates();

		F_distances.clear();
		B_distances.clear();
		F_prevs.clear();
		B_prevs.clear();
		F_queue.clear();
		B_queue.clear();	
		_distance = std::numeric_limits<int>::max();
	}

	mid_node = nullptr;

	/* Read-time search processing */
	if (this->run() < 0)
		return -1;

	if (mid_node == nullptr) {
		return 0;
	}

	/* Forward nodes */
	V* vSource = findForwardPreviousVertex(RouteTopology::start_nodeid);
	V* vTarget = findForwardPreviousVertex(mid_node->nodeid());

	if (vSource == nullptr || vTarget == nullptr) {
		return -1;
	}

	std::vector<double> F_out;;

	F_out.insert(F_out.begin(), vTarget->nodeid());

	double local_distance = 0;
	double prev_nodeid = 0;
	double target_nodeid = 0;

	while (RouteTopology::start_nodeid != vTarget->nodeid()) {
		prev_nodeid = getForwardPreviousNodeId(vTarget);
		target_nodeid = vTarget->nodeid();

		if (prev_nodeid <= 0) {
			return 0;
		} else {
			local_distance += CalculateDistance(prev_nodeid, target_nodeid);

			vTarget = RouteTopology::getVertex(prev_nodeid);
			F_out.insert(F_out.begin(), prev_nodeid);
		}
	}

	if (F_out.size()) {
		if (F_out.front() != start) {
			return 0;
		}
	} else {
		return 0;
	}

	/* Backward nodes */
	vSource = findForwardPreviousVertex(mid_node->nodeid());
	vTarget = findForwardPreviousVertex(RouteTopology::goal_nodeid);

	if (vTarget == nullptr) {
		return -1;
	}

	out.insert(out.begin(), F_out.begin(), F_out.end());

	double post_nodeid = 0;
	double source_nodeid = 0;

	while (RouteTopology::goal_nodeid != vSource->nodeid()) {
		post_nodeid = getBackwardPreviousNodeId(vSource);
		source_nodeid = vSource->nodeid();

		if (post_nodeid <= 0) {
			return 0;
		} else {
			local_distance += CalculateDistance(source_nodeid, post_nodeid);

			vSource = RouteTopology::getVertex(post_nodeid);
			out.push_back(post_nodeid);
		}
	}

	if (out.back() != goal) {
		out.clear();
		_distance = 0;
	}

	return _distance;
}

/**
 * \class Value for A* Algorithm
 *
 * \brief
 * A collection that stores the distance values between two junctions
 * to get the shortest path with A* search algorithm
 * This is used in A* search algorithm.
 */
class Value {

public:
	/// Returns the lowest value that is the sum of the exact and estimated values
	double lowestCost;

	/// Returns the exact value that is the weight between the specified vertices
	double exactCost;

	/// Returns the estimated value that is the heuristic distance from a specified vertex to the goal one
	double estimatedCost;

	/**
 	 * Creates a Value with distance values
 	 */
	Value(double fVal, double gVal, double hVal):lowestCost(fVal),exactCost(gVal),estimatedCost(hVal) {}
		
	~Value() {}
};

/**
 * \class AStar
 *
 * \brief
 * A* search algorithm implementation can solve for sinlge-pair shortest path 
 * using heuristics to try to speed up the search.
 */
template<typename V, typename E>
class AStar : public CityRouteTopology<V, E> {

private:
    std::list<std::pair<V*, std::pair<V*, Value*>>> open;
    std::unordered_map<V*, std::pair<V*, Value*>> 	closed;
	std::unordered_map<V*, Value*> 					valueMap;

	/// if search filed, run one more with Bidirectional Dijkstra algorithm
	BiDijkstra<V, E>* bd_router;

	double goal_latitude;
	double goal_longitude;
	double _distance;

	typedef typename dtsim::CityRouteTopology<V, E> RouteTopology;

public:
    AStar(std::unordered_map<double, V*>& nodes, double start, double goal);

    ~AStar();

	/// Clears local sturcutres
	void cleanUp();

 	/// Returns a distance generated by Mahattan Distance Heuristic from vertex to goal
	double ManhattanDistanceHeuristic(V* vertex);

 	/// Returns a distance generated by Euclidean Distance Heuristic from vertex to goal
	double EuclideanDistanceHeuristic(V* vertex);

	/// Gets a node not yet processed from OPEN
    V* getNextNode(std::pair<V*, std::pair<V*, Value*>>& node);

	/// Inserts a node to OPEN in ascending order of the lowest cost 
    void rearrange(std::pair<V*, std::pair<V*, Value*>> adjNode);

	/// Moves the current node erased from CLOSED to OPEN
	void moveClosedToOpen(V* current, double gVal, V* parent);

	/// Runs the shortest path search between start and goal
	int run();

	/// Finds the shorest path from CLOSED and puts them into OUT
	virtual double getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag = true);

	/// Updates the weight of the specified edge
	virtual void updateEdge(double edgeId, double newWeight);
};

template<typename V, typename E>
AStar<V, E>::AStar(std::unordered_map<double, V*>& nodes, double start, double goal)
: RouteTopology(nodes, start, goal), goal_latitude(0), goal_longitude(0), _distance(0)
{
	bd_router = new BiDijkstra<V, E>(nodes, -1, -1);
}

template<typename V, typename E>
AStar<V, E>::~AStar()
{
	cleanUp();

	delete bd_router;
}

template<typename V, typename E>
void AStar<V, E>::cleanUp()
{
	typename std::unordered_map<V*, Value*>::iterator mIter;
	for (mIter = valueMap.begin(); mIter != valueMap.end(); mIter++) {
		Value* value = mIter->second;
		delete value;
	}

	valueMap.clear();
	open.clear();
	closed.clear();
}

template<typename V, typename E>
void AStar<V, E>::updateEdge(double edgeId, double newWeight)
{
	RouteTopology::updateEdge(edgeId, newWeight);
}

/**
 * Manhattan Distance Heuristic(MDH)
 * 	h = abs(x.start-x.destination) + abs(y.start-y.destination); 
 */
template<typename V, typename E>
double AStar<V, E>::ManhattanDistanceHeuristic(V* vertex)
{
	return (std::fabs(vertex->getLatitude() - goal_latitude) + std::fabs(vertex->getLongitude() - goal_longitude));
}

/**
 * Eudlidean Distance Heuristic(EDH)
 *	h = sqrt((x.start-x.destination)^2 + (y.start-y.destination)^2); 
 */
template<typename V, typename E>
double AStar<V, E>::EuclideanDistanceHeuristic(V* vertex)
{
	double rLat = std::pow((vertex->getLatitude() - goal_latitude), 2.0);
	double rLong = std::pow((vertex->getLongitude() - goal_longitude), 2.0);
	return std::sqrt(rLat + rLong);
}

template<typename V, typename E>
V* AStar<V, E>::getNextNode(std::pair<V*, std::pair<V*, Value*>>& pCurrent)
{
	V* current;

	while (open.size() > 0) {
		/* gets the first entry */
       	pCurrent = open.front();
        current = pCurrent.first;

		/* this node has been already processed. try next */
		if(current->visited() == true) {
       		open.pop_front();
			continue;
		}

		/* begins from start node */
		if (current->nodeid() == RouteTopology::start_nodeid) {
			closed[current] = pCurrent.second;
			current->visited(true);
		}

       	current->moved(false);

		/* removes the first entry of OPEN list(queue) */
        open.pop_front();

		return current;
	}

    return nullptr;
}

template<typename V, typename E>
void AStar<V, E>::rearrange(std::pair<V*, std::pair<V*, Value*>> adjNode)
{
	typename std::list<std::pair<V*, std::pair<V*, Value*>>>::iterator ait;
    for (ait = open.begin(); ait != open.end(); ait++) {
    	Value* currentValue = (*ait).second.second;
		Value* adjValue = adjNode.second.second;

    	if (currentValue->lowestCost > adjValue->lowestCost) {
        	open.insert(ait, adjNode);
            break;
        }
    }

    if (ait == open.end()) {
        open.insert(open.end(), adjNode);
    }
}

template<typename V, typename E>
void AStar<V, E>::moveClosedToOpen(V* current, double gVal, V* parent)
{
	typename std::unordered_map<V*, std::pair<V*, Value*>>::iterator cIter = closed.find(current);
	std::pair<V*, Value*> keyValue = cIter->second;

	current->visited(false);

	/* moves to OPEN list again */
	current->moved(true);

	/* removes from CLOSED map */
	closed.erase(cIter);

	keyValue.first = parent;
	keyValue.second->lowestCost = gVal + keyValue.second->estimatedCost;
	keyValue.second->exactCost = gVal;

    std::pair<V*, std::pair<V*, Value*>> adjNode(current, keyValue);

	/* adds to OPEN list */
	if (open.size() == 0) {
		open.insert(open.begin(), adjNode);
	} else {
		rearrange(adjNode);
	}
}

template<typename V, typename E>
int AStar<V, E>::run()
{
	V* source = RouteTopology::getVertex(RouteTopology::start_nodeid);
	V* target = RouteTopology::getVertex(RouteTopology::goal_nodeid);

	if (source == nullptr || target == nullptr) {
		return -1;
	}

	goal_latitude = target->getLatitude();
	goal_longitude = target->getLongitude();

	Value* nodeVal = new Value(0.0,0.0,0.0);
   	open.push_back(std::make_pair(source, std::make_pair(source, nodeVal)));
	valueMap[source] = nodeVal;

	while (open.size() > 0) {
		std::pair<V*, std::pair<V*, Value*>> pCurrent;
        V* current = getNextNode(pCurrent);

		if (current == nullptr) {
            return -1;
		}

        if (current->nodeid() == RouteTopology::goal_nodeid) {
	       	closed[current] = pCurrent.second;
        	current->visited(true);
            break;
        }

		/* gets the successors of current node */
		std::vector<V*> out;
		current->getSuccessors(out);

		if (out.size() == 0) {
			return -1;
		}

		Value* currentValue = pCurrent.second.second;
        double distToCurrent = currentValue->exactCost;	// distance from source to current
		double gVal = 0;
		std::vector<E*> edges;

		/* gets edges between current node and its successors */
		for (V* node : out ) {
			E* e = current->findEdge(node);
			if (e) {
				edges.push_back(e);
			}
		}

		/* calculates the lowest cost of each edge */
		for (E* edge : edges) {
			if (edge->visited()) {
				continue;
			}

			V* successor = edge->target();

			if (successor->nodeid() == pCurrent.first->nodeid()) {
				successor = edge->source();
			}

			/* computes the distance from current node to its successor */
			gVal = edge->weight() + distToCurrent;	

			typename std::unordered_map<V*, Value*>::iterator vIter = valueMap.find(successor);

            if (successor->visited()) {
				/* if a successor is in CLOSED list */
				if (successor->nodeid() == RouteTopology::start_nodeid) {
					continue;
				}

				Value* successorValue = vIter->second;
				if (successorValue->exactCost <= gVal) {
					continue;
				}

				/* moves to OPEN list again */
				moveClosedToOpen(successor, gVal, current);
				continue;
            } else if (successor->moved()) {
				/* successor is in the OPEN list */
				Value* successorValue = vIter->second;
				if (successorValue->exactCost <= gVal) {
					continue;
				} 
				successorValue->exactCost = gVal;	// distance from source to this node
				continue;
			} else { 
				/* successor is not in any lists */
			//	double hVal = ManhattanDistanceHeuristic(successor);
				double hVal = EuclideanDistanceHeuristic(successor);

				Value* new_value = new Value(gVal+hVal, gVal, hVal);
				valueMap[successor] = new_value;

				std::pair<V*, std::pair<V*, Value*>> pSuccessor;
           		pSuccessor = std::make_pair(successor, std::make_pair(current, new_value));

				edge->visited(true);

				/* add pSuccessor to the OPEN list */
				successor->moved(true);

				if (open.size() == 0) {
					open.insert(open.begin(), pSuccessor);
				} else {
					typename std::list<std::pair<V*, std::pair<V*, Value*>>>::iterator oIter;
					for (oIter = open.begin(); oIter != open.end(); oIter++) {
						Value* oValue = (*oIter).second.second;
						Value* sValue = pSuccessor.second.second;

						if (oValue->lowestCost > sValue->lowestCost) {
							open.insert(oIter, pSuccessor);
							break;
						}
					}

					if (oIter == open.end()) {
						open.insert(open.end(), pSuccessor);
					}
				}
			}
		}

		if (current->nodeid() != RouteTopology::start_nodeid) {
	       	closed[current] = pCurrent.second;
       		current->visited(true);
		}

		out.clear();
		edges.clear();
	}

	return 0;
}
  
template<typename V, typename E>
double AStar<V, E>::getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag)
{
	if (resetFlag) {
		RouteTopology::start_nodeid = start;
		RouteTopology::goal_nodeid = goal;
		RouteTopology::resetStates();

		cleanUp();
	}

	if (run() < 0) {
		return -1;
	}

	std::vector<double> printRoute;
    double target = goal;
	double prev = target;

	V* source = RouteTopology::getVertex(start);
	V* current = RouteTopology::getVertex(goal);
	if (source == nullptr || current == nullptr) {
		return -1;
	}

	printRoute.insert(printRoute.begin(), current->nodeid());
	out.insert(out.begin(), current->nodeid());

    while (target != start) {
    	typename std::unordered_map<V*, std::pair<V*, Value*>>::iterator cIter;
    	for (cIter = closed.begin(); cIter != closed.end(); cIter++) {
			V* child = cIter->first;

			if (child->nodeid() == goal) {
				Value* cost = cIter->second.second;
				_distance = cost->exactCost;
			}

            if (child->nodeid() == target) {
				V* parent = cIter->second.first;
				if (parent->passed()) {
					continue;
				}

				printRoute.insert(printRoute.begin(), parent->nodeid());
				out.insert(out.begin(), parent->nodeid());
                target = parent->nodeid();
				parent->passed(true);
                break;
            }
        }

		if (prev == target) {
			break;
		}

		prev = target;
	}

	if (out.front() != start) {
		out.clear();
		_distance = 0;

		/* run BiDijkstra algorithm if there's no path from start to goal */  
		_distance = bd_router->getShortestPath(start, goal, out, resetFlag);
	}

	printRoute.clear();

	return _distance;
}

/**
 * \class BFS
 *
 * \brief
 * BFS is an unifrom-cost search algorithm, which can solve for sinlge-pair shortest path.
 * 
 * Search procedure is as follows;
 * 	1. Extracts a node(U) with the lowest cost from OPEN
 * 	2. Generates U's neighbors and determine tis cost of each node V
 * 	3. Performs a duplicate check on V and if V passed, inserts V to OPEN
 * 	4. Inserts U into CLOSED
 */
template<typename V, typename E>
class BFS : public CityRouteTopology<V, E> {

private:
    std::list<std::pair<V*, std::pair<V*, double>>> open;
    std::unordered_map<V*, std::pair<V*, double>> 	 closed;

	double _distance;

	typedef typename dtsim::CityRouteTopology<V, E> RouteTopology;

public:
	/**
	 * Creates a BFS object
	 *
	 * @param nodes the vertices to be added to this route topology
	 * @param start	the star tnode identifier
	 * @param goal the goal node identifier
	 */
	BFS(std::unordered_map<double, V*>& nodes, double start, double goal);

	~BFS() {}

	/// Gets a not not yet processed from OPEN
	V* getNextNode(std::pair<V*, std::pair<V*, double>>& node);

	/// Inserts a node to OPEN in ascending order of the lowest cost
	void rearrange(std::pair<V*, std::pair<V*,double>> adjNode);

	/// Calculates total distance from start to goal
	void CalculateDistance(std::vector<V*> Route);

	/// Runs the shortest path from start to goal
	int run();

	/// Gets the shortest path from CLOSED and puts them into OUT
	virtual double getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag = true);

	/// Updates the weight of the specified edge
	virtual void updateEdge(double edgeId, double newWeight);
};

template<typename V, typename E>
BFS<V, E>::BFS(std::unordered_map<double, V*>& nodes, double start, double goal)
: RouteTopology(nodes, start, goal), _distance(0)
{}

template<typename V, typename E>
void BFS<V, E>::updateEdge(double edgeId, double newWeight)
{
	RouteTopology::updateEdge(edgeId, newWeight);
}

template<typename V, typename E>
V* BFS<V, E>::getNextNode(std::pair<V*, std::pair<V*, double>>& node) 
{
	V* current;
	V* previous;
	double distance = 0;

	while (open.size() > 0) {
		/* get first queue entry */
       	node = open.front();
        current = node.first;

		if (current->visited() == true) {
			/* this node is already processed. try next */
       		open.pop_front();
			continue;
		}

		previous = RouteTopology::getVertex(node.second.first->nodeid());
       	distance = node.second.second;
        closed[current] = std::make_pair(previous, distance);

       	current->visited(true);

		/* remove first queue entry */
        open.pop_front();

		return current;
	}

	return nullptr;
}

template<typename V, typename E>
void BFS<V, E>::rearrange(std::pair<V*, std::pair<V*, double>> adjNode) 
{
	typename std::list<std::pair<V*, std::pair<V*, double>>>::iterator oIter;

	for (oIter = open.begin(); oIter != open.end(); oIter++) {
    	if ((*oIter).first->nodeid() == adjNode.first->nodeid()) {
			double cur_distance = (*oIter).second.second;
			double adj_distance = adjNode.second.second;

        	if (cur_distance > adj_distance) {
            	open.erase(oIter);
        	} else if (cur_distance == adj_distance) {
				V* current_previous = (*oIter).second.first;
				V* neighbor_previous = adjNode.second.first;

        		if (current_previous->nodeid() == neighbor_previous->nodeid()) {
					/* duplicated entry */
					return;
				}
			}
        	break;
		}
	}

    for (oIter = open.begin(); oIter != open.end(); oIter++) {
		double cur_distance = (*oIter).second.second;
		double adj_distance = adjNode.second.second;

    	if (cur_distance > adj_distance) {
        	open.insert(oIter, adjNode);
            break;
        }
    }

    if (oIter == open.end()) {
        open.insert(open.end(), adjNode);
    }
}

template<typename V, typename E>
int BFS<V, E>::run() 
{
	V* source = RouteTopology::getVertex(RouteTopology::start_nodeid);
	if (source == nullptr) {
		return -1;
	}
	open.push_back(std::make_pair(source, std::make_pair(source, 0)));

	while (open.size() > 0) {
		std::pair<V*, std::pair<V*, double>> pCurrent;
		V* current = getNextNode(pCurrent);

		if (current == nullptr) {
			return -1;
		}

		if (current->nodeid() == RouteTopology::goal_nodeid) {
			break;
		}

		std::vector<V*> out;
		current->getSuccessors(out);

		if (out.size() == 0) {
			return -1;
		}

		double distance = pCurrent.second.second;
		std::vector<E*> edges;

		for (V* s_node : out) {
            edges.push_back(current->findEdge(s_node));
        }

		for (E* edge : edges) {
			V* predecessor = edge->source();
			V* successor = edge->target();

			if (predecessor->visited() && successor->visited()) {
				continue;
			}

			if (successor->nodeid() == current->nodeid()) {
				successor = predecessor;
			}

			std::pair<V*, std::pair<V*, double>> pSuccessor;
			pSuccessor = std::make_pair(successor, std::make_pair(current, edge->weight()+distance));

			if (open.size() == 0) {
				open.insert(open.begin(), pSuccessor);
			} else {
				rearrange(pSuccessor);
			}
		}

		out.clear();
		edges.clear();
	}

	return 0;
}

template<typename V, typename E>
void BFS<V, E>::CalculateDistance(std::vector<V*> Route) 
{
	V *source = nullptr;
	V *target = nullptr;

	_distance = 0;

	typename std::vector<V*>::iterator rIter;
	for (rIter = Route.begin(); rIter != Route.end(); rIter++) {
		if (rIter == Route.begin()) {
			source = (*rIter);
			continue;
		}
		target = (*rIter);

		std::vector<V*> out;
		source->getSuccessors(out);

		std::vector<E*> edges;
		for (V* node : out) {
			E* edge = source->findEdge(node);
			if (edge) {
				edges.push_back(edge);
			}
		}

		for (E* edge : edges) {
			V* predecessor = edge->source();
			V* successor = edge->target();

			if (predecessor->nodeid() == target->nodeid() || successor->nodeid() == target->nodeid()) {
				_distance += edge->weight();
				break;
			}
		}
		source = target;

		out.clear();
		edges.clear();
	}
}

template<typename V, typename E>
double BFS<V, E>::getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag) 
{
	if (resetFlag) {
		RouteTopology::start_nodeid = start;
		RouteTopology::goal_nodeid = goal;
		RouteTopology::resetStates();

		open.clear();
		closed.clear();
	}

	if (run() < 0) {
		return 0;
	}

	std::vector<V*> printRoute;
    double target = goal;
	double prev = target;

	V* source = RouteTopology::getVertex(start);
	V* current = RouteTopology::getVertex(goal);
	if (source == nullptr || current == nullptr) {
		return -1;
	}

	printRoute.insert(printRoute.begin(), current);
	out.insert(out.begin(), current->nodeid());

    while (target != start) {
    	typename std::unordered_map<V*, std::pair<V*, double>>::iterator cIter;
    	for (cIter = closed.begin(); cIter != closed.end(); cIter++) {
        	if (cIter->first->nodeid() == target) {
				current = (cIter->second).first;
				printRoute.insert(printRoute.begin(), current);
				out.insert(out.begin(), current->nodeid());
                target = (cIter->second).first->nodeid();
                break;
            }
        }

		if (prev == target) {
			break;
		}

		prev = target;
    }

	if (out.front() != start) {
		out.clear();
		_distance = 0;
	} else {
		CalculateDistance(printRoute);
	}

	printRoute.clear();

	return _distance;
}

/**
 * \class Dijkstra
 *
 * \brief
 * Dijkstra search algorithm can solve single-source shortest path with non-negative edge weight.
 * It constructs the tree of minimum total length between the n nodes and 
 * finds the path of minimum total length between two given nodes P and Q.
 */

template<typename V, typename E>
class Dijkstra : public CityRouteTopology<V, E> {

private:
	typedef typename std::unordered_map<V*, double> VertexMap;
	typedef typename VertexMap::iterator VertexMapIterator;

	/// second value is the total distance from start to current
	VertexMap distances;

	/// second value is the previous nodeid
	VertexMap prevs;

	std::vector<V*> queue;

	double _distance;

	typedef typename dtsim::CityRouteTopology<V, E> RouteTopology;

public:
	Dijkstra(std::unordered_map<double, V*>& nodes, double start, double goal = -1);

	~Dijkstra() {}

	/// Gets a node with minimum total length from start to current from OPEN
	V* getNodeWithMinDistanceFromStart();

	/// Sets recomputed weight to current neighbors
	void setDistance(V* current);

	/// Retrunes the node identifier of the predecessor of target node 
	double getPreviousNodeId(V* target);

	/// Returns the predecessor of a node with the specified identifier
	V* findPreviousVertex(double nodeid);

	/// Calculates total distance from source to target
	double CalculateDistance(double source, double target);

	/// Runs shortest paths from start to all remaining nodes
	int run();

	/// Gets the shortest path from start to goal from PREVS and puts them into out
	virtual double getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag = true);

	/// Updates the weight of the specified edge
	virtual void updateEdge(double edgeId, double newWeight);

	/// Used by cacheGen Utility
	void runWithCache(double start);

	/// Used by cacheGen Utility
	double getShortestPathWithCache(double start, double goal, std::vector<double>& out);
};

template<typename V, typename E>
Dijkstra<V, E>::Dijkstra(std::unordered_map<double, V*>& nodes, double start, double goal)
: RouteTopology(nodes, start, goal), _distance(0)
{}

template<typename V, typename E>
void Dijkstra<V, E>::updateEdge(double edgeId, double newWeight)
{
	RouteTopology::updateEdge(edgeId, newWeight);
}

template<typename V, typename E>
V* Dijkstra<V, E>::getNodeWithMinDistanceFromStart() 
{
	double min = (double)std::numeric_limits<int>::max();
	V* target = nullptr;

	VertexMapIterator iter = distances.begin();
	VertexMapIterator iterEnd = distances.end();
	while (iter != iterEnd) {
		V* current = iter->first;

		if ((current->visited() == false) && (min > iter->second)) {
			min = iter->second;
			target = current;
		}

		iter++;
	}

	return target;
}

template<typename V, typename E>
void Dijkstra<V, E>::setDistance(V* current)
{
	std::vector<V*> out;
	current->getSuccessors(out);
	if (out.size() == 0) {
		return;
	}

	std::vector<E*> edges;
	for (V* node : out) {
    	E* edge = current->findEdge(node);
		if (edge) {
        	edges.push_back(edge);
		}
    } 

	double alt = 0;

	for (E* edge : edges) {
		V* vSource = RouteTopology::getVertex(edge->source()->nodeid());
		V* vTarget = RouteTopology::getVertex(edge->target()->nodeid());

		double length = edge->weight();

		if (current->nodeid() == vSource->nodeid()) {
			alt = distances[current] + length;
			if (distances[vTarget] == std::numeric_limits<int>::max()|| alt < distances[vTarget]) {
				distances[vTarget] = alt;
				prevs[vTarget] = current->nodeid();
			}
		} else if (current->nodeid() == vTarget->nodeid()) {
			alt = distances[current] + length;
			if (distances[vSource] == std::numeric_limits<int>::max()|| alt < distances[vSource]) {
				distances[vSource] = alt;
				prevs[vSource] = current->nodeid();
			} else  {
				if (distances[current] == std::numeric_limits<int>::max()|| alt < distances[current]) {
					distances[current] = alt;
					prevs[current] = vSource->nodeid();
				}
			}
		}
	}

	out.clear();
	edges.clear();
}

template<typename V, typename E>
double Dijkstra<V, E>::getPreviousNodeId(V* target) 
{
    typename std::unordered_map<V*, double>::iterator pIter;
	for (pIter = prevs.begin(); pIter != prevs.end(); pIter++) {
		V* current = pIter->first;
		if (current->nodeid() == target->nodeid()) {
			double nodeid = pIter->second;
			return nodeid;
		}
	}

	return 0;
}

template<typename V, typename E>
V* Dijkstra<V, E>::findPreviousVertex(double nodeid) 
{
    typename std::unordered_map<V*, double>::iterator pIter;
	for (pIter = prevs.begin(); pIter != prevs.end(); pIter++) {
		V* current = pIter->first;
		if (current->nodeid() == nodeid) {
			return current;
		}
	}

	return nullptr;
}

template<typename V, typename E>
double Dijkstra<V, E>::CalculateDistance(double source, double target)
{
	V* vSource = RouteTopology::getVertex(source);
	double weight = 0;

	std::vector<V*> out;
	vSource->getSuccessors(out);

	std::vector<E*> edges;
	for (V* node : out) {
    	E* edge = vSource->findEdge(node);
		if (edge) {
        	edges.push_back(edge);
		}
    }

	for (E* edge : edges) {
		if (edge->target()->nodeid() == target || edge->source()->nodeid() == target) {
			weight = edge->weight();
			break;
		}
	}

	out.clear();
	edges.clear();

	return weight;
}

template<typename V, typename E>
int Dijkstra<V, E>::run()
{
	typename RouteTopology::RouteVertexMapIterator iter = RouteTopology::routeVertices.begin();
	typename RouteTopology::RouteVertexMapIterator iterEnd = RouteTopology::routeVertices.end();

	while (iter != iterEnd) {
		distances[iter->second] = std::numeric_limits<int>::max();
		prevs[iter->second] = (double)UNDEFINED;
		queue.push_back(iter->second);
		iter++;
	}

	V* vStart = RouteTopology::getVertex(RouteTopology::start_nodeid);
	if (vStart == nullptr) {
		return -1;
	}
	distances[vStart] = 0;

	while (!queue.empty()) {
		V* current = getNodeWithMinDistanceFromStart();
		if (current == nullptr) {
			break;
		}

		current->visited(true);

		typename std::vector<V*>::iterator qIter;
		for (qIter = queue.begin(); qIter != queue.end(); qIter++) {
			if ((*qIter)->nodeid() == current->nodeid()) {
				queue.erase(qIter);
				break;
			}
		}

		setDistance(current);
	}

	if (!queue.empty()) {
		queue.clear();
	}

	return 0;
}

template<typename V, typename E>
double Dijkstra<V, E>::getShortestPath(double start, double goal, std::vector<double>& out, bool resetFlag)
{
	if (resetFlag) {
		RouteTopology::start_nodeid = start;
		RouteTopology::goal_nodeid = goal;
		RouteTopology::resetStates();

		distances.clear();
		prevs.clear();
		queue.clear();
	}

	/* Read-time search processing */
	if (this->run() < 0) {
		return -1;
	}

	V* vSource = findPreviousVertex(RouteTopology::start_nodeid);
	V* vTarget = findPreviousVertex(RouteTopology::goal_nodeid);

	if (vSource == nullptr || vTarget == nullptr) {
		return -1;
	}

	std::vector<double> printRoute;

	out.insert(out.begin(), vTarget->nodeid());
	printRoute.insert(printRoute.begin(), vTarget->nodeid());

	_distance = 0;

	while (RouteTopology::start_nodeid != vTarget->nodeid()) {
		double prev_nodeid = getPreviousNodeId(vTarget);
		double target_nodeid = vTarget->nodeid();

		if (prev_nodeid <= 0) {
			out.clear();
			return 0;
		} else {
			_distance += CalculateDistance(prev_nodeid, target_nodeid);

			vTarget = RouteTopology::getVertex(prev_nodeid);
			out.insert(out.begin(), prev_nodeid);
			printRoute.insert(printRoute.begin(), vTarget->nodeid());
		}
	}

	if (out.front() != start) {
		out.clear();
		_distance = 0;
	}

	printRoute.clear();

	return _distance;
}

/* for Cache Utility */
template<typename V, typename E>
void Dijkstra<V, E>::runWithCache(double start)
{
	/* Set a start nodeid and reset the status of all vertices and edges */
	RouteTopology::start_nodeid = start;
	RouteTopology::resetStates();

	distances.clear();
	prevs.clear();
	queue.clear();

	typename RouteTopology::RouteVertexMapIterator iter = RouteTopology::routeVertices.begin();
	typename RouteTopology::RouteVertexMapIterator iterEnd = RouteTopology::routeVertices.end();

	while (iter != iterEnd) {
		distances[iter->second] = std::numeric_limits<int>::max();
		prevs[iter->second] = (double)UNDEFINED;
		queue.push_back(iter->second);
		iter++;
	}

	V* vStart = RouteTopology::getVertex(start);
	distances[vStart] = 0;

	while (!queue.empty()) {
		V* current = getNodeWithMinDistanceFromStart();
		if (current == nullptr) {
			break;
		}

		current->visited(true);

		typename std::vector<V*>::iterator qIter;
		for (qIter = queue.begin(); qIter != queue.end(); qIter++) {
			if ((*qIter)->nodeid() == current->nodeid()) {
				queue.erase(qIter);
				break;
			}
		}

		setDistance(current);
	}

	if (!queue.empty()) {
		queue.clear();
	}
}

template<typename V, typename E>
double Dijkstra<V, E>::getShortestPathWithCache(double start, double goal, std::vector<double>& out)
{
	if (start != RouteTopology::start_nodeid) {
		throw dtsim::CityException(__func__, "start nodeid mismatch");
	}

	RouteTopology::goal_nodeid = goal;

	V* vSource = findPreviousVertex(start);
	V* vTarget = findPreviousVertex(goal);

	if ((vSource == nullptr) || (vTarget == nullptr)) {
		return 0;
	}

	/* At first, put the goal nodeid */
	out.insert(out.begin(), vTarget->nodeid());

	_distance = 0;
	while (RouteTopology::start_nodeid != vTarget->nodeid()) {
		double prev_nodeid = getPreviousNodeId(vTarget);
		double target_nodeid = vTarget->nodeid();

		if (prev_nodeid <= 0) {
			out.clear();
			return 0;
		} else {
			_distance += CalculateDistance(prev_nodeid, target_nodeid);

			vTarget = RouteTopology::getVertex(prev_nodeid);
			out.insert(out.begin(), vTarget->nodeid());
		}
	}

	if (out.front() != start) {
		out.clear();
		_distance = 0;
	}

	return _distance;
}

} /* namespace dtsim */

#endif /* CITY_ROUTE_TOPOLOGY_H_ */
