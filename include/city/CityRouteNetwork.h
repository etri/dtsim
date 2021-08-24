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
 * CityRouteNetwork.h
 *
 * $Revision: $
 * $LastChangedDate: $
 */

#ifndef CITY_ROUTE_NETWORK_H_
#define CITY_ROUTE_NETWORK_H_

#include <vector>
#include <unordered_map>
#include <memory>
#include <queue>

#include "CitySpace.h"
#include "CityJunction.h"
#include "CityRoad.h"
#include "CityGeometry.h"
#include "CityCoordinate.h"
#include "CityRouter.h"
#include "CityRouteCache.h"

namespace dtsim {

class Edge;

/**
 * \class Vertex
 *
 * \brief
 * Vertex represents a junction in the route network.
 */
class Vertex {

private:
	/// a numeric identifer read from the NODE shape file
	double _nodeid;

	/// geographic coordinates
	double _latitude, _longitude;

	/// uses to filter out what have visited to choose its successors
	bool _visited;

	/// uses for only A* to recalculate the distance to this vertex
	bool _moved;

	/// uses for only A* to filter out what have passed to get a shortest path
	bool _passed;

	// all edges to this vertex
	std::vector<Edge*> 	_edges;

	/// vertices connected to edges outgoing from this vertex
	std::vector<Vertex*> _successors;

	/// vertices connected to edges incoming to this vertex
	std::vector<Vertex*> _predecessors;

public:
	/**
	 * Constructor
	 *
	 * @param nodeid the numeric identifier of a junction represened by this vertex
	 * @param latitude the X-axis value
	 * @param longitude the Y-axis value
	 */
	Vertex(double nodeid, double latitude, double longitude);

	~Vertex() {}

	/// Returns the node identifer
	double nodeid() const {
		return _nodeid;
	}

	/// Returns the X-axis value
	double getLatitude() const {
		return _latitude;
	}

	/// Returns the Y-axis value
	double getLongitude() const {
		return _longitude;
	}

	/// Returns true if two coorindates are same or false if different
	bool isSameCoordinate(double latitude, double longitude) {
		return (_latitude == latitude && _longitude == longitude);
	}

	/// Returns the state whether it has visited
	bool visited() const {
		return _visited;
	}

	void visited(bool value) {
		_visited = value;
	}

	/// Returns the state whether it has moved to the open list
	bool moved() const {
		return _moved;
	}

	void moved(bool value) {
		_moved = value;
	}

	/// Returns the state whether it has passed
	bool passed() const {
		return _passed;
	}

	void passed(bool value) {
		_passed = value;
	}

	/// Returns the list of edges connected to this
	std::vector<Edge*> edges() const {
		return _edges;
	}

	int edgeCount() const {
		return _edges.size();
	}

	/// Adds an input vertex to the list of successors
	void putSuccessors(Vertex* v) {
		_successors.push_back(v);
	}

	/// Adds an vertex to the list of predecessors
	void putPredecessors(Vertex* v) {
		_predecessors.push_back(v);
	}

	/// Returns the successors of this vertex 
	void getSuccessors(std::vector<Vertex*>& out) const;

	/// Returns the predecessors of this vertex 
	void getPredecessors(std::vector<Vertex*>& out) const;

	/// Retrieves the edge from this to the specified vertex
	Edge* findEdge(Vertex* target);

	/// Adds the specified edge to the list of edges 
	void doAddEdge(Edge* e);
};

/** 
 * \class Edge
 *
 * \brief
 * Edge represents a road in the route network. 
 */
class Edge {

private:
	Vertex* _source;
	Vertex* _target;

	// an edge identifier
	double _edgeid;

	/// a distance from source to target
	double _weight;

	/// means whether it has passed or not
	bool _visited;

public:
	/** Constructor
	 *
	 * @param source the source vertex
	 * @param target the target vertex
	 * @param edgeId the edge identifier
	 */
	Edge(Vertex* source, Vertex* target, double edgeId);

	/** Constructor
	 *
	 * @param source the source vertex
	 * @param target the target vertex
	 * @param edgeId the edge identifier
	 * @param weight the distance from soruce to target
	 */
	Edge(Vertex* source, Vertex* target, double edgeId, double weight);

	~Edge() {}

	/// Returns the edge ID
	double getEdgeId() const {
		return _edgeid;
	}

	/// Sets the edge ID
	void setEdgeId(double edgeId) {
		_edgeid = edgeId;
	}

	/// Returns the source vertex
	Vertex* source() const {
		return _source;
	}

	/// Returns the target vertex
	Vertex* target() const {
		return _target;
	}

	/// Sets the source vertex
	void source(Vertex* vSource) {
		_source = vSource;
	}

	/// Sets the target vertex
	void target(Vertex* vTarget) {
		_target = vTarget;
	}

	/// Returns the weight
	double weight() const {
		return _weight;
	}

	/// Sets the weight
	void weight(double wgt) {
		_weight = wgt;
	}

	/// Returns the state whether it has visited or not
	bool visited() const {
		return _visited;
	}

	/// Sets the state to the specified value
	void visited(bool value) {
		_visited = value;
	}
};

/** 
 * \class CityRouteNetwork
 *
 * \brief
 * Road network implementation
 *
 * CityRouteNetwork describes a road network that is composed of junctions and roads.
 * Junction is depicted as a vertex and Road as an edge. It is used to get the shortest path 
 * between the specified vertices
 */
class CityRouteNetwork: public CitySpace {

private:
	typedef typename std::unordered_map<double, std::pair<Vertex*, CityJunction*>> VerticesMap;
	typedef typename VerticesMap::iterator VerticesMapIterator;

	/// all vertices on this network
	VerticesMap vertices;

	/// all edges on this network
	std::vector<Edge*> edges;
	std::unordered_map<double, Edge*> edgeMap;

	/// CityRouteCache object
	CityRouteCache* cacheMap;

	/// parallel CityRouters
	std::vector<CityRouter<Vertex, Edge>*> routers;

	std::mutex qLock;
	std::queue<CityRouter<Vertex, Edge>*> waitQ;

public:
	static std::string DEFAULT_SEARCH_ALGORITHM;

	/**
 	 * Creates a CityRouteNetwork consisting of the specified vertices and edges
 	 *
 	 * @param name road network name
 	 * @param context CityContext
 	 */
	CityRouteNetwork(std::string& name, CityContext* context);

	~CityRouteNetwork(); 

	void build(std::vector<CityJunction*>& junctions, std::vector<CityRoad*>& roads);

	Edge* findEdge(CityJunction* source, CityJunction* target);

	/// Update the weight of the specified edge
	void updateEdge(double edgeId, double newWeight);

	CityRouter<Vertex, Edge>* getRouter();
	void putRouter(CityRouter<Vertex, Edge>* r);

	/// Gets the shortest path between start and goal junctions and puts them into out
	void getShortestPath(double startJunctionId, double endJunctionId, std::vector<CityJunction*>& junctions);

	void getShortestPath(double startJunctionId, double endJunctionId, std::vector<CityRoad*>& roads);

	void getShortestPath(double startJunctionId, double endJunctionId, std::vector<std::unique_ptr<CityCoordinateSequence>>& roadCoordSeqs, std::vector<std::vector<double>>& roadDistances);

	void getShortestPath(double startJunctionId, double endJunctionId, std::vector<std::pair<double, std::pair<std::unique_ptr<CityCoordinateSequence>, std::vector<double>>>>& roadDistances);

	// debug
	void printPath(double start, double goal, std::vector<CityJunction*>& paths);
};

} /* namespace */

#endif /* CITY_ROUTE_NETWORK_H_ */
