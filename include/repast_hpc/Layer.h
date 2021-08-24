/*
 *  Layer.h
 */

#ifndef LAYER_H_
#define LAYER_H_

#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/iterator.hpp>

#include "AgentId.h"
#include "geos/geom/Geometry.h"
#include "geos/geom/GeometryFactory.h"
#include "geos/geom/Envelope.h"
#include "geos/index/ItemVisitor.h"
#include "geos/index/quadtree/Quadtree.h"

namespace repast {

template<typename T>
class Layer;

/**
 * WithinItemVisitor derived from GEOS ItemVisitor
 * A visitor for items in an index
 */
template<typename T>
class WithinItemVisitor: public geos::index::ItemVisitor {
private:
	Layer<T>* layer;
	geos::geom::Geometry* withinGeom;
	std::vector<T*>& agents;

public:
	WithinItemVisitor(Layer<T>* layer_,
					  const geos::geom::Envelope* env_,
					  std::vector<T*>& agents_);
	virtual ~WithinItemVisitor();

	void visitItem(void* item);
};

template<typename T>
WithinItemVisitor<T>::WithinItemVisitor(Layer<T>* layer_,
										const geos::geom::Envelope* env_,
										std::vector<T*>& agents_)
: layer(layer_), agents(agents_)
{
	const geos::geom::GeometryFactory* geomFactory = geos::geom::GeometryFactory::getDefaultInstance();
	withinGeom = geomFactory->toGeometry(env_).release();
}

template<typename T>
WithinItemVisitor<T>::~WithinItemVisitor()
{
	delete withinGeom;
}

template<typename T>
void WithinItemVisitor<T>::visitItem(void* item)
{
	T* agent = static_cast<T*>(item);

	typename Layer<T>::AgentGeometryMapIter it = layer->agentGeometryMap.find(agent->getId());
	if (it != layer->agentGeometryMap.end()) {
		geos::geom::Geometry* geom = it->second->geom;
		if (geom->within(withinGeom))
			agents.push_back(agent);
	}
}

/**
 *
 */
template<typename T>
class GeometryData {
public:
	const geos::geom::Envelope* env;
	geos::geom::Geometry* geom;
	T* agent;

	GeometryData();
	virtual ~GeometryData();
};

template<typename T>
GeometryData<T>::GeometryData()
{}

template<typename T>
GeometryData<T>::~GeometryData()
{
	delete env;
}

/**
 * Layer implementation
 */
template<typename T>
class Layer {
private:
	typedef typename boost::unordered_map<AgentId, GeometryData<T>*, HashId> AgentGeometryMap;
	typedef typename AgentGeometryMap::iterator AgentGeometryMapIter;
	AgentGeometryMap agentGeometryMap;

	geos::index::quadtree::Quadtree* index;

	friend class WithinItemVisitor<T>;

public:
	Layer();
	virtual ~Layer();

	std::size_t getSize();

	void addAgent(T* agent, geos::geom::Geometry* geom);
	void removeAgent(T* agent);

	void getAgentsWithin(const geos::geom::Envelope* env, std::vector<T*>& agents);
	void getAgents(std::vector<T*>& agents);

	void showQuadTree();
};

template<typename T>
Layer<T>::Layer()
{
	index = new geos::index::quadtree::Quadtree();
}

template<typename T>
Layer<T>::~Layer()
{
	AgentGeometryMapIter iter = agentGeometryMap.begin();
	while (iter != agentGeometryMap.end()) {
		GeometryData<T>* geomData = iter->second;
		index->remove(geomData->env, geomData->agent);
		iter = agentGeometryMap.erase(iter);
		delete geomData;
	}

	delete index;
}

template<typename T>
std::size_t Layer<T>::getSize()
{
	return agentGeometryMap.size();
}

template<typename T>
void Layer<T>::addAgent(T* agent, geos::geom::Geometry* geom)
{
	GeometryData<T>* geomData;
	AgentGeometryMapIter agentGeometryMapIter;
	AgentId agentId = agent->getId();

	if (geom == nullptr) {
		agentGeometryMapIter = agentGeometryMap.find(agentId);
		if (agentGeometryMapIter != agentGeometryMap.end()) {
			geomData = agentGeometryMapIter->second;
			index->remove(geomData->env, agent);
			agentGeometryMap.erase(agentGeometryMapIter);
			delete geomData->geom;
			delete geomData;
		}
		return;
	}

	agentGeometryMapIter = agentGeometryMap.find(agentId);
	if (agentGeometryMapIter != agentGeometryMap.end()) {
		geomData = agentGeometryMapIter->second;
		index->remove(geomData->env, agent);
		delete geomData->env;
	} else {
		geomData = new GeometryData<T>();
		agentGeometryMap[agentId] = geomData;

		geomData->agent = agent;
	}

	geom->geometryChanged();
	const geos::geom::Envelope* env = geom->getEnvelopeInternal();

#if 0
	const geos::geom::Envelope* envelope = new geos::geom::Envelope(*env);
#else
	double minx = env->getMinX();
	double maxx = env->getMaxX();
	double miny = env->getMinY();
	double maxy = env->getMaxY();

	if (minx == maxx) {
		double minExtent = 0.001;
		double delta = env->getWidth();
		if ((delta < minExtent) && (delta > 0.0)) {
			minExtent = delta;
		}

		minx = minx - minExtent / 2.0;
		maxx = maxx + minExtent / 2.0;
	}

	if (miny == maxy) {
		double minExtent = 0.001;
		double delta = env->getHeight();
		if ((delta < minExtent) && (delta > 0.0)) {
			minExtent = delta;
		}

		miny = miny - minExtent / 2.0;
		maxy = maxy + minExtent / 2.0;
	}

	const geos::geom::Envelope* envelope = new geos::geom::Envelope(minx, maxx, miny, maxy);
#endif

	geomData->env = envelope;
	geomData->geom = geom;

	index->insert(geomData->env, agent);
}

template<typename T>
void Layer<T>::removeAgent(T* agent)
{
	AgentId agentId = agent->getId();

	AgentGeometryMapIter iter = agentGeometryMap.find(agentId);
	if (iter != agentGeometryMap.end()) {
		GeometryData<T>* geomData = iter->second;
		index->remove(geomData->env, agent);
		agentGeometryMap.erase(iter);
		delete geomData;
	}
}

template<typename T>
void Layer<T>::getAgents(std::vector<T*>& agents)
{
	AgentGeometryMapIter iter = agentGeometryMap.begin();
	while (iter != agentGeometryMap.end()) {
		agents.push_back(iter->second->agent);
		iter++;
	}
}

/**
 * Get agents within envelope from thi layer
 */
template<typename T>
void Layer<T>::getAgentsWithin(const geos::geom::Envelope* env, std::vector<T*>& agents)
{
	WithinItemVisitor<T> visitor(this, env, agents);

	index->query(env, visitor);
}

template<typename T>
void Layer<T>::showQuadTree()
{
	index->showQuadTree();
}

}

#endif /* LAYER_H_ */
