/*
 *  SharedGeography.h
 */

#ifndef SHAREDGEOGRAPHY_H_
#define SHAREDGEOGRAPHY_H_

#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/iterator.hpp>

#include "AgentId.h"
#include "Geography.h"
#include "Layer.h"
#include "geos/geom/Geometry.h"
#include "geos/geom/Envelope.h"

namespace repast {

/**
 * SharedGeography implementation in RepastHPC
 */
template<typename T>
class SharedGeography: public Geography<T> {
private:
	typedef typename boost::unordered_map<int, Layer<T>*> LayerMap;
	typedef typename LayerMap::iterator LayerMapIter;
	LayerMap layerMap;

	int rank;
	boost::mpi::communicator* comm;

public:
	SharedGeography(std::string name, boost::mpi::communicator* communicator);
	virtual ~SharedGeography();

	Layer<T>* getLayer(int type);
	Layer<T>* addLayer(int type);

	std::size_t getSize();
	std::size_t getLayerSize();

	bool addAgent(boost::shared_ptr<T> agent);
	void removeAgent(T* agent);
	void moveTo(T* agent, geos::geom::Geometry* geom);

	void getAllAgents(std::vector<T*>& agents);
};

template<typename T>
SharedGeography<T>::SharedGeography(std::string name, boost::mpi::communicator* communicator)
: Geography<T>(name), comm(communicator)
{
	rank = comm->rank();
}

template<typename T>
SharedGeography<T>::~SharedGeography()
{
	LayerMapIter layerMapIter = layerMap.begin();
	while (layerMapIter != layerMap.end()) {
		Layer<T>* layer = layerMapIter->second;
		layerMapIter = layerMap.erase(layerMapIter);
		delete layer;
	}
}

/**
 * Add layer of agents
 */
template<typename T>
Layer<T>* SharedGeography<T>::addLayer(int type)
{
	Layer<T>* layer = new Layer<T>();
	layerMap[type] = layer;

	return layer;
}

/**
 * Gets layer of specified type
 */
template<typename T>
Layer<T>* SharedGeography<T>::getLayer(int type)
{
	LayerMapIter layerMapIter = layerMap.find(type);
	if (layerMapIter != layerMap.end())
		return layerMapIter->second;
	else
		return nullptr;
}

/**
 * Returns the number of agents in this geography
 */
template<typename T>
std::size_t SharedGeography<T>::getSize()
{
	std::size_t size = 0;

	LayerMapIter layerMapIter = layerMap.begin();
	while (layerMapIter != layerMap.end()) {
		size += layerMapIter->second->getSize();
		layerMapIter++;
	}
	
	return size;
}

/**
 * Returns the number of layer in this geography
 */
template<typename T>
std::size_t SharedGeography<T>::getLayerSize()
{
	return layerMap.size();
}

/**
 * Add agent to geography
 */
template<typename T>
bool SharedGeography<T>::addAgent(boost::shared_ptr<T> agent)
{
	if (!Projection<T>::agentCanBeAdded(agent))
		return false;

	int type = agent->getId().agentType();

	LayerMapIter layerMapIter = layerMap.find(type);
	if (layerMapIter == layerMap.end())
		addLayer(type);

	return true;
}

/**
 * Remove agent from geography
 */
template<typename T>
void SharedGeography<T>::removeAgent(T* agent)
{
	LayerMapIter layerMapIter = layerMap.find(agent->getId().agentType());
	if (layerMapIter != layerMap.end())
		layerMapIter->second->removeAgent(agent);
}

/**
 * Moves the specified object to the specified location. If the location is
 * null then the object remains "in" this projection but without a location.
 * 
 * @param agent the agent to move
 * @param geom the location to move the agent to
 */
template<typename T>
void SharedGeography<T>::moveTo(T* agent, geos::geom::Geometry* geom)
{
	Layer<T>* layer;
	int type = agent->getId().agentType();

	LayerMapIter layerMapIter = layerMap.find(type);
	if (layerMapIter == layerMap.end())
		layer = addLayer(type);
	else
		layer = layerMapIter->second;

	layer->addAgent(agent, geom);
}

template<typename T>
void SharedGeography<T>::getAllAgents(std::vector<T*>& agents)
{
	LayerMapIter layerMapIter = layerMap.begin();
	while (layerMapIter != layerMap.end()) {
		layerMapIter->second->getAgents(agents);
		layerMapIter++;
	}
}

}

#endif /* SHAREDGEOGRAPHY_H_ */
