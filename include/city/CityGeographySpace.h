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
 * CityGeographySpace.h
 *
 * $Revision: 865 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_GEOGRAPHY_SPACE_H
#define CITY_GEOGRAPHY_SPACE_H

#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

#include "repast_hpc/Layer.h"
#include "repast_hpc/SharedGeography.h"
#include "repast_hpc/AgentId.h"
#include "geos/geom/Envelope.h"
#include "geos/geom/Coordinate.h"
#include "CityContext.h"
#include "CitySpace.h"
#include "CityAgent.h"
#include "CityAgentFactory.h"
#include "CityCoordinate.h"
#include "CityCoordinateSequence.h"
#include "CityGeometry.h"
#include "CityGeodeticCalculator.h"
#include "CityException.h"

namespace dtsim {

class City;

/**
 * \class CityGeographySpace
 *
 * \brief
 * Geography Space implementation
 *
 * CityGeographySpace contains RepastHPC Geography space
 */
class CityGeographySpace: public CitySpace {
private:
	/// RepastHPC SharedGeography space
	repast::SharedGeography<CityAgent>* geographySpace;

public:
	/**
	 * Creates a CityGeographySpace
	 *
	 * @param name geography space name
	 * @param context CityContext
	 */
	CityGeographySpace(std::string& name, CityContext* context);
	virtual ~CityGeographySpace();

	template<typename T>
	T* addAgent(T* agent);

	template<typename T>
	void removeAgent(T* agent);

	template<typename T>
	void deployAgent(T* agent, CityGeometry* cityGeom);

	template<typename T>
	void moveAgent(T* agent, CityGeometry* cityGeom);

	template<typename T>
	CityGeometry* moveAgent(T* agent, double x, double y);

	template<typename T>
	CityGeometry* moveAgent(T* agent, CityCoordinate& cityCoord);

	template<typename T>
	CityGeometry* moveAgent(T* agent, CityCoordinateSequence& cityCoordSeq);

	template<typename T>
	CityGeometry* moveAgentByDisplacement(T* agent, double xShift, double yShift);

	template<typename T>
	CityGeometry* moveAgentByVector(T* agent, double distance, double azimuth);

	template<typename T>
	void getAgentsWithin(CityGeometry* cityGeom, double distance, std::vector<T*>& agents);

	template<typename T>
	void getAgentsWithin(double x, double y, double distance, std::vector<T*>& agents);

	template<typename T>
	void getAgentsWithin(CityGeometry* cityGeom, double distance, std::vector<std::pair<double, T*>>& agents);

	template<typename T>
	void getAgentsWithin(double x, double y, double distance, std::vector<std::pair<double, T*>>& agents);

	template<typename T>
	T* getNearestAgent(CityGeometry* cityGeom);

	template<typename T>
	T* getNearestAgent(CityGeometry* cityGeom, double maxDistance);

	template<typename T>
	T* getNearestAgent(double x, double y);

	template<typename T>
	T* getNearestAgent(double x, double y, double maxDistance);

	/// Returns the number of agents in this geography
	std::size_t getSize();

	/// Returns the number of layer in this geography
	std::size_t getLayerSize();

	/// Returns all agents in this geography
	void getAllAgents(std::vector<CityAgent*>& agents);

	template<typename T>
	int getLayerAgentType();

	template<typename T>
	std::size_t getLayerAgentSize();

	template<typename T>
	void getLayerAgents(std::vector<T*>& agents);

	template<typename T>
	void writeToWKTFile(std::string fileName);

	template<typename T>
	void showQuadTree();
};

/**
 * Add agent to geography
 *
 * @param agent agent to add
 * @return agent object pointer
 */
template<typename T>
T* CityGeographySpace::addAgent(T* agent)
{
	return context->addAgent(agent);
}

/**
 * Remove agent from geography
 *
 * @param agent agent to remove
 */
template<typename T>
void CityGeographySpace::removeAgent(T* agent)
{
	context->removeAgent(agent);
}

/**
 * Initial deployment of agent to the specified geometry
 * If agent's geometry already exist, occur exception
 */
template<typename T>
void CityGeographySpace::deployAgent(T* agent, CityGeometry* cityGeom)
{
	CityGeometry* geom = agent->getGeometry();
	if (geom != nullptr)
		throw CityException(__func__, "geometry already exist");

	agent->setGeometry(cityGeom);

	spaceLock();
	geographySpace->moveTo(static_cast<CityAgent*>(agent), cityGeom->getGEOSgeometry());
	spaceUnlock();
}

/**
 * Moves the specified agent to the specified geometry.
 * If the geometry is null, then the agent remains in this geography but without a geometry.
 * The type of geometry must match the type currently associated with the layer.
 * For example, an agent cannot be located at a Point if the layer geometery type is a Polygon.
 *
 * @param agent agent to move
 * @param cityGeom CityGeometry to move agent
 */
template<typename T>
void CityGeographySpace::moveAgent(T* agent, CityGeometry* cityGeom)
{
	CityGeometry* geom = agent->getGeometry();
	geos::geom::Geometry* geosGeom;

	if (geom == nullptr) {
		if (cityGeom == nullptr)
			throw CityException(__func__, "null geometry");

		geosGeom = cityGeom->getGEOSgeometry();
		agent->setGeometry(cityGeom);
	} else {
		if (cityGeom == nullptr) {
			geosGeom = nullptr;
		} else {
			geosGeom = geom->getGEOSgeometry();
			geos::geom::GeometryTypeId geomTypeId = geosGeom->getGeometryTypeId();

			if (geomTypeId != cityGeom->getGEOSgeometry()->getGeometryTypeId())
				throw CityException(__func__, "unmatched geometry type");

			if (geomTypeId == geos::geom::GeometryTypeId::GEOS_POINT) {
				double x, y;
				cityGeom->getCoordinate(x, y);

				const CityCoordinateFilter filter(x, y);
				geosGeom->apply_rw(&filter);
			} else if ((geomTypeId == geos::geom::GeometryTypeId::GEOS_LINESTRING) ||
					   (geomTypeId == geos::geom::GeometryTypeId::GEOS_LINEARRING) ||
					   (geomTypeId == geos::geom::GeometryTypeId::GEOS_POLYGON)) {
				CityCoordinateSequence cityCoordSeq;
				cityGeom->getCoordinates(cityCoordSeq);

				CityCoordinateSequenceFilter filter(cityCoordSeq);
				geosGeom->apply_rw(filter);
			} else {
				throw CityException(__func__, "unknown geometry type");
			}
		}
	}

	spaceLock();
	geographySpace->moveTo(static_cast<CityAgent*>(agent), geosGeom);
	spaceUnlock();
}

/**
 * Move agent to the specified x, y coordinate
 * If agent's geometry already exist, exception occur
 *
 * @param agent agent to move
 * @param x x-coordinate to move
 * @param y y-coordinate to move
 * @return CityGeometry object pointer
 */
template<typename T>
CityGeometry* CityGeographySpace::moveAgent(T* agent, double x, double y)
{
	CityGeometry* cityGeom = agent->getGeometry();
	if (cityGeom == nullptr)
		throw CityException(__func__, "geometry not found");

	geos::geom::Geometry* geosGeom = cityGeom->getGEOSgeometry();
	geos::geom::GeometryTypeId geomTypeId = geosGeom->getGeometryTypeId();

	if (geomTypeId != geos::geom::GeometryTypeId::GEOS_POINT)
		throw CityException(__func__, "geometry type is not point");

	const CityCoordinateFilter filter(x, y);
	geosGeom->apply_rw(&filter);

	spaceLock();	
	geographySpace->moveTo(static_cast<CityAgent*>(agent), geosGeom);
	spaceUnlock();	

	return cityGeom;
}

/**
 * Move agent to the specified coordinate
 * If agent's geometry already exist, exception occur
 *
 * @param agent agent to move
 * @param cityCoord coordinate to move
 * @return CityGeometry object pointer
 */
template<typename T>
CityGeometry* CityGeographySpace::moveAgent(T* agent, CityCoordinate& cityCoord)
{
	CityGeometry* cityGeom = agent->getGeometry();
	if (cityGeom == nullptr)
		throw CityException(__func__, "geometry not found");

	geos::geom::Geometry* geosGeom = cityGeom->getGEOSgeometry();
	geos::geom::GeometryTypeId geomTypeId = geosGeom->getGeometryTypeId();

	if (geomTypeId != geos::geom::GeometryTypeId::GEOS_POINT)
		throw CityException(__func__, "geometry type is not point");

	const CityCoordinateFilter filter(cityCoord.x, cityCoord.y);
	geosGeom->apply_rw(&filter);

	spaceLock();	
	geographySpace->moveTo(static_cast<CityAgent*>(agent), geosGeom);
	spaceUnlock();	

	return cityGeom;
}

/**
 * Move agent to the specified coordinate sequence
 * If agent's geometry already exist, exception occur
 *
 * @param agent agent to move
 * @param cityCoordSeq coordinate sequence to move
 * @return CityGeometry object pointer
 */
template<typename T>
CityGeometry* CityGeographySpace::moveAgent(T* agent, CityCoordinateSequence& cityCoordSeq)
{
	CityGeometry* cityGeom = agent->getGeometry();
	if (cityGeom == nullptr)
		throw CityException(__func__, "geometry not found");

	geos::geom::Geometry* geosGeom = cityGeom->getGEOSgeometry();
	geos::geom::GeometryTypeId geomTypeId = geosGeom->getGeometryTypeId();

	if ((geomTypeId != geos::geom::GeometryTypeId::GEOS_LINESTRING) &&
		(geomTypeId != geos::geom::GeometryTypeId::GEOS_LINEARRING) &&
		(geomTypeId != geos::geom::GeometryTypeId::GEOS_POLYGON))
		throw CityException(__func__, "invalid geometry type");

	CityCoordinateSequenceFilter filter(cityCoordSeq);
	geosGeom->apply_rw(filter);

	spaceLock();	
	geographySpace->moveTo(static_cast<CityAgent*>(agent), geosGeom);
	spaceUnlock();	

	return cityGeom;
}

/**
 * Displaces the specified object by the specified lon and lat amount.
 * If agent's geometry already exist, exception occur
 * 
 * @param agent agent to move
 * @param xShift the amount to move longitudinaly
 * @param yShift the amount to move latitudinaly
 * @return the new geometry of the object
 */
template<typename T>
CityGeometry* CityGeographySpace::moveAgentByDisplacement(T* agent, double xShift, double yShift)
{
	CityGeometry* cityGeom = agent->getGeometry();
	if (cityGeom == nullptr)
		throw CityException(__func__, "geometry not found");

	geos::geom::Geometry* geosGeom = cityGeom->getGEOSgeometry();
	geos::geom::GeometryTypeId geomTypeId = geosGeom->getGeometryTypeId();
	double x, y;

	if (geomTypeId == geos::geom::GeometryTypeId::GEOS_POINT) {
		x = cityGeom->getCoordinate()->x + xShift;
		y = cityGeom->getCoordinate()->y + yShift;

		const CityCoordinateFilter filter(x, y);
		geosGeom->apply_rw(&filter);
	} else if ((geomTypeId == geos::geom::GeometryTypeId::GEOS_LINESTRING) ||
			   (geomTypeId == geos::geom::GeometryTypeId::GEOS_LINEARRING) ||
			   (geomTypeId == geos::geom::GeometryTypeId::GEOS_POLYGON)) {
		CityCoordinateSequence cityCoordSeq;
		cityGeom->getCoordinates(cityCoordSeq);

		for (std::size_t i = 0, n = cityCoordSeq.getSize(); i < n; i++) {
			cityCoordSeq.getAt(i, x, y);
			x += xShift;
			y += yShift;

			cityCoordSeq.setAt(x, y, i);
		}

		CityCoordinateSequenceFilter filter(cityCoordSeq);
		geosGeom->apply_rw(filter);
	} else {
		throw CityException(__func__, "unknown geometry type");
	}

	spaceLock();	
	geographySpace->moveTo(static_cast<CityAgent*>(agent), geosGeom);
	spaceUnlock();	

	return cityGeom;
}

/**
 * Moves the specified object the specified distance along the specified angle.
 * If agent's geometry already exist, exception occur
 * 
 * @param agent agent to move
 * @param distance the distance to move in meters
 * @param azimuth the angle along which to move.
 * @return the geometric location the object was moved to
 */
template<typename T>
CityGeometry* CityGeographySpace::moveAgentByVector(T* agent, double distance, double azimuth)
{
	if ((azimuth > 360) || (azimuth < 0))
		throw CityException(__func__, "invalid azimuth");

	CityGeometry* cityGeom = agent->getGeometry();
	if (cityGeom == nullptr)
		throw CityException(__func__, "geometry not found");

	geos::geom::Geometry* geosGeom = cityGeom->getGEOSgeometry();
	geos::geom::GeometryTypeId geomTypeId = geosGeom->getGeometryTypeId();

	double startX, startY;
	double destX, destY;

	if (geomTypeId == geos::geom::GeometryTypeId::GEOS_POINT) {
		startX = cityGeom->getCoordinate()->x;
		startY = cityGeom->getCoordinate()->y;
		CityGeodeticCalculator::instance()->getDestinationPoint(startX, startY, distance, azimuth, destX, destY);

		const CityCoordinateFilter filter(destX, destY);
		geosGeom->apply_rw(&filter);
	} else if ((geomTypeId == geos::geom::GeometryTypeId::GEOS_LINESTRING) ||
			   (geomTypeId == geos::geom::GeometryTypeId::GEOS_LINEARRING) ||
			   (geomTypeId == geos::geom::GeometryTypeId::GEOS_POLYGON)) {
		CityCoordinateSequence cityCoordSeq;
		cityGeom->getCoordinates(cityCoordSeq);

		for (std::size_t i = 0, n = cityCoordSeq.getSize(); i < n; i++) {
			cityCoordSeq.getAt(i, startX, startY);

			CityGeodeticCalculator::instance()->getDestinationPoint(startX, startY, distance, azimuth, destX, destY);

			cityCoordSeq.setAt(destX, destY, i);
		}

		CityCoordinateSequenceFilter filter(cityCoordSeq);
		geosGeom->apply_rw(filter);
	} else {
		throw CityException(__func__, "unknown geometry type");
	}

	spaceLock();	
	geographySpace->moveTo(static_cast<CityAgent*>(agent), geosGeom);
	spaceUnlock();

	return cityGeom;
}

/**
 * Get agents that are within the specified distance from start x and y point
 *
 * @param startX start X point
 * @param startY start Y point
 * @param distance distance range
 * @param agents returned vector of agents
 * @return return vector of agents
 */
template<typename T>
void CityGeographySpace::getAgentsWithin(double startX, double startY, double distance, std::vector<T*>& agents)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	repast::Layer<CityAgent>* layer = geographySpace->getLayer(type);
	if (layer == nullptr)
		return;

	double destX, destY;
	double x1, y1;
	double x2, y2;

	CityGeodeticCalculator::instance()->getDestinationPoint(startX, startY, distance, 0, destX, destY);
	y1 = destY;

	CityGeodeticCalculator::instance()->getDestinationPoint(startX, startY, distance, 90, destX, destY);
	x1 = destX;

	CityGeodeticCalculator::instance()->getDestinationPoint(startX, startY, distance, 180, destX, destY);
	y2 = destY;

	CityGeodeticCalculator::instance()->getDestinationPoint(startX, startY, distance, 270, destX, destY);
	x2 = destX;

	const geos::geom::Envelope* envelope = new geos::geom::Envelope(x1, x2, y1, y2);

	std::vector<CityAgent*> cityAgents;
	layer->getAgentsWithin(envelope, cityAgents);

	std::vector<CityAgent*>::iterator iter = cityAgents.begin();
	std::vector<CityAgent*>::iterator iterEnd = cityAgents.end();
	while (iter != iterEnd) {
		agents.push_back(static_cast<T*>(*iter));
		iter++;
	}

	delete envelope;
}

/**
 * Get agents that are within the specified distance from the centroid of the geometry
 *
 * @param cityGeom search agents within distance from the centroid of CityGeometry
 * @param distance distance range
 * @param agents returned vector of agents
 * @return return vector of agents
 */
template<typename T>
void CityGeographySpace::getAgentsWithin(CityGeometry* cityGeom, double distance, std::vector<T*>& agents)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	double startX, startY;
	if (cityGeom->getCentroid(startX, startY) == false)
		return;

	getAgentsWithin(startX, startY, distance, agents);
}

/**
 * Get agents that are within the specified distance from specified start x and y point
 * Returned agents is sorted by distance
 *
 * @param startX start X point
 * @param startY start Y point
 * @param distance distance range
 * @param agents returned ordered vector of agents
 * @return return vector of pair(distance, agent). pair(distance, agent) is sorted by distance
 */
template<typename T>
void CityGeographySpace::getAgentsWithin(double startX, double startY, double distance, std::vector<std::pair<double, T*>>& agents)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	std::vector<T*> cityAgents;
	getAgentsWithin(startX, startY, distance, cityAgents);
	if (cityAgents.empty())
		return;

	double destX;
	double destY;
	double length;

	typename std::vector<T*>::iterator iter = cityAgents.begin();
	typename std::vector<T*>::iterator iterEnd = cityAgents.end();
	while (iter != iterEnd) {
		destX = (*iter)->getGeometry()->getCoordinate()->x;
		destY = (*iter)->getGeometry()->getCoordinate()->y;
		length = CityGeodeticCalculator::instance()->getDistance(startX, startY, destX, destY);

		if (length <= distance)
			agents.push_back(std::make_pair(length, static_cast<T*>(*iter)));

		iter++;
	}

	std::sort(agents.begin(), agents.end());
}

/**
 * Get agents that are within the specified distance from the centroid of the geometry
 * Returned agents is sorted by distance
 *
 * @param cityGeom search agents within distance from the centroid of CityGeometry
 * @param distance distance range
 * @param agents returned ordered vector of agents
 * @return return vector of pair(distance, agent). pair(distance, agent) is sorted by distance
 */
template<typename T>
void CityGeographySpace::getAgentsWithin(CityGeometry* cityGeom, double distance, std::vector<std::pair<double, T*>>& agents)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	double startX, startY;
	if (cityGeom->getCentroid(startX, startY) == false)
		return;

	getAgentsWithin(startX, startY, distance, agents);
}

/**
 * Get nearest agent from specified start x and y point
 *
 * @param startX start X point
 * @param startY start Y point
 * @return return nearest agent
 */
template<typename T>
T* CityGeographySpace::getNearestAgent(double startX, double startY)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return nullptr;

	double distance = 200;
	std::vector<std::pair<double, T*>> agents;

	for (std::size_t i = 1; i <= 100; i++) {
		getAgentsWithin(startX, startY, i*distance, agents);
		if (!agents.empty()) {
			return agents[0].second;
		}
	}

	return nullptr;
}

/**
 * Get nearest agent from specified start x and y point within the specified max distance
 *
 * @param startX start X point
 * @param startY start Y point
 * @param maxDistance max distance from specified start x and y point
 * @return return nearest agent
 */
template<typename T>
T* CityGeographySpace::getNearestAgent(double startX, double startY, double maxDistance)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return nullptr;

	std::vector<std::pair<double, T*>> agents;

	getAgentsWithin(startX, startY, maxDistance, agents);
	if (!agents.empty())
		return agents[0].second;

	return nullptr;
}

/**
 * Get nearest agent from the centroid of the geometry
 *
 * @param cityGeom search agents within distance from the centroid of CityGeometry
 * @return return nearest agent
 */
template<typename T>
T* CityGeographySpace::getNearestAgent(CityGeometry* cityGeom)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return nullptr;

	double startX, startY;
	if (cityGeom->getCentroid(startX, startY) == false)
		return nullptr;

	double distance = 200;
	std::vector<std::pair<double, T*>> agents;

	for (std::size_t i = 1; i <= 100; i++) {
		getAgentsWithin(startX, startY, i*distance, agents);
		if (!agents.empty()) {
			return agents[0].second;
		}
	}

	return nullptr;
}

/**
 * Get nearest agent from the centroid of the geometry within specified max distance
 *
 * @param cityGeom search agents within distance from the centroid of CityGeometry
 * @param maxDistance max distance from specified geometry
 * @return return nearest agent
 */
template<typename T>
T* CityGeographySpace::getNearestAgent(CityGeometry* cityGeom, double maxDistance)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return nullptr;

	double startX, startY;
	if (cityGeom->getCentroid(startX, startY) == false)
		return nullptr;

	std::vector<std::pair<double, T*>> agents;

	getAgentsWithin(startX, startY, maxDistance, agents);
	if (!agents.empty())
		return agents[0].second;

	return nullptr;
}

/**
 * Returns the type of layer
 */
template<typename T>
int CityGeographySpace::getLayerAgentType()
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return -1;

	repast::Layer<CityAgent>* layer = geographySpace->getLayer(type);
	if (layer == nullptr)
		return -1;

	return type;
}

/**
 * Returns the number of agents in specified type's layer
 */
template<typename T>
std::size_t CityGeographySpace::getLayerAgentSize()
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return 0;

	repast::Layer<CityAgent>* layer = geographySpace->getLayer(type);
	if (layer == nullptr)
		return 0;

	return layer->getSize();
}

/**
 * Get Returns all agents in specified type's layer
 *
 * @param agents returned vector of agents
 * @return vector of agents of given the type
 */
template<typename T>
void CityGeographySpace::getLayerAgents(std::vector<T*>& agents)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	repast::Layer<CityAgent>* layer = geographySpace->getLayer(type);
	if (layer == nullptr)
		return;

	std::vector<CityAgent*> cityAgents;
	layer->getAgents(cityAgents);

	std::vector<CityAgent*>::iterator iter = cityAgents.begin();
	std::vector<CityAgent*>::iterator iterEnd = cityAgents.end();
	while (iter != iterEnd) {
		agents.push_back(static_cast<T*>(*iter));
		iter++;
	}
}

/**
 * Write agent's geometry to file as a WKT fommat
 *
 * @param fileName file name
 */
template<typename T>
void CityGeographySpace::writeToWKTFile(std::string fileName)
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	repast::Layer<CityAgent>* layer = geographySpace->getLayer(type);
	if (layer == nullptr)
		return;

	std::vector<CityAgent*> cityAgents;
	layer->getAgents(cityAgents);

	if (cityAgents.empty() == false) {
		std::vector<CityAgent*>::iterator iter = cityAgents.begin();
		std::vector<CityAgent*>::iterator iterEnd = cityAgents.end();

		CityGeometry* cityGeom = (*iter)->getGeometry();
		geos::geom::Geometry* geosGeom = cityGeom->getGEOSgeometry();
		geos::geom::GeometryTypeId geomTypeId = geosGeom->getGeometryTypeId();
		T* agent;

		std::ofstream file(fileName);

		while (iter != iterEnd) {
			agent = static_cast<T*>(*iter);

			if (geomTypeId == geos::geom::GeometryTypeId::GEOS_POINT) {
				file << "POINT (" << agent->getCoordinate()->toString() << ")" << std::endl;
			} else if (geomTypeId == geos::geom::GeometryTypeId::GEOS_LINEARRING) {
				file << "LINEARRING" << agent->getCoordinates()->toString() << std::endl;
			} else if (geomTypeId == geos::geom::GeometryTypeId::GEOS_LINESTRING) {
				file << "LINESTRING " << agent->getCoordinates()->toString() << std::endl;
			} else if (geomTypeId == geos::geom::GeometryTypeId::GEOS_POLYGON) {
				file << "POLYGON (" << agent->getCoordinates()->toString() << ")" << std::endl;
			}
			iter++;
		}

		file.close();
	}
}

template<typename T>
void CityGeographySpace::showQuadTree()
{
	int type = CityAgentFactory::instance()->findAgentType<T>();
	if (type < 0)
		return;

	repast::Layer<CityAgent>* layer = geographySpace->getLayer(type);
	if (layer == nullptr)
		return;

	layer->showQuadTree();
}

} /* namespace dtsim */

#endif /* CITY_GEOGRAPHY_SPACE_H */
