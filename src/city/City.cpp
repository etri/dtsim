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
 * City.cpp
 *
 * $Revision: 959 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <sys/resource.h>

#include "repast_hpc/RepastProcess.h"
#include "repast_hpc/initialize_random.h"
#include "City.h"
#include "CityContext.h"
#include "CityGeographySpace.h"
#include "CityGridSpace.h"
#include "CityNetworkSpace.h"
#include "CityArea.h"
#include "CityBuilding.h"
#include "CityBusNetwork.h"
#include "CityJunction.h"
#include "CityParkingLot.h"
#include "CityRoad.h"
#include "CityAgentFactory.h"
#include "CityCoordinate.h"
#include "CityCoordinateSequence.h"
#include "CityGeometry.h"
#include "CityGeometryFactory.h"
#include "CityCSVFileReader.h"
#include "CityShapefileReader.h"
#include "CityException.h"
#include "CityUtils.h"

namespace dtsim {

boost::mpi::environment* City::env = 0;
City* City::instance_ = 0;

std::string City::DEFAULT_MODEL_PROPERTY_FILE  = "model.props";
std::string City::DEFAULT_AGENT_GEOGRAPHY = "Default_Agent_Geography";
std::string City::DEFAULT_ENVIRONMENT_GEOGRAPHY = "Default_Environment_Geography";
std::string City::DEFAULT_BUS_NETWORK = "Default_Bus_Network";
std::string City::DEFAULT_ROUTE_NETWORK = "Default_Route_Network";

City::City(std::string modelPropsFile)
: comm(0), context(0)
{
	std::string configPropsFile;

	repast::RepastProcess::init(configPropsFile, comm);
	comm = repast::RepastProcess::instance()->getCommunicator();

	props = new repast::Properties(modelPropsFile, comm);

	repast::initializeRandom(*props, comm);
	srand(time(NULL));

	context = new CityContext(comm);

	int numWorkers = 1;
	if (containProperty("schedule.worker.threads"))
		numWorkers = getIntProperty("schedule.worker.threads");

	scheduler = new CityScheduler(comm, numWorkers);

	if (!containProperty("schedule.stop.at"))
		throw CityException(__func__, "not found 'schedule.stop.at' in property file");

	if (getDoubleProperty("schedule.stop.at") <= 0)
		throw CityException(__func__, "schedule.stop.at should be greater than 0");

	scheduler->addStopEvent(getDoubleProperty("schedule.stop.at"));

	/// set max number of open descriptor
	{
		struct rlimit limit;

		limit.rlim_cur = 65536;
		limit.rlim_max = 65536;
		setrlimit(RLIMIT_NOFILE, &limit);
	}
}

City::~City()
{
	scheduler->stop();

	delete props;
	delete context;
	delete scheduler;
	delete buildingMap;

	instance_ = 0;
}

City* City::instance()
{
	try {
		if (instance_ == 0)
			throw CityException(__func__, "City must be initialized before calling instance");
	} catch (CityException& e) {
		throw e;
	}

	return instance_;
}

void City::init(std::string modelPropsFile)
{
	try {
		if (instance_ != 0)
			throw CityException(__func__, "City was already initialized");

		if (modelPropsFile.empty())
			modelPropsFile = DEFAULT_MODEL_PROPERTY_FILE;

		std::ifstream fs(modelPropsFile);
		if (!fs.is_open())
			throw CityException(__func__, "not found model property file");
		fs.close();

		env = new boost::mpi::environment();

		instance_ = new City(modelPropsFile);
	} catch (CityException& e) {
		throw e;
	}
}

void City::buildEnvironment()
{
	buildTimer.start();

	try {
		CityGeographySpace* geography = new CityGeographySpace(DEFAULT_ENVIRONMENT_GEOGRAPHY, context);

		buildAreaGeography(geography);
		buildZoneGeography(geography);
		buildBuildingGeography(geography);
		buildParkingLotGeography(geography);
		buildBusNetwork(geography);
		buildRouteNetwork(geography);

		buildingMap = new CityBuildingMap();
	} catch (CityException& e) {
		throw e;
	}

	buildTimer.end();
}

void City::buildAreaGeography(CityGeographySpace* geography)
{
	if (!containProperty("shapefile.areas"))
		throw CityException(__func__, "not found 'shapefile.areas' in property file");

	if (!containProperty("shapefile.areas.id.field"))
		throw CityException(__func__, "shapefile.areas.id.field not specified");

	if (!containProperty("shapefile.areas.name.field"))
		throw CityException(__func__, "shapefile.areas.name.field not specified");

	CityShapefileReader shapefileReader;

	if (!shapefileReader.open(getStringProperty("shapefile.areas")))
		throw CityException(__func__, "shapefile.areas open fail");

	int shapeCount = shapefileReader.getShapeCount();
	int idField = getIntProperty("shapefile.areas.id.field");
	int nameField = getIntProperty("shapefile.areas.name.field");

	int id;
	std::string name;
	CityGeometry* cityGeom;

	for (int i = 0; i < shapeCount; i++) {
		cityGeom = shapefileReader.readShape(i);
		if (cityGeom == nullptr)
			throw CityException(__func__,"shapefile read shape fail");

		if (shapefileReader.readShapeAttr(i, idField, id) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, nameField, name) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		CityArea* area = new CityArea(id, name);

		areaMap[id] = area;

		geography->addAgent(area);
		geography->moveAgent(area, cityGeom);
	}
	shapefileReader.close();
}

void City::buildZoneGeography(CityGeographySpace* geography)
{
	if (!containProperty("shapefile.zones"))
		throw CityException(__func__, "not found 'shapefile.zones' in property file");

	if (!containProperty("shapefile.zones.id.field"))
		throw CityException(__func__, "shapefile.zones.id.field not specified");

	if (!containProperty("shapefile.zones.area.field"))
		throw CityException(__func__, "shapefile.zones.area.field not specified");

	if (!containProperty("shapefile.zones.name.field"))
		throw CityException(__func__, "shapefile.zones.name.field not specified");

	CityShapefileReader shapefileReader;

	if (!shapefileReader.open(getStringProperty("shapefile.zones")))
		throw CityException(__func__, "shapefile.zones open fail");

	int shapeCount = shapefileReader.getShapeCount();
	int idField = getIntProperty("shapefile.zones.id.field");
	int areaField = getIntProperty("shapefile.zones.area.field");
	int nameField = getIntProperty("shapefile.zones.name.field");

	int id;
	int areaCode;
	std::string name;
	CityGeometry* cityGeom;

	for (int i = 0; i < shapeCount; i++) {
		cityGeom = shapefileReader.readShape(i);
		if (cityGeom == nullptr)
			throw CityException(__func__,"shapefile read shape fail");

		if (shapefileReader.readShapeAttr(i, idField, id) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, areaField, areaCode) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, nameField, name) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		CityZone* zone = new CityZone(id, areaCode, name);

		zoneMap[id] = zone;

		/* register in the area to which this zone belongs */
		std::map<int, CityArea*>::iterator iter;
		iter = areaMap.find(areaCode);
		if(iter != areaMap.end()) {
			CityArea *area = iter->second;
			area->addZone(id, zone);
		}

		geography->addAgent(zone);
		geography->moveAgent(zone, cityGeom);
	}
	shapefileReader.close();
}

void City::buildBuildingGeography(CityGeographySpace* geography)
{
	if (!containProperty("buildings.file.path"))
		throw CityException(__func__, "not found 'buildings.file.path' in property file");

	if (!containProperty("buildings.id.field"))
		throw CityException(__func__, "buildings.id.field not specified");

	if (!containProperty("buildings.type.field"))
		throw CityException(__func__,"buildings.type.field not specified");

	CityShapefileReader shapefileReader;

	if (!shapefileReader.open(getStringProperty("buildings.file.path")))
		throw CityException(__func__, "buildings shapefile open fail");

	int shapeCount = shapefileReader.getShapeCount();
	int idField = getIntProperty("buildings.id.field");
	int typeField = getIntProperty("buildings.type.field");
	int nameField = getIntProperty("buildings.name.field");

	double id;
	double type;
	std::string name;
	CityGeometry* cityGeom;

	for (int i = 0; i < shapeCount; i++) {
		cityGeom = shapefileReader.readShape(i);
		if (cityGeom == nullptr)
			throw CityException(__func__,"shapefile read shape fail");

		if (shapefileReader.readShapeAttr(i, idField, id) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, typeField, type) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, nameField, name) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		CityBuilding* building = new CityBuilding(id, type, name);

		geography->addAgent(building);
		geography->moveAgent(building, cityGeom);

		building->setAreaCode(getAreaCode(building->getGeometry()));
		building->setZoneCode(getZoneCodes(building->getGeometry()));
	}

	shapefileReader.close();
}

void City::buildParkingLotGeography(CityGeographySpace* geography)
{
	if (!containProperty("parkinglots.file.path"))
		throw CityException(__func__, "parkinglots.file.path not specified");

	if (!containProperty("parkinglots.skip.first.row"))
		throw CityException(__func__, "parkinglots.skip.first.row not specified");

	if (!containProperty("parkinglots.id.field"))
		throw CityException(__func__, "parkinglots.id.field not specified");

	if (!containProperty("parkinglots.name.field"))
		throw CityException(__func__, "parkinglots.name.field not specified");

	if (!containProperty("parkinglots.capacity.field"))
		throw CityException(__func__, "parkinglots.capacity.field not specified");

	if (!containProperty("parkinglots.cost.field"))
		throw CityException(__func__, "parkinglots.cost.field not specified");

	if (!containProperty("parkinglots.longitude.field"))
		throw CityException(__func__, "parkinglots.longitude.field not specified");

	if (!containProperty("parkinglots.latitude.field"))
		throw CityException(__func__, "parkinglots.latitude.field not specified");

	int skip = getIntProperty("parkinglots.skip.first.row");
	int idField = getIntProperty("parkinglots.id.field");
	int nameField = getIntProperty("parkinglots.name.field");
	int capacityField = getIntProperty("parkinglots.capacity.field");
	int costField = getIntProperty("parkinglots.cost.field");
	int longitudeField = getIntProperty("parkinglots.longitude.field");
	int latitudeField = getIntProperty("parkinglots.latitude.field");

	CityCSVFileReader reader;

	if (!reader.open(getStringProperty("parkinglots.file.path")))
		throw CityException(__func__, "parkinglots csv file open fail");

	std::vector<std::string> row;
	int id;
	std::string name;
	int capacity;
	double cost;
	CityGeometry* cityGeom;
	double lat;
	double lon;

	if (skip) {
		reader.read(row);
		row.clear();
	}

	while (reader.read(row)) {
		id = std::stoi(row[idField]);
		name = row[nameField];
		capacity = std::stoi(row[capacityField]);
		cost = std::stod(row[costField]);
		lat = std::stod(row[latitudeField]);
		lon = std::stod(row[longitudeField]);

		CityParkingLot* parkingLot = new CityParkingLot(id, name, capacity, cost);

		cityGeom = CityGeometryFactory::instance()->createPoint(lon, lat);
		geography->addAgent(parkingLot);
		geography->moveAgent(parkingLot, cityGeom);

		parkingLot->setAreaCode(getAreaCode(parkingLot->getGeometry()));

		row.clear();
		name.clear();
	}

	reader.close();
}

void City::buildBusNetwork(CityGeographySpace* geography)
{
	if (!containProperty("bus.stops.file.path"))
		throw CityException(__func__, "bus.stops.file.path not specified");

	if (!containProperty("bus.stops.id.field"))
		throw CityException(__func__, "bus.stops.id.field not specified");

	if (!containProperty("bus.stops.name.field"))
		throw CityException(__func__, "bus.stops.name.field not specified");

	if (!containProperty("bus.lines.file.path"))
		throw CityException(__func__, "bus.lines.file.path not specified");

	if (!containProperty("bus.lines.id.field"))
		throw CityException(__func__, "bus.lines.id.field not specified");

	if (!containProperty("bus.lines.name.field"))
		throw CityException(__func__, "bus.lines.name.field not specified");

	if (!containProperty("bus.lines.from.stop.field"))
		throw CityException(__func__, "bus.lines.from.stop.field not specified");

	if (!containProperty("bus.lines.to.stop.field"))
		throw CityException(__func__, "bus.lines.to.stop.field not specified");

	if (!containProperty("bus.lines.section.order.field"))
		throw CityException(__func__, "bus.lines.section.order.field not specified");

	CityBusNetwork* network = new CityBusNetwork(DEFAULT_BUS_NETWORK, context);

	std::string busStopFilePath = getStringProperty("bus.stops.file.path");
	int stopIdField = getIntProperty("bus.stops.id.field");
	int stopNameField = getIntProperty("bus.stops.name.field");

	CityShapefileReader shapefileReader;

	if (!shapefileReader.open(busStopFilePath))
		throw CityException(__func__, "bus stop shapefile open fail");

	int shapeCount = shapefileReader.getShapeCount();

	double stopId;
	std::string stopName;
	CityGeometry* cityGeom;
	std::unordered_map<double, CityBusStop*> busStops;
	std::unordered_map<double, CityBusStop*>::iterator busStopsIter;
	CityBusStop* busStop;

	for (int i = 0; i < shapeCount; i++) {
		cityGeom = shapefileReader.readShape(i);
		if (cityGeom == nullptr)
			throw CityException(__func__,"shapefile read shape fail");

		if (shapefileReader.readShapeAttr(i, stopIdField, stopId) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, stopNameField, stopName) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		busStop = new CityBusStop(stopId, stopName);

		geography->addAgent(busStop);
		geography->moveAgent(busStop, cityGeom);

		busStop->setAreaCode(getAreaCode(busStop->getGeometry()));

		busStops[stopId] = busStop;

		network->addBusStop(busStop);
	}
	shapefileReader.close();

	std::string busLineFilePath = getStringProperty("bus.lines.file.path");
	int lineIdField = getIntProperty("bus.lines.id.field");
	int lineNameField = getIntProperty("bus.lines.name.field");
	int fromStopIdField = getIntProperty("bus.lines.from.stop.field");
	int toStopIdField = getIntProperty("bus.lines.to.stop.field");
	int sectionOrderField = getIntProperty("bus.lines.section.order.field");

	if (!shapefileReader.open(busLineFilePath))
		throw CityException(__func__, "bus line shape file open fail");

	shapeCount = shapefileReader.getShapeCount();

	double lineId;
	std::string lineName;
	double fromStopId;
	double toStopId;
	int sectionOrder;
	std::unordered_map<double, CityBusLine*> busLines;
	std::unordered_map<double, CityBusLine*>::iterator busLinesIter;
	CityBusLine* busLine;
	CityBusStop* fromBusStop;
	CityBusStop* toBusStop;

	int lineCount = 0;
	for (int i = 0; i < shapeCount; i++) {
		cityGeom = shapefileReader.readShape(i);
		if (cityGeom == nullptr)
			throw CityException(__func__,"shapefile read shape fail");

		if (shapefileReader.readShapeAttr(i, lineIdField, lineId) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, lineNameField, lineName) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, fromStopIdField, fromStopId) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, toStopIdField, toStopId) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, sectionOrderField, sectionOrder) == false)
			throw CityException(__func__,"shapefile read shape attr fail");

		busLinesIter = busLines.find(lineId);
		if (busLinesIter != busLines.end()) {
			busLine = busLinesIter->second;
		} else {
			busLine = new CityBusLine(lineId, lineName);
			busLines[lineId] = busLine;

			network->addBusLine(busLine);
			lineCount++;
		}

		busStopsIter = busStops.find(fromStopId);
		if (busStopsIter == busStops.end())
			throw CityException(__func__,"not found from bus stop id from bus line attr");

		fromBusStop = busStopsIter->second;
		fromBusStop->addBusLine(busLine, sectionOrder);
		busLine->addBusStop(fromBusStop, sectionOrder);

		busStopsIter = busStops.find(toStopId);
		if (busStopsIter == busStops.end())
			throw CityException(__func__,"not found to bus stop id from bus line attr");

		toBusStop = busStopsIter->second;
		toBusStop->addBusLine(busLine, sectionOrder+1);
		busLine->addBusStop(toBusStop, sectionOrder+1);

		busLine->addBusLineSection(fromStopId, toStopId, sectionOrder, cityGeom);
	}
	shapefileReader.close();

	std::cout << "BUSLINES = " << lineCount << std::endl;

	/* builds bus transfer lines */
	network->buildTransferLines();
}

void City::buildRouteNetwork(CityGeographySpace* geography)
{
	if (!containProperty("junctions.file.path"))
		throw CityException(__func__, "not found 'junctions.file.path' in property file");

	if (!containProperty("junctions.id.field"))
		throw CityException(__func__, "junctions.id.field not specified");

	if (!containProperty("roads.file.path"))
		throw CityException(__func__, "not found 'roads.file.path' in property file");

	if (!containProperty("roads.id.field"))
		throw CityException(__func__, "roads.id.field not specified");

	if (!containProperty("roads.start.junction.id.field"))
		throw CityException(__func__, "roads.start.junction.id.field not specified");

	if (!containProperty("roads.end.junction.id.field"))
		throw CityException(__func__, "roads.end.junction.id.field not specified");

	if (!containProperty("roads.lanes.field"))
		throw CityException(__func__, "roads.lanes.field not specified");

	if (!containProperty("roads.max.speed.field"))
		throw CityException(__func__, "roads.max.speed.field not specified");

	if (!containProperty("roads.length.field"))
		throw CityException(__func__, "roads.length.field not specified");

	if (!containProperty("roads.rank.field"))
		throw CityException(__func__, "roads.rank.field not specified");

	CityRouteNetwork* network = new CityRouteNetwork(DEFAULT_ROUTE_NETWORK, context);

	std::vector<CityRoad*> roads;
	std::unordered_map<double, CityJunction*> junctionMap;
	std::unordered_map<double, CityJunction*>::iterator iter;
	CityShapefileReader shapefileReader;
	int shapeCount;

	std::string junctionFile = getStringProperty("junctions.file.path");
	std::string roadFile = getStringProperty("roads.file.path");

	if (!shapefileReader.open(junctionFile))
		throw CityException(__func__, "junctions shapefile open fail");

	shapeCount = shapefileReader.getShapeCount();
	int junctionIdField = getIntProperty("junctions.id.field");

	double junctionId;
	CityGeometry* cityGeom;

	for (int i = 0; i < shapeCount; i++) {
		cityGeom = shapefileReader.readShape(i);
		if (cityGeom == nullptr)
			throw CityException(__func__,"junctions shapefile read shape fail");

		if (shapefileReader.readShapeAttr(i, junctionIdField, junctionId) == false)
			throw CityException(__func__,"junctions shapefile read shape attr fail");

		CityJunction* junction = new CityJunction(junctionId);

		if (junctionMap.find(junctionId) != junctionMap.end())
			throw CityException(__func__,"junction already exist");

		junctionMap[junctionId] = junction;

		geography->addAgent(junction);
		geography->moveAgent(junction, cityGeom);
	}
	shapefileReader.close();

	if (!shapefileReader.open(roadFile))
		throw CityException(__func__, "roads shapefile open fail");

	shapeCount = shapefileReader.getShapeCount();
	int roadIdField = getIntProperty("roads.id.field");
	int startJunctionIdField = getIntProperty("roads.start.junction.id.field");
	int endJunctionIdField = getIntProperty("roads.end.junction.id.field");
	int lanesField = getIntProperty("roads.lanes.field");
	int maxSpeedField = getIntProperty("roads.max.speed.field");
	int lengthField = getIntProperty("roads.length.field");
	int rankField = getIntProperty("roads.rank.field");

	double roadId;
	double startJunctionId;
	double endJunctionId;
	double lanes;
	double maxSpeed;
	double length;
	int rank;

	for (int i = 0; i < shapeCount; i++) {
		cityGeom = shapefileReader.readShape(i);
		if (cityGeom == nullptr)
			throw CityException(__func__,"roads shapefile read shape fail");

		if (shapefileReader.readShapeAttr(i, roadIdField, roadId) == false)
			throw CityException(__func__,"roads shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, startJunctionIdField, startJunctionId) == false)
			throw CityException(__func__,"roads shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, endJunctionIdField, endJunctionId) == false)
			throw CityException(__func__,"roads shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, lanesField, lanes) == false)
			throw CityException(__func__,"roads shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, maxSpeedField, maxSpeed) == false)
			throw CityException(__func__,"roads shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, lengthField, length) == false)
			throw CityException(__func__,"roads shapefile read shape attr fail");

		if (shapefileReader.readShapeAttr(i, rankField, rank) == false)
			throw CityException(__func__,"roads shapefile read shape attr fail");

		/* skips expressways */
		if (rank == 101)
			continue;

		iter = junctionMap.find(startJunctionId);
		if (iter == junctionMap.end())
			continue;

		CityJunction* startJunction = iter->second;

		iter = junctionMap.find(endJunctionId);
		if (iter == junctionMap.end())
			continue;

		CityJunction* endJunction = iter->second;

		CityRoad* road = new CityRoad(roadId, startJunctionId, endJunctionId, lanes, maxSpeed, length);

		road->makeDistances(cityGeom->getCoordinates().get());

		geography->addAgent(road);
		geography->moveAgent(road, cityGeom);

		roads.push_back(road);

		startJunction->addRoad(endJunctionId, road);
	}
	shapefileReader.close();

	/* removes the junctions with no roads like ICs */ 
	std::vector<CityJunction*> junctions;
	for (iter = junctionMap.begin(); iter != junctionMap.end(); iter++) { 
		CityJunction* junction = iter->second; 
		if (junction->getRoadSize() == 0) 
			geography->removeAgent(junction); 
		else
			junctions.push_back(junction);
	}

	network->build(junctions, roads);
}

CityGeographySpace* City::createGeographySpace(std::string name)
{
	try {
		if (name.empty())
			throw CityException(__func__, "name empty");

		if (name.compare(DEFAULT_ENVIRONMENT_GEOGRAPHY) == 0)
			throw CityException(__func__, "'Default_Environment_Geography' not allowed");

		CityGeographySpace* geography = new CityGeographySpace(name, context);
		return geography;
	} catch (CityException& e) {
		std::cout << e.what() << std::endl;
		return nullptr;
	}
}

CityGridSpace* City::createGridSpace(std::string name)
{
	try {
		if (name.empty())
			throw CityException(__func__, "name empty");

		CityGridSpace* grid = new CityGridSpace(name, context);
		return grid;
	} catch (CityException& e) {
		std::cout << e.what() << std::endl;
		return nullptr;
	}
}

CityNetworkSpace* City::createNetworkSpace(std::string name)
{
	try {
		if (name.empty())
			throw CityException(__func__, "name empty");

		CityNetworkSpace* network = new CityNetworkSpace(name, context);
		return network;
	} catch (CityException& e) {
		std::cout << e.what() << std::endl;
		return nullptr;
	}
}

CityRouteNetwork* City::createRouteNetwork(std::string name, CityGeographySpace* geography)
{
	try {
		if (name.empty())
			throw CityException(__func__, "name empty");

		CityRouteNetwork* network = new CityRouteNetwork(name, context);

		std::vector<CityJunction*> junctions;
		geography->getLayerAgents<CityJunction>(junctions);

		std::vector<CityRoad*> roads;
		geography->getLayerAgents<CityRoad>(roads);

		network->build(junctions, roads);

		return network;
	} catch (CityException& e) {
		std::cout << e.what() << std::endl;
		return nullptr;
	}
}

void City::exit()
{
	try {
		if (instance_ == 0)
			throw CityException(__func__, "City must be initialized before calling exit");
	} catch (CityException& e) {
		throw e;
		std::cout << e.what() << std::endl;
		return;
	}

	repast::RepastProcess::instance()->done();

	delete env;
	delete instance_;
}

boost::mpi::communicator* City::getCommunicator() const
{
	return comm;
}

CityContext* City::getContext() const
{
	return context;
}

CityScheduler* City::getScheduler() const
{
	return scheduler;
}

CityBuildingMap* City::getCityBuildingMap() const
{
	return buildingMap;
}

CityTimer City::getBuildTimer()
{
	return buildTimer;
}

CityGeographySpace* City::getGeographySpace() const
{
	return static_cast<CityGeographySpace*>(context->getSpace(DEFAULT_ENVIRONMENT_GEOGRAPHY));
}

CityGeographySpace* City::getGeographySpace(std::string name) const
{
	if (name.empty())
		return nullptr;

	return static_cast<CityGeographySpace*>(context->getSpace(name));
}

CityGridSpace* City::getGridSpace(std::string name) const
{
	if (name.empty())
		return nullptr;

	return static_cast<CityGridSpace*>(context->getSpace(name));
}

CityBusNetwork* City::getBusNetwork() const
{
	return static_cast<CityBusNetwork*>(context->getSpace(DEFAULT_BUS_NETWORK));
}

CityRouteNetwork* City::getRouteNetwork() const
{
	return static_cast<CityRouteNetwork*>(context->getSpace(DEFAULT_ROUTE_NETWORK));
}

CityRouteNetwork* City::getRouteNetwork(std::string name) const
{
	if (name.empty())
		return nullptr;

	return static_cast<CityRouteNetwork*>(context->getSpace(name));
}

CityNetworkSpace* City::getNetworkSpace(std::string name) const
{
	if (name.empty())
		return nullptr;

	return static_cast<CityNetworkSpace*>(context->getSpace(name));
}

int City::getIntProperty(const std::string& key) const
{
	return strToInt(props->getProperty(key));
}

double City::getDoubleProperty(const std::string& key) const
{
	return strToDouble(props->getProperty(key));
}

std::string City::getStringProperty(const std::string& key) const
{
	return props->getProperty(key);
}

void City::putProperty(const std::string& key, std::string value)
{
	props->putProperty(key, value);
}

void City::putProperty(const std::string& key, long double value)
{
	props->putProperty(key, value);
}

bool City::containProperty(const std::string& key) const
{
	return props->contains(key);
}

int City::getAreaCode(CityGeometry *geometry) const
{
	/* first search with contain() interface */
	for(auto entry : areaMap) {
		CityArea *area = entry.second;
		if (area->getGeometry()->contains(geometry) ||
			area->getGeometry()->intersects(geometry)) {
			return area->getAreaCode();
		}
	}

	/* if not found, return zero */                                                           
	return 0;
}

int City::getZoneCode(CityGeometry *geometry) const
{
	for(auto entry : zoneMap) {
		CityZone *zone = entry.second;
		if (zone->getGeometry()->contains(geometry) ||
			zone->getGeometry()->intersects(geometry)) {
			return zone->getZoneCode();
		}
	}

	/* if not found, return zero */
	return 0;
}

std::vector<int> City::getZoneCodes(CityGeometry *geometry) const
{
	std::vector<int> zones;

	for(auto entry : zoneMap) {
		CityZone *zone = entry.second;
		if (zone->getGeometry()->contains(geometry) ||
			zone->getGeometry()->intersects(geometry)) {
			zones.push_back(zone->getZoneCode());
		}
	}

	return zones;
}

} /* namespace dtsim */
