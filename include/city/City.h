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
 * City.h
 *
 * $Revision: 959 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_H
#define CITY_H

#include <boost/mpi/communicator.hpp>
#include <string>

#include "repast_hpc/Properties.h"
#include "CityScheduler.h"
#include "CityBuildingMap.h"
#include "CityBusNetwork.h"
#include "CityRouteNetwork.h"
#include "CityAgentFactory.h"
#include "CityArea.h"
#include "CityZone.h"
#include "CityTimer.h"

namespace dtsim {

class CityContext;
class CityGeographySpace;
class CityGridSpace;
class CityNetworkSpace;

/**
 * \class City
 *
 * \brief
 * Base implementation of City Model
 *
 * City contains context, scheduler, spaces and properties information.
 * City is initialized by City::init(argc, argv)
 * City is access by City::instance()
 * City is exited by City::exit()
 */
class City {
private:
	boost::mpi::communicator* comm;
	repast::Properties* props;

	CityContext* context;
	CityScheduler* scheduler;

	/// Area Map (area code, CityArea object pointer)
	typedef typename std::map<int, CityArea*> AreaMap;
	AreaMap areaMap;

	/// Zone Map (zone code, CityZone object pointer)
	typedef typename std::map<int, CityZone*> ZoneMap;
	ZoneMap zoneMap;

	CityBuildingMap* buildingMap;

	static boost::mpi::environment* env;
	static City* instance_;
	CityTimer buildTimer;

	void buildAreaGeography(CityGeographySpace* geography);
	void buildZoneGeography(CityGeographySpace* geography);
	void buildBuildingGeography(CityGeographySpace* geography);
	void buildParkingLotGeography(CityGeographySpace* geography);
	void buildBusNetwork(CityGeographySpace* geography);
	void buildRouteNetwork(CityGeographySpace* geography);

public:
	static std::string DEFAULT_MODEL_PROPERTY_FILE;
	static std::string DEFAULT_ENVIRONMENT_GEOGRAPHY;
	static std::string DEFAULT_AGENT_GEOGRAPHY;
	static std::string DEFAULT_BUS_NETWORK;
	static std::string DEFAULT_ROUTE_NETWORK;

	/**
	 * Creates a City by calling init function.
	 *
	 * @param propsFile model property file path
	 */
	City(std::string propsFile);
	virtual ~City();

	/// Get this City
	static City* instance();

	/**
	 * Initialize this City. This must be called before City is used
	 *
	 * @param modelPropsFile model property file path
	 * @throws CityException if City was already initialized or not found property files
	 */
	static void init(std::string modelPropsFile);

	/**
	 * Exit city simulator. It must be called before process return
	 *
	 * @throws CityException if City didn't initialized
	 */
	static void exit();

	/**
	 * Build city environment
	 *  - Create default city geography
	 *  - Add areas, buildings, bus stops, junctions, parkinglots and roads to default city geography
	 *  - Creates default city road network with junctions and roads
	 *
	 * @throws CityException if not found property files or invalid property
	 */
	void buildEnvironment();

	/// Get MPI Communicator
	boost::mpi::communicator* getCommunicator() const;

	/// Get CityContext
	CityContext* getContext() const;

	/// Get CityScheduler
	CityScheduler* getScheduler() const;

	/// return area code to with this coordinate belongs
	int getAreaCode(CityGeometry *geometry) const; 

	/// return zone code to with this coordinate belongs
	int getZoneCode(CityGeometry *geometry) const; 

	/// return zone codes to with this coordinate belongs
	std::vector<int> getZoneCodes(CityGeometry *geometry) const; 

	/// Get city building map
	CityBuildingMap* getCityBuildingMap() const;

	/// Return City Build timer
	CityTimer getBuildTimer();

	/// Get default city geography
	CityGeographySpace* getGeographySpace() const;

	/// Get non default city geography
	CityGeographySpace* getGeographySpace(std::string name) const;

	/// Get grid space
	CityGridSpace* getGridSpace(std::string name) const;

	/// Get network space
	CityNetworkSpace* getNetworkSpace(std::string name) const;

	/// Get default bus network
	CityBusNetwork* getBusNetwork() const;

	/// Get non default bus network
	CityBusNetwork* getBusNetwork(std::string name) const;

	/// Get default route network
	CityRouteNetwork* getRouteNetwork() const;

	/// Get non default route network
	CityRouteNetwork* getRouteNetwork(std::string name) const;

	/// Creates non default city geography
	CityGeographySpace* createGeographySpace(std::string name);

	/// Creates grid space
	CityGridSpace* createGridSpace(std::string name);

	/// Creates network space
	CityNetworkSpace* createNetworkSpace(std::string name);

	/// Creates non default route network from non default city geography
	/// Junctions and Roads must be added to specified geography
	CityRouteNetwork* createRouteNetwork(std::string name, CityGeographySpace* geography);

	/// Get value(integer) for key from property file
	int getIntProperty(const std::string& key) const;

	/// Get value(double) for key from property file
	double getDoubleProperty(const std::string& key) const;

	/// Get value(string) for key from property file
	std::string getStringProperty(const std::string& key) const;

	/// Put key-value to property file
	void putProperty(const std::string& key, std::string value);

	/// Put key-value to property file
	void putProperty(const std::string& key, long double value);

	/// Returns true if key-value is contained in property file
	bool containProperty(const std::string& key) const;
};

} /* namespace dtsim */

#endif /* CITY_H */
