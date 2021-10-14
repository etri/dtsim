#ifndef ROUTER_H
#define ROUTER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "city/DTSim.h"

using namespace std;
using namespace dtsim;

class Router {

public:
	dtsim::CityGeographySpace* envSpace;
	dtsim::CityBusNetwork* busnet;

	std::vector<BusRouteInfo> routes;

	std::vector<dtsim::CityBuilding*> Buildings;
	std::vector<dtsim::CityBusStop*> BusStops;

	dtsim::CityTimer runTimer;

public:
	Router();
	virtual ~Router();

	void getPathSize(std::vector<BusRouteInfo> routes);
	void getBusRoute();
	void run();
};

#endif /* ROUTER_H */
