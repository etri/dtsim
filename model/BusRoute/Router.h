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
#if 1
	void run();
#else
	void do_test1();
	void do_test2();
	void do_test3();
	void do_test4();
	void do_test5();
	void do_test6();
	void do_test7();
	void do_test8();

	void execute(const int tid);
	void forkWorkers();
	void do_parallel_test5();
#endif
};

#endif /* ROUTER_H */
