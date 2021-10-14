#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <vector>
#include <limits>
#include <iostream>

#include "city/DTSim.h"
#include "Router.h"

using namespace std;
using namespace dtsim;

#define LOOPCOUNT	1
#define MAXDISTANCE	500.0
#define NUMWORKERS	48

std::mutex g_lock;
int g_totals = 0;
int g_noPaths = 0;
int g_totalRoutes = 0;
double g_totalTime = 0.0;

Router::Router()
{
	envSpace = dtsim::City::instance()->getGeographySpace();
	envSpace->getLayerAgents<dtsim::CityBuilding>(Buildings);
	envSpace->getLayerAgents<dtsim::CityBusStop>(BusStops);

	busnet = dtsim::City::instance()->getBusNetwork();
//	busnet->printBusStops();
//	busnet->printBusLines();
}

Router::~Router()
{
}

void Router::getPathSize(std::vector<BusRouteInfo> routes)
{
	int totalStops = 0;
	int routeIdx = 0;

	for (int i = 0; i < routes.size(); i++) {
		BusRouteInfo S = routes[i];

		std::cout << "[ Route-" << i << " ]" << std::endl;

		int stopsPerLine = 0;
		int lineIdx = 0;
		for (int j = 0; j < S.busRoute.size(); j++) {
			double lineId = S.busRoute[j].first;
			stopsPerLine += S.busRoute[j].second.size();
			std::cout << "Line-" << j << "[" << lineId << "] stops=" << stopsPerLine << std::endl;
		}
		totalStops += stopsPerLine;
	}

	std::cout << "# Total_Stops=" << totalStops << std::endl;
}

void Router::run()
{
	double startX, startY, goalX, goalY;
	dtsim::CityTimer rTimer;
	std::string keyin;
	bool bDirect = false;
	int maxRoutes = 5;
	double maxRadius = 500;

	dtsim::CityBuilding* startBuilding = nullptr;
	dtsim::CityBuilding* goalBuilding = nullptr;

	do {
		/* step1 : randomly choose start and goal buildings, respectively */
		startBuilding = Buildings[rand() % Buildings.size()];
		if (startBuilding == nullptr) {
			std::cout << "Not found start building" << std::endl;
			continue;
		}

		goalBuilding = Buildings[rand() % Buildings.size()];
		if (goalBuilding == nullptr) {
			std::cout << "Not found goal building" << std::endl;
			continue;
		}

		startBuilding->getGeometry()->getCentroid(startX, startY);
		goalBuilding->getGeometry()->getCentroid(goalX, goalY);

		/* step2 : skip if the distance between two buildings is too close */
		double distance = dtsim::CityGeodeticCalculator::instance()->getDistance(startX, startY, goalX, goalY);
		if (distance < 200) {
			std::cout << "Too close between start and goal buildingds: distance=" << (unsigned int)distance << "m, please go to walk" << std::endl;
			continue;
		}

		std::cout << std::endl;
		std::cout << "# Input max radius (meter)   = ";
		std::cin >> keyin;
		maxRadius = stol(keyin);

		std::cout << "# Input max routes           = ";
		std::cin >> keyin;
		maxRoutes = stoi(keyin);

		std::cout << "# Input direct flag (0 or 1) = ";
		std::cin >> keyin;
		bDirect = stoi(keyin);

		/* step3 : find bus stops within maxRadius from start and goal buildings */
		std::vector<CityBusStop*> startStops;
		envSpace->getAgentsWithin(startX, startY, maxRadius, startStops); 
		if (startStops.size() == 0) { 
			std::cout << "----- Not found start stops!" << std::endl;
			continue;
		}

		std::vector<CityBusStop*> goalStops;
		envSpace->getAgentsWithin(goalX, goalY, maxRadius, goalStops); 
		if (goalStops.size() == 0) { 
			std::cout << "----- Not found goal stops!" << std::endl;
			continue;
		} 

		std::cout << "# Start Building Coordinate [ " << (unsigned int)startX << "," << (unsigned int)startY << "] " << std::endl;
		std::cout << "# Goal Building Coordinate [" << (unsigned int)goalX << "," << (unsigned int)goalY << "] " << std::endl;
		std::cout << "# maxRadius=" << (unsigned int)maxRadius << ", maxRoutes=" << maxRoutes << ", direct=" << bDirect << std::endl;
		std::cout << "# Distance (start->goal)=" << (unsigned int)distance << ", Route_length_limit=" << (unsigned int)(distance*4) << std::endl;

		std::cout << "# Start stops within " << (unsigned int)maxRadius << "m = " << startStops.size() << std::endl; 
		std::cout << " "; 
		for (int i = 0; i < startStops.size(); i++) { 
			CityBusStop* p_start = startStops[i]; 
			std::cout << " " << (unsigned int)(p_start->getBusStopId()); 
		} 
		std::cout << std::endl;

		std::cout << "# Goal stops within " << (unsigned int)maxRadius << "m = " << goalStops.size() << std::endl; 
		std::cout << " "; 
		for (int i = 0; i < goalStops.size(); i++) { 
			CityBusStop* p_goal = goalStops[i]; 
			std::cout << " " << (unsigned int)(p_goal->getBusStopId()); 
		} 
		std::cout << std::endl;

		/* step 4. get bus routes */
		rTimer.start();
		busnet->getBusRoute(startX, startY, goalX, goalY, maxRadius, maxRoutes, routes, bDirect);
		rTimer.end();

		if (routes.size() == 0)
			printf("There is no bus path\n");
		else {
			int directs = 0;
			for (int i = 0; i < routes.size(); i++) {
				BusRouteInfo R = routes[i];
				if (R.busRoute.size() == 1)
					directs++;
			}

			std::cout << std::endl << ">>> Found PATH: " << "Direct=" << directs << ", Xfer=" << routes.size()-directs;
			std::cout << " Time=" << std::setprecision(4) << rTimer.getElapsedMSec() << " msec" << std::endl;

			std::cout.precision(std::numeric_limits<double>::max_digits10);
			busnet->printBusRoutes(routes);
		}

		std::cout << std::endl << "Continue (y or n) ? ";
		std::cin >> keyin;
	} while (keyin.compare("y") == 0);
}

void Router::getBusRoute()
{
	run();
}
