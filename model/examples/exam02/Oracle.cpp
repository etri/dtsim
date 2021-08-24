#include "city/DTSim.h"
#include "Oracle.h"

Oracle::Oracle()
{
	envSpace = dtsim::City::instance()->getGeographySpace();
}

Oracle::~Oracle()
{
}

void Oracle::cityLookup()
{
	int key;

	do {
		printf("\n");
		printf("1. lookup buildings\n");
		printf("2. lookup busstops\n");
		printf("3. lookup parkinglots\n");
		printf("4. lookup junctions\n");
		printf("5. lookup areas\n");
		printf("6. exit\n");
		printf("input key -> ");

		cin >> key;

		switch(key) {
			case 1 : lookupBuildings(); break;
			case 2 : lookupBusStops(); break;
			case 3 : lookupParkingLots(); break;
			case 4 : lookupJunctions(); break;
			case 5 : lookupAreas(); break;
			case 6 : 
			default: return;
		}
	} while(1);
}

void Oracle::lookupBuildings()
{
	int count = 0;

	vector<dtsim::CityBuilding*> cityBuildings;
	envSpace->getLayerAgents<dtsim::CityBuilding>(cityBuildings);

	for (auto building : cityBuildings) {
		double id = building->getBuildingId();
		double type = building->getBuildingType();
		string coord = building->getCoordinates()->toString();
		printf("[%4d] id: %5.0f, type: %2.0f, Coord=%s\n", 
			++count, id, type, coord.c_str());
	}

	printf("Total Buildings : %d\n", count);
}

void Oracle::lookupBusStops()
{
	int count = 0;

	vector<dtsim::CityBusStop*> cityBusStops;
	envSpace->getLayerAgents<dtsim::CityBusStop>(cityBusStops);

	for (auto busstop : cityBusStops) {
		double id = busstop->getBusStopId();
		string name = busstop->getBusStopName();
		string coord = busstop->getCoordinate()->toString();
		printf("[%4d] id: %5.0f, name: %s, Coord=%s\n", 
			++count, id, name.c_str(), coord.c_str());
	}

	printf("Total BusStops : %d\n", count);
}

void Oracle::lookupParkingLots()
{
	int count = 0;

	vector<dtsim::CityParkingLot*> cityParkingLots;
	envSpace->getLayerAgents<dtsim::CityParkingLot>(cityParkingLots);

	for (auto parkingLot : cityParkingLots) {
		double id = parkingLot->getParkingLotId();
		int cost = parkingLot->getParkingLotCost();
		int capacity = parkingLot->getParkingLotCapacity();
		string name = parkingLot->getParkingLotName();
		string coord = parkingLot->getCoordinate()->toString();
		printf("[%4d] id: %5.0f, name: %s, capacity=%d, cost=%d, Coord=%s\n", 
			++count, id, name.c_str(), capacity, cost, coord.c_str());
	}

	printf("Total ParkingLots : %d\n", count);
}

void Oracle::lookupJunctions()
{
	int count = 0;

	vector<dtsim::CityJunction*> cityJunctions;
	envSpace->getLayerAgents<dtsim::CityJunction>(cityJunctions);

	for (auto junction : cityJunctions) {
		double id = junction->getJunctionId();
		string coord = junction->getCoordinate()->toString();
		printf("[%4d] id: %5.0f, Coord=%s\n", 
			++count, id, coord.c_str());
	}

	printf("Total Junctions : %d\n", count);
}

void Oracle::lookupAreas()
{
	int count = 0;

	vector<dtsim::CityArea*> cityAreas;
	envSpace->getLayerAgents<dtsim::CityArea>(cityAreas);

	for (auto area : cityAreas) {
		double id = area->getAreaCode();
		string name = area->getAreaName();
		string coord = area->getCoordinates()->toString();
		printf("[%4d] id: %5.0f, name: %s, Coord=%s\n", ++count, id, name.c_str(), coord.c_str());
	}

	printf("Total Areas : %d\n", count);
}

