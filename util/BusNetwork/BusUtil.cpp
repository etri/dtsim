#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <iomanip>
#include "city/DTSim.h"

using namespace std;

static dtsim::CityGeographySpace* geography;
static dtsim::CityBusNetwork* busNetwork;
static unordered_map<double, dtsim::CityBusStop*> busStopMap;

static void printBusStopInfo(string busStopId)
{
	double id = dtsim::strToDouble(busStopId);

	unordered_map<double, dtsim::CityBusStop*>::iterator iter = busStopMap.find(id);
	if (iter == busStopMap.end()) {
		cout << "Not found Bus Stop ID : " << busStopId << endl;
	} else {
		cout << "BusStopID   : " << busStopId << endl;
		cout << "BusStopName : " << iter->second->getBusStopName() << endl;
	}
}

static void printBusLineInfo(string busLineId)
{
	double id = dtsim::strToDouble(busLineId);

	dtsim::CityBusLine* busLine = busNetwork->getBusLine(id);

	if (busLine == nullptr) {
		cout << "Not found Bus Line ID : " << busLineId << endl;
	} else {
		cout << "BusLineID   : " << busLineId << endl;
		cout << "BusLineName : " << busLine->getBusLineName() << endl;
	}
}

static void printBusStops(string busLineId)
{
	double id = dtsim::strToDouble(busLineId);

	dtsim::CityBusLine* busLine = busNetwork->getBusLine(id);

	if (busLine == nullptr) {
		cout << "Not found Bus Line ID : " << busLineId << endl;
	} else {
		cout << "BusLineID   : " << busLineId << endl;
		cout << "BusLineName : " << busLine->getBusLineName() << endl;

		vector<dtsim::CityBusStop*> busStops;
		busLine->getBusStops(busStops);
		cout << "Order   BusStopID   BusStopName" << endl;
		cout << "-------------------------------" << endl;
		for (int i = 0, n = busStops.size(); i < n; i++) {
			cout << setw(5) << i << "   ";
			cout << setprecision(9) << busStops[i]->getBusStopId() << "   ";
			cout << busStops[i]->getBusStopName() << endl;
		}
	}
}

static void printBusLines(string busStopId)
{
	double id = dtsim::strToDouble(busStopId);

	unordered_map<double, dtsim::CityBusStop*>::iterator iter = busStopMap.find(id);
	if (iter == busStopMap.end()) {
		cout << "Not found Bus Stop ID : " << busStopId << endl;
	} else {
		vector<pair<dtsim::CityBusLine*, int>> busLines;

		cout << "BusStopID   : " << busStopId << endl;
		cout << "BusStopName : " << iter->second->getBusStopName() << endl;

		iter->second->getBusLines(busLines);

		cout << "Number   BusLineID   Order" << endl;
		cout << "--------------------------" << endl;
		for (int i = 0, n = busLines.size(); i < n; i++) {
			pair<dtsim::CityBusLine*, int> aLine;

			cout << setw(6) << i << "   ";
			cout << setprecision(9) << busLines[i].first->getBusLineId() << "   ";
			cout << busLines[i].second << endl;
		}
	}
}

int main(int argc, char **argv)
{
	string propsFile;
	int ret = 0;

	if (argc < 2) {
		cout << "Usage: BusUtil  model_property_file" << endl;
		return 0;
	}

	propsFile = argv[1];

	try { 
		dtsim::City::init(propsFile);
		dtsim::City::instance()->buildEnvironment();
	} catch (dtsim::CityException& e) {
		cout << e.what() << endl;
		return -1;
	}

	geography = dtsim::City::instance()->getGeographySpace();

	vector<dtsim::CityBusStop*> busStops;
	geography->getLayerAgents<dtsim::CityBusStop>(busStops);

	vector<dtsim::CityBusStop*>::iterator iter = busStops.begin();
	while (iter != busStops.end()) {
		busStopMap[(*iter)->getBusStopId()] = *iter;
		iter++;
	}

	busNetwork = dtsim::City::instance()->getBusNetwork();

	string num;
	string busStopId;
	string busLineId;

	while (true) {
		cout << "1. Print bus stop info" << endl;
		cout << "2. Print bus line info" << endl;
		cout << "3. Print bus stops in a bus line" << endl;
		cout << "4. Print bus lines through a bus stop" << endl;
		cout << "99. Exit" << endl;
		cout << "==> select number ? ";

		getline(cin, num);

		if (num.compare("1") == 0) {
			cout << "bus stop id ? ";
			getline(cin, busStopId);
			cout << endl << endl;
			printBusStopInfo(busStopId);
		} else if (num.compare("2") == 0) {
			cout << "bus line id ? ";
			getline(cin, busLineId);
			cout << endl << endl;
			printBusLineInfo(busLineId);
		} else if (num.compare("3") == 0) {
			cout << "bus line id ? ";
			getline(cin, busLineId);
			cout << endl << endl;
			printBusStops(busLineId);
		} else if (num.compare("4") == 0) {
			cout << "bus stop id ? ";
			getline(cin, busStopId);
			cout << endl << endl;
			printBusLines(busStopId);
		} else if (num.compare("99") == 0) {
			break;
		}

		cout << endl << endl;
	}

	try {
		dtsim::City::exit();
	} catch(dtsim::CityException& e) {
		cout << e.what() << endl;
		ret = -1;
	}

	return ret;
}
