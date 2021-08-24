#ifndef ORACLE_H
#define ORACLE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "city/DTSim.h"

class Oracle {
private:
	dtsim::CityGeographySpace* envSpace;

public:
	Oracle();
	virtual ~Oracle();

	void cityLookup();
	void lookupBuildings();
	void lookupBusStops();
	void lookupParkingLots();
	void lookupJunctions();
	void lookupAreas();
};

#endif /* ORACLE_H */
