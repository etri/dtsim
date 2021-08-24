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
	dtsim::CityGeographySpace* mySpace;

public:
	Oracle();
	virtual ~Oracle();

	void initModel();
	void showSpace();
};

#endif /* ORACLE_H */
