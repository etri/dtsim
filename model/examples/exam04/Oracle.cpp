#include "city/DTSim.h"
#include "Oracle.h"

Oracle::Oracle()
{
}

Oracle::~Oracle()
{
}

void Oracle::initModel()
{
	string SpaceName = "testSpace";

	mySpace = dtsim::City::instance()->createGeographySpace(SpaceName);

	printf("Created GIS Space for the model....\n");
	printf("Model Initialization is done...\n\n");
}

void Oracle::showSpace()
{
	printf("GIS Space Information\n");
	printf(" - space name = %s\n", mySpace->getName().c_str());
	printf(" - space size = %lu (number of agents)\n", mySpace->getSize());
}
