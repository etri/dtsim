#include "city/DTSim.h"
#include "DefaultPeople.h"

DefaultPeople::DefaultPeople(double x_, double y_, int age_)
{
	dtsim::CityGeometry* geom = dtsim::CityGeometryFactory::instance()->createPoint(x_, y_);

	age = age_;
}

DefaultPeople::~DefaultPeople()
{
}

void DefaultPeople::generateLocation(double &x, double &y)                                              
{
	x = 127.30 + ((double)(rand()% 100) / 100.0);
	y = 36.40 + ((double)(rand()% 100) / 100.0);
}
