#include "city/DTSim.h"
#include "Oracle.h"
#include "Teacher.h"

Teacher::Teacher(double x_, double y_, int age_, int salary_)
{
	dtsim::CityGeometry* geom = dtsim::CityGeometryFactory::instance()->createPoint(x_, y_);

	age = age_;
	salary = salary_;

	/* 
	 * (IMPORTANT!!) When creating agent, the following two functions must be called
	 */

	/* add this agent to DTSim layer */
	Oracle::getSpace()->addAgent(this);

	/* move this agent to its original location */
	Oracle::getSpace()->deployAgent(this, geom);
}

Teacher::~Teacher()
{
}

