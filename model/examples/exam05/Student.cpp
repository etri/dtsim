#include "city/DTSim.h"
#include "Oracle.h"
#include "Student.h"

Student::Student(double x_, double y_, int age_, int grade_)
{
	dtsim::CityGeometry* geom = dtsim::CityGeometryFactory::instance()->createPoint(x_, y_);

	age = age_;
	grade = grade_;

	/* 
	 * (IMPORTANT!!) When creating agent, the following two functions must be called
	 */

	/* add this agent to DTSim layer */
	Oracle::getSpace()->addAgent(this);

	/* move this agent to its original location */
	Oracle::getSpace()->deployAgent(this, geom);
}

Student::~Student()
{
}

