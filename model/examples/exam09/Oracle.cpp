#include "city/DTSim.h"
#include "Oracle.h"

dtsim::CityGeographySpace* Oracle::mySpace;

Oracle::Oracle()
{
}

Oracle::~Oracle()
{
}

void Oracle::initModel()
{
	createSpace();
	createTeachers();
	createStudents();
	printf("Model Initialization is done...\n\n");
}

void Oracle::createSpace()
{
	string SpaceName = "testSpace";

	mySpace = dtsim::City::instance()->createGeographySpace(SpaceName);

	printf("Created GIS Space for the model....\n");
}

void Oracle::createTeachers()
{
	int count = dtsim::City::instance()->getIntProperty("teacher.count");

	for(int i=0; i<count; i++) {
		int age;	
		int salary;
		double x, y;

		/* generate teacher attributes */
		age = (rand() %  50) + 20;
		salary = (rand() %  200) + 100;
		DefaultPeople::generateLocation(x, y);

		/* create teacher agent */
		Teacher *t = new Teacher(x, y, age, salary);

		/* add teacher agent to model layer for easy-of-use */
		Teachers.push_back(t);
	}

	printf("Created Teachers....\n");
}

void Oracle::createStudents()
{
	int count = dtsim::City::instance()->getIntProperty("student.count");

	for(int i=0; i<count; i++) {
		int age;	
		int grade;
		double x, y;

		/* generate student attributes */
		age = (rand() %  50) + 20;
		grade = rand() %  6;
		DefaultPeople::generateLocation(x, y);

		/* create student agent */
		Student *s = new Student(x, y, age, grade);

		/* add student agent to model layer for easy-of-use */
		Students.push_back(s);
	}
	
	printf("Created Students....\n");
}

void Oracle::moveExampleByDisplacement()
{
	/* iterate for all students */
	printf("\n------------ move example by displacement ------------\n");
	for(auto S: Students) {
		double nx, ny;
		double ox, oy;
		double xShift, yShift;

		/* get current location for print */
		S->getGeometry()->getCoordinate(ox, oy);

		/* set displacement */
		xShift = yShift = 0.5;

		/* move by displacement */
		getSpace()->moveAgentByDisplacement(S, xShift, yShift);
		
		/* get moved location for print */
		S->getGeometry()->getCoordinate(nx, ny);

		printf("Student[%d] (%3.2f, %3.2f) moves + <shiftX=%.1f, shiftY=%.1f> --> (%3.2f, %3.2f)\n", 
			S->getAgentId(), ox, oy, xShift, yShift, nx, ny);
	}
}

void Oracle::moveExampleByVector()
{
	/* iterate for all students */
	printf("\n------------ move example by vector ------------\n");
	double x, y;
	double meters = 1000;
	double angle;

	Student *S = Students[0];

	angle = 0;
	getSpace()->moveAgent(S, 0, 0);
	getSpace()->moveAgentByVector(S, meters, angle);
	S->getGeometry()->getCoordinate(x, y);
	printf("Student (0, 0) moves + <%3.0f degree, 1000 meters> --> (%f, %f)\n", angle, x, y);

	angle = 45;
	getSpace()->moveAgent(S, 0, 0);
	getSpace()->moveAgentByVector(S, meters, angle);
	S->getGeometry()->getCoordinate(x, y);
	printf("Student (0, 0) moves + <%3.0f degree, 1000 meters> --> (%f, %f)\n", angle, x, y);

	angle = 90;
	getSpace()->moveAgent(S, 0, 0);
	getSpace()->moveAgentByVector(S, meters, angle);
	S->getGeometry()->getCoordinate(x, y);
	printf("Student (0, 0) moves + <%3.0f degree, 1000 meters> --> (%f, %f)\n", angle, x, y);

	angle = 180;
	getSpace()->moveAgent(S, 0, 0);
	getSpace()->moveAgentByVector(S, meters, angle);
	S->getGeometry()->getCoordinate(x, y);
	printf("Student (0, 0) moves + <%3.0f degree, 1000 meters> --> (%f, %f)\n", angle, x, y);

	angle = 270;
	getSpace()->moveAgent(S, 0, 0);
	getSpace()->moveAgentByVector(S, meters, angle);
	S->getGeometry()->getCoordinate(x, y);
	printf("Student (0, 0) moves + <%3.0f degree, 1000 meters> --> (%f, %f)\n", angle, x, y);

	angle = 360;
	getSpace()->moveAgent(S, 0, 0);
	getSpace()->moveAgentByVector(S, meters, angle);
	S->getGeometry()->getCoordinate(x, y);
	printf("Student (0, 0) moves + <%3.0f degree, 1000 meters> --> (%f, %f)\n", angle, x, y);
}
