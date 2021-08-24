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

void Oracle::moveExampleByPoint()
{
	int tsize = Teachers.size();	

	/* iterate for all students */
	printf("\n------------ move example by point ------------\n");
	for(auto S: Students) {
		double x, y;
		double ox, oy;

		/* get current student location for print */
		S->getGeometry()->getCoordinate(ox, oy);

		/* randomly select teacher to move */
		Teacher *T = Teachers[rand() % tsize];

		/* move by point */
		T->getGeometry()->getCoordinate(x, y);
		getSpace()->moveAgent(S, x, y);
		
		printf("Student[%d] moves from (%3.2f, %3.2f) to Teacher[%d] (%3.2f, %3.2f)\n", 
			S->getAgentId(), ox, oy, T->getAgentId(), x, y);
	}
}

void Oracle::moveExampleByCoordinate()
{
	int tsize = Teachers.size();	

	/* iterate for all students */
	printf("\n------------ move example by coordinate class ------------\n");
	for(auto S: Students) {
	 	dtsim::CityCoordinate coord;
		double x, y;
		double ox, oy;

		/* get current student location for print */
		S->getGeometry()->getCoordinate(ox, oy);

		/* randomly select teacher to move */
		Teacher *T = Teachers[rand() % tsize];

		/* (method #2) : move by coordination */
		T->getGeometry()->getCoordinate(coord);
		getSpace()->moveAgent(S, coord);
		
		printf("Student[%d] moves from (%3.2f, %3.2f) to Teacher[%d] (%3.2f, %3.2f)\n", 
			S->getAgentId(), ox, oy, T->getAgentId(), coord.x, coord.y);
	}
}
