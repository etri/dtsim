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
		generateLocation(x, y);

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
		generateLocation(x, y);

		/* create student agent */
		Student *s = new Student(x, y, age, grade);

		/* add student agent to model layer for easy-of-use */
		Students.push_back(s);
	}

	printf("Created Students....\n");
}

void Oracle::generateLocation(double &x, double &y)
{
	x = 127.30 + ((double)(rand()% 100) / 100.0);
	y = 36.40 + ((double)(rand()% 100) / 100.0);
}

void Oracle::showSpace()
{
	printf("GIS Space Information\n");
	printf(" - space name = %s\n", getSpace()->getName().c_str());
	printf(" - space size = %lu (number of agents)\n", getSpace()->getSize());
}

void Oracle::showTeachers()
{
	printf("Teacher list --> \n");
	for(auto T : Teachers) {
		int age = T->getAge();
		int salary = T->getSalary();
		double x, y;
		T->getGeometry()->getCoordinate(x, y);
		printf("\tAge=%3d, Salary=%d, Coord=%3.2f, %3.2f\n", age, salary, x, y);
	}

	printf("\n");
}

void Oracle::showStudents()
{
	printf("Student list --> \n");
	for(auto S : Students) {
		int age = S->getAge();
		int grade = S->getGrade();
		double x, y;
		S->getGeometry()->getCoordinate(x, y);
		printf("\tAge=%3d, Grade=%d, Coord=%3.2f, %3.2f\n", age, grade, x, y);
	}

	printf("\n");
}
