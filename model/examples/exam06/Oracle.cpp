#include <iostream>

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
		Peoples.push_back(t);
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
		Peoples.push_back(s);
	}

	printf("Created Students....\n");
}

void Oracle::showSpace()
{
	printf("GIS Space Information\n");
	printf(" - space name = %s\n", mySpace->getName().c_str());
	printf(" - space size = %lu (number of agents)\n", mySpace->getSize());
}

void Oracle::showTeachers()
{
	printf("Teacher list --> \n");
	for(auto T : Teachers) {
		int age = T->getAge();
		int salary = T->getSalary();
		double x, y;
		T->getGeometry()->getCoordinate(x, y);
		printf("\t[TEACHER] Age=%3d, Coord=%3.2f, %3.2f, Salary=%d\n", age, x, y, salary);
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
		printf("\t[STUDENT] Age=%3d, Coord=%3.2f, %3.2f, Grade=%d\n", age, x, y, grade);
	}

	printf("\n");
}

void Oracle::showPeoples()
{
	Student *S;
	Teacher *T;
	int age;
	int grade;
	int salary;
	double x, y;

	printf("Whole People list --> \n");
	for(auto P : Peoples) {
		age = P->getAge();
		P->getGeometry()->getCoordinate(x, y);

		switch(P->getType()) {
		case DefaultPeople::STUDENT :
			S = static_cast<Student*>(P);
			grade = S->getGrade();
			printf("\t[STUDENT] Age=%3d, Coord=%3.2f, %3.2f, Salary=%d\n", age, x, y, grade);
			break;
		case DefaultPeople::TEACHER :
			T = static_cast<Teacher*>(P);
			salary = T->getSalary();
			printf("\t[TEACHER] Age=%3d, Coord=%3.2f, %3.2f, Grade=%d\n", age, x, y, salary);
			break;
		}
	}

	printf("\n");
}
