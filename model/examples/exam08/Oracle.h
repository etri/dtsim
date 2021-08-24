#ifndef ORACLE_H
#define ORACLE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "city/DTSim.h"
#include "Teacher.h"
#include "Student.h"

class Oracle {
private:
	static dtsim::CityGeographySpace* mySpace;

	std::vector<Teacher*> Teachers;
	std::vector<Student*> Students;

private:
	void createSpace();
	void createTeachers();
	void createStudents();

public:
	Oracle();
	virtual ~Oracle();

	void initModel();

	void moveExampleByPoint();
	void moveExampleByCoordinate();

	static dtsim::CityGeographySpace* getSpace() { return mySpace; }
};

#endif /* ORACLE_H */
