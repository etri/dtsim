#ifndef TEACHER_H
#define TEACHER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "city/DTSim.h"
#include "DefaultPeople.h"

class Teacher : public DefaultPeople {
private:
	int salary;

public:
	Teacher(double x_, double y_, int age_, int salary_);
	virtual ~Teacher();

	int getSalary() { return salary; };
};

#endif /* TEACHER_H */
