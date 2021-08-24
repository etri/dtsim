#ifndef STUDENT_H
#define STUDENT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "city/DTSim.h"
#include "DefaultPeople.h"

class Student : public DefaultPeople {
private:
	int grade;

public:
	Student(double x_, double y_, int age_, int grade_);
	virtual ~Student();

	int getGrade() { return grade; };
};

#endif /* STUDENT_H */
