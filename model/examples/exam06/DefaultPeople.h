#ifndef DEFAULT_PEOPLE_H
#define DEFAULT_PEOPLE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "city/DTSim.h"

class DefaultPeople : public dtsim::CityAgent {
private:
	int age;
	int type;

public:
	typedef enum { TEACHER, STUDENT } AGENTTYPE;

public:
	DefaultPeople(double x_, double y_, int age_);
	virtual ~DefaultPeople();

	int getAge() { return age; };
	int getType() { return type; };
    void setType(int val) { type = val; };

	static void generateLocation(double &x, double &y);
};

#endif /* DEFAULT_PEOPLE_H */
