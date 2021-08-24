/*
 * Note: This license has also been called the "New BSD License" 
 * or "Modified BSD License".
 * 
 * Copyright (c) 2021 Electronics and Telecommunications Research 
 * Institute All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CITYGRID_H
#define CITYGRID_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <mutex>

#include "CityGeographySpace.h"
#include "CityBuilding.h"
#include "CityBusStop.h"
#include "CityJunction.h"
#include "CityParkingLot.h"

namespace dtsim {

class Cell {
public:
	int id;
	double x;
	double y;
	std::pair<int, void *> nearest;		/* <dist, agent>     */

public:
	Cell() {}
	virtual ~Cell() {}

	void getCoordinate(double &x_, double &y_) {
		x_ = x;
		y_ = y;
	}
};

template<typename T>
class CityGrid {
private:
	CityGeographySpace* Space;
	CityCoordinate Min;
	CityCoordinate Max;
	CityCoordinate Delta;
	int cellRows;
	int cellCols;
	int cellSize;						/* meter */
	int lookupRange;					/* meter */

private:
	std::map<int, Cell*> gridMap;
	typedef typename std::map<int, Cell*>::iterator gridMapIter;

	std::mutex lock;

public:
	CityGrid(CityGeographySpace *space);
	virtual ~CityGrid();

private:
	Cell* getCell(double x, double y);
	Cell* findCell(double x, double y);
	void getCentroid(double x, double y, double &cx, double &cy);
	void getCellId(double x, double y, double &cx, double &cy, int &centId);
	void getBoundedCells(double x, double y, std::vector<Cell*>& cells);
	void getWithinCells(double x, double y, std::vector<Cell*>& cells, int distance);

public:
	void buildGrid();
	T* getNearest(double x, double y); 
	T* getNearest(CityCoordinate *coord);
	T* getNearest(CityGeometry *geom);
	std::vector<std::pair<double, T*>> getNeighbors(double x, double y, double distance);
	std::vector<std::pair<double, T*>> getNeighbors(CityCoordinate *coord, double distance);
	std::vector<std::pair<double, T*>> getNeighbors(CityGeometry *geom, double distance);
	int getAreaCodeByPosition(double x, double y);
	int getZoneCodeByPosition(double x, double y);
};

template<typename T>
CityGrid<T>::CityGrid(CityGeographySpace *space)
{
	/* set grid boundaries */
	Min.x = 127.10;
	Max.x = 127.45;
	Min.y = 36.35;
	Max.y = 36.80;

	/* set cellsize & lookup range depending on AgentType */
	if(typeid(T) == typeid(CityJunction)) {
		cellSize = 10;
		lookupRange = cellSize * 2;
	} else if(typeid(T) == typeid(CityBuilding)) {
		cellSize = 10;
		lookupRange = cellSize * 2;
	} else {
		cellSize = 10;
		lookupRange = cellSize * 2;
	}

	/* caculate coordinate delta value of a cell */
	Delta.x = cellSize * 0.000011;
	Delta.y = cellSize * 0.000009;

	/* caculate the number of cells */
	cellRows = int((Max.y - Min.y) / Delta.y) + 1;
	cellCols = int((Max.x - Min.x) / Delta.x) + 1;

	/* set Geography space which includes T type agents */
	Space = space; 

	/* build grid space */
	buildGrid();
}

template<typename T>
CityGrid<T>::~CityGrid()
{
	for(auto C : gridMap) {
		Cell *cell = C.second;
		if(cell) {
			delete cell;
		}
	}
}

template<typename T>
void CityGrid<T>::getCentroid(double x, double y, double &cx, double &cy)
{
	/* calculate coordinate of cell which contains the given coordinate */
	cx = Min.x + ((int)(((x - Min.x) / Delta.x) + 0.1) * Delta.x);
    cy = Min.y + ((int)(((y - Min.y) / Delta.y) + 0.1) * Delta.y);
}

template<typename T>
void CityGrid<T>::getCellId(double x, double y, double &cx, double &cy, int &cid)
{
	cid = -1;

	getCentroid(x, y, cx, cy);

	/* if coordinates are out-of-range, return -1 */
	if((x < Min.x || x > Max.x) || (cx < Min.x || cx > Max.x)) return;
	if((y < Min.y || y > Max.y) || (cy < Min.y || cy > Max.y)) return;

	/* caculate the position of the X-axis cell(Longitude) */
	int posX = (int)(((cx - Min.x) / Delta.x) + 0.1);

	/* caculate the position of the Y-axis cell(Latitude) */
	int posY = (int)(((cy - Min.y) / Delta.y) + 0.1);

	/* caculate cell id */
	cid = (posY * cellCols) + posX ;
}

template<typename T>
Cell* CityGrid<T>::getCell(double x, double y)
{
	Cell *cell = nullptr;
	gridMapIter it;
	int cid = -1;
	double cx;
	double cy;

	/* get id of cell which contains the given coordinate */
	getCellId(x, y, cx, cy, cid);	
	if(cid == -1) {
		return nullptr;
	}

	/* find the cell from the gridMap */
	it = gridMap.find(cid);
	if(it != gridMap.end()) {
		cell = it->second;
	} else {
		/* if the first accessed cell, the new cell is created */
		cell = new Cell();
		cell->id = cid;
		cell->x = cx;
		cell->y = cy;
		cell->nearest.second = nullptr;

		gridMap[cell->id] = cell;
	}

	return cell;
}

template<typename T>
Cell* CityGrid<T>::findCell(double x, double y)
{
	Cell *cell = nullptr;
	gridMapIter it;
	int cid = -1;
	double cx;
	double cy;

	/* get id of cell which contains the given coordinate */
	getCellId(x, y, cx, cy, cid);	
	if(cid == -1) {
		return nullptr;
	}

	/* find the cell from the gridMap */
	it = gridMap.find(cid);
	if(it != gridMap.end()) {
		cell = it->second;
	}

	return cell;
}

template<typename T>
void CityGrid<T>::getWithinCells(double x, double y, std::vector<Cell*>& neighbors, int distance)
{
	Cell *center = nullptr;
	Cell *cell = nullptr;
	int range;
	double cx, cy;

	center = getCell(x, y);
	if(center == nullptr) {
		/* it is impossible */
		return;
	}

	if(distance < cellSize) {
		range = 1;
	} else {
		range = (distance / cellSize);
	}

	/* find all cells whthin the distance from the given coordinate */
	for(int i=-range; i<=range; i++) {
		cy = center->y + (i * Delta.y);
		for(int j=-range; j<=range; j++) {
			cx = center->x + (j * Delta.x);
			cell = getCell(cx, cy);
			if(cell == nullptr) {
				/* it is impossible */
				continue;
			}

			neighbors.push_back(cell);
		}
	}
}

template<typename T>
void CityGrid<T>::getBoundedCells(double x, double y, std::vector<Cell*>& neighbors)
{
	/* find cells adjacent to a given coordinates */
	getWithinCells(x, y, neighbors, cellSize);
}


template<typename T>
void CityGrid<T>::buildGrid()
{
	Cell *center;
	Cell *cell;
	std::vector<T*> agents;
	double cx, cy;
	double ax, ay;
	int dist;

	Space->getLayerAgents<T>(agents);

	for(auto A : agents) {
		std::vector<Cell*> neighbors;

		ax = A->getGeometry()->getCoordinate()->x;
		ay = A->getGeometry()->getCoordinate()->y;

		/* find cells belonging to a given lookupRange */
		getWithinCells(ax, ay, neighbors, lookupRange);

		/* update all neighbor cells */
		for(int i=0; i<neighbors.size(); i++) {
			cell = neighbors[i];

			/* caculate the distance between the agent and the cell */
			cell->getCoordinate(cx, cy);
			dist = (int)CityGeodeticCalculator::instance()->getDistance(ax, ay, cx, cy);

			/* update this cell's nearest agent */
			if(cell->nearest.second == nullptr) {
				/* register the agent */
				cell->nearest = std::make_pair(dist, A);
			} else {
				/* update to a closer agent */
				if(dist < cell->nearest.first) {
					cell->nearest.first = dist;
					cell->nearest.second = A;
				}
			}
		}
	}
}

template<typename T>
T* CityGrid<T>::getNearest(double x, double y)
{
	std::vector<Cell*> neighbors;
	Cell *cell = nullptr;
	double cx, cy;
	double ax, ay;
	int dist;

	std::lock_guard<std::mutex> guard(lock);

	cell = getCell(x, y);
	if(cell == nullptr) {
		return Space->getNearestAgent<T>(x, y);
	}

	if(cell->nearest.second == nullptr) {
		/* there is no nearest agent in the cell, */
		/* it must be searched using the quad tree.  */
		T* Agent= Space->getNearestAgent<T>(x, y);
		if(Agent == nullptr) {
			return nullptr;
		}

		ax = Agent->getGeometry()->getCoordinate()->x;
		ay = Agent->getGeometry()->getCoordinate()->y;

		/* caculate the distance between the agent and the cell */
		cell->getCoordinate(cx, cy);
		dist = (int)CityGeodeticCalculator::instance()->getDistance(ax, ay, cx, cy);

		/* register this agent to the nearest object of this cell */
		cell->nearest= std::make_pair(dist, Agent);
	}

	/* debuging codes */
#if 0
	static int pass = 1;
	static int fail = 1;
	static int error_distsum = 0;

	{
		T *Quad= Space->getNearestAgent<T>(x, y);
		T *Grid = static_cast<T*>(cell->nearest.second);
		double qx, qy;
		double gx, gy;
		int dist1, dist2;

		qx = Quad->getGeometry()->getCoordinate()->x;
		qy = Quad->getGeometry()->getCoordinate()->y;
		gx = Grid->getGeometry()->getCoordinate()->x;
		gy = Grid->getGeometry()->getCoordinate()->y;

		dist1 = (int)CityGeodeticCalculator::instance()->getDistance(qx, qy, x, y);
		dist2 = (int)CityGeodeticCalculator::instance()->getDistance(gx, gy, x, y);

		if(dist1 == dist2) {
			pass++;
		} else {
			fail++;

			error_distsum += abs(dist1 - dist2);
			int error_ratio = (fail * 100) / pass;
			int error_distavg = error_distsum / fail;

			printf("[%30s] error distance = %3d meters(avg=%3d), error ratio=%d PF(%d:%d)\n", 
				typeid(T).name(), abs(dist1 - dist2), error_distavg, error_ratio, pass, fail);
		}
	}
#endif

	return static_cast<T*>(cell->nearest.second);
}

template<typename T>
T* CityGrid<T>::getNearest(CityCoordinate *coord)
{
	double x = coord->x;
	double y = coord->y;

	return getNearest(x, y);
}

template<typename T>
T* CityGrid<T>::getNearest(CityGeometry *geom)
{
	double x, y;
	geom->getCoordinate(x, y);

	return getNearest(x, y);
}

template<typename T>
std::vector<std::pair<double, T*>> CityGrid<T>::getNeighbors(double x, double y, double distance)
{
	std::vector<std::pair<double, T*>> neighbors;

	Space->getAgentsWithin<T>(x, y, distance, neighbors);

	return neighbors;
}

template<typename T>
std::vector<std::pair<double, T*>> CityGrid<T>::getNeighbors(CityCoordinate *coord, double distance)
{
	double x = coord->x;
	double y = coord->y;

	return getNeighbors(x, y, distance);
}

template<typename T>
std::vector<std::pair<double, T*>> CityGrid<T>::getNeighbors(CityGeometry *geom, double distance)
{
	double x, y;
	geom->getCoordinate(x, y);

	return getNeighbors(x, y, distance);
}

template<typename T>
int CityGrid<T>::getAreaCodeByPosition(double x, double y)
{
	T *agent = getNearest(x, y);
	if(agent == nullptr) {
		return -1;
	}

	return agent->getAreaCode();
}

template<typename T>
int CityGrid<T>::getZoneCodeByPosition(double x, double y)
{
	T *agent = getNearest(x, y);
	if(agent == nullptr) {
		return -1;
	}

	return agent->getZoneCode();
}

} /* namespace dtsim */

#endif /* CITYGRID_H */
