/*                                                                                                                  
 *                                                                                                                   * Note: This license has also been called the "New BSD License" 
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
 
/*
 *  CityRouteCache.cpp
 *
 *  Created by ksjin on 18/04/2019.
 *  Copyright © 2019 smlee. All rights reserved.
 *
 *  $Revision:$
 *  $LastChangedDate:$
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>

#include "City.h"
#include "CityRouteCache.h"
#include "CityException.h"

namespace dtsim {

/*
 * CityRouteCache Member Functions
 */
CityRouteCache::CityRouteCache(std::string cacheFile_, bool utility) : updated(false), cacheFile(cacheFile_)
{
	/* checks whether the specified cache file exists or not */
	if (utility == true) {
		/* if not, creates an empty file */
		std::fstream mfile(cacheFile, std::ios::in | std::ios::binary);
		if (mfile.fail()) {
			std::ofstream mfile;
			mfile.open(cacheFile);
			mfile.close();
		} else {
			mfile.close();
		}
	} else {
		load(); 
	}
}

CityRouteCache::~CityRouteCache() 
{
	flush();
	clear();
}

void CityRouteCache::clear() 
{
	RouteMapIterator sIter; 
	TargetMapIterator tIter; 
	TargetMap* targets;
	
	for (sIter = routeMap.begin(); sIter != routeMap.end(); sIter++) {
		targets = sIter->second; 
		for (tIter = targets->begin(); tIter != targets->end(); tIter++) { 
			RouteData* R = tIter->second; 
			delete R; 
		} 
		delete targets; 
	} 
	routeMap.clear(); 
}

void CityRouteCache::load() 
{
	std::fstream fileInStream(cacheFile, std::ios::in | std::ios::binary); 
	
	struct RouteInfo Record; 
	
	while (!fileInStream.eof()) { 
		unsigned int len = 0; 
		Record = {0}; 
		
		fileInStream.read((char *)&len, sizeof(int)); 
		fileInStream.read((char *)&Record, len); 
		
		if (len == 0)
			break; 
		
		double start = Record.start; 
		double goal = Record.goal; 
		
		RouteData* R = new RouteData(); 
		for (int i = 0; i < Record.pathSize-1; i++) 
			R->route.push_back(Record.paths[i]); 
		
		RouteMapIterator sIter = routeMap.find(start); 
		if (sIter == routeMap.end()) { 
			routeMap[start] = new TargetMap();
			sIter = routeMap.find(start); 
		} 
		
		TargetMap* targets = sIter->second; 
		if (targets->size() > 0) { 
			TargetMapIterator tIter = targets->find(goal); 
			if (tIter != targets->end()) { 
				std::cout << "Cache Load - Already existed target" << std::endl; 
				continue; 
			} 
		} 
		
		targets->emplace(goal, R); 
	} 
	
	fileInStream.close(); 
	std::cout << "RouteCache loading....done" << std::endl;
}

int CityRouteCache::getPath(const double start, const double goal, std::vector<double>& path) 
{
	bool reverse = false; 
	
	/* get a start index */ 
	RouteMapIterator sIter; 
	sIter = routeMap.find(start); 
	if (sIter == routeMap.end() || sIter->second->size() == 0)
		reverse = true; 
	
	/* get a target entry */ 
	TargetMapIterator tIter;
	
	if (reverse == true) { 
		sIter = routeMap.find(goal); 
		if (sIter == routeMap.end() || sIter->second->size() == 0)
			return 0; 
		
		tIter = sIter->second->find(start); 
	} else { 
		tIter = sIter->second->find(goal); 
	} 
	
	if (tIter == sIter->second->end())
		return 0; 
	
	RouteData* R = tIter->second; 
	path = R->route; 
	
//	printPath(start, goal, path);
    return path.size();
}

int CityRouteCache::putPath(const double start, const double goal, std::vector<double>& path) 
{
	RouteMapIterator sIter; 
	TargetMapIterator tIter;
	TargetMap* targets; 

	std::lock_guard<std::mutex> guard(mapLock);

	sIter = routeMap.find(start); 
	if (sIter != routeMap.end()) { 
		if (sIter->second->size() > 0) { 
			targets = sIter->second; 
			tIter = targets->find(goal); 
			if (tIter != targets->end()) { 
				RouteData* R = tIter->second; 
				R->route.clear(); 
				R->route = path; 
				updated = true;
				return 0; 
			} 
		} 
	} else { 
		routeMap[start] = new TargetMap();
		sIter = routeMap.find(start); 
	} 
	
	RouteData* R = new RouteData(); 
	R->route = path; 
	
	targets = sIter->second; 
	targets->emplace(goal, R); 
	updated = true; 
	return 0;
}

void CityRouteCache::flush() 
{
	if (updated == false)
		return; 
	
	std::ofstream ofs(cacheFile, std::ofstream::out | std::ofstream::trunc); 
	if (ofs.is_open() == 0)
		return; 
	ofs.close(); 
	
	std::ofstream fs(cacheFile, std::ios::out | std::ios::binary | std::ios::app);
	fs.seekp(0, fs.beg); 
	
	RouteMapIterator sIter; 
	TargetMapIterator tIter;
	TargetMap* targets;
	
	for (sIter = routeMap.begin(); sIter != routeMap.end(); sIter++) { 
		double start = sIter->first; 
		if (sIter->second->size() == 0) 
			continue; 
			
		targets = sIter->second; 
		for (tIter = targets->begin(); tIter != targets->end(); tIter++) { 
			RouteData *R = tIter->second; 
			struct RouteInfo RI = {0}; 
			double goal = tIter->first; 
			RI.start = start; 
			RI.goal = goal; 
			RI.pathSize = R->route.size() + 1; 
			
			int len = sizeof(double) * 2 + sizeof(int); 
			len += sizeof(double) * RI.pathSize; 
			
			int pos = 0; 
			for (double nodeid : R->route)
				RI.paths[pos++] = nodeid;
			
			fs.write((char *)&len, sizeof(int)); 
			fs.write((char *)&RI, len); 
		} 
	} 
	
	fs.close(); 
	std::cout << "Cache: FLUSH done" << std::endl;

}

void CityRouteCache::printPath(const double start, const double goal, std::vector<double>& path) 
{
	std::cout.precision(std::numeric_limits<double>::max_digits10);
	std::cout << std::endl; 
	std::cout << "START=" << start << ", GOAL=" << goal << ", size=" << path.size() << std::endl;

	for (double nodeid : path) {
		if (nodeid == start)
			std::cout << "" << nodeid << "-";
		else if (nodeid == goal)
			std::cout << "" << nodeid;
		else
			std::cout << "" << nodeid << "-";
	}
	std::cout << std::endl;
}

} /* namespace dtsim */
