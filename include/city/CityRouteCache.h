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

/**
 * CityRouteCache.h
 *
 * $Revision: $
 * $LastChangedDate: $
 */

#ifndef CITY_ROUTE_CACHE_H_
#define CITY_ROUTE_CACHE_H_

#include <unordered_map>
#include <vector>
#include <list>
#include <string>
#include <mutex>
#include <pthread.h>

namespace dtsim {

/**
 * Provides data writing to and reading from a single file 
 * It contains a shortest path from start node and goal one.
 */
struct RouteInfo {
    double start;
    double goal;
    int pathSize;
    double paths[1024];
};

/**
 * \class RouteData
 *
 * \brief
 * It is a data format to manager a shortest path in CityRouteCache.
 */
struct RouteData {
	std::vector<double> route;

	RouteData& operator=(std::vector<double>& in) {
		route.insert(route.begin(), in.begin(), in.end());
		return *this;
	}
};

/**
 * \class CityRouteCache
 *
 * \brief
 * CityRouteCache manages all shortest paths on the specified route network in memory
 * and handles shortest path requests immediately.
 * Those routing data are generated by a cacheGen utility and kept on storage.
 */
class CityRouteCache {

private:
	/// Route path map to target (target node id, shortest path)
	typedef typename std::unordered_map<double, RouteData*> TargetMap;
	typedef typename TargetMap::iterator TargetMapIterator;

	/// Route map from start (start node id, route path to target)
	typedef typename std::unordered_map<double, TargetMap*> RouteMap;
	typedef typename RouteMap::iterator RouteMapIterator;
	RouteMap routeMap;

	std::mutex mapLock;
	std::string cacheFile;
	bool updated;
	
public:
	/**
	 * Creates a CityRouteCache to cache shortest paths of the specified route network
	 * It is used by only Cache Utility 
	 *
	 * @param cacheFile the specific file path for data recording or retrieval 
	 */
	CityRouteCache(std::string cacheFile_, bool utility = false);

	~CityRouteCache();

	/// Configures a hash table by loading route data from the specific file
	void load();

	/// Flushes all updated route data in cache to the specific file
	void flush();

	/// Removes a hash table
	void clear();

	/// Retrunes a shortest path from start node to goal one from cache
	int getPath(const double start, const double goal, std::vector<double>& path);

	/// Puts new shortest path from start node to goal one into cache
	int putPath(const double start, const double goal, std::vector<double>& path);

	void printPath(double start, double goal, std::vector<double>& path);
};

} /* namespace dtsim */

#endif	/* CITY_ROUTE_CACHE_H_ */
