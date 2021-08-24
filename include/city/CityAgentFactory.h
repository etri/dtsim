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
 *  CitAgentFactory.h
 *
 * $Revision: 923 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_AGENT_FACTORY_H
#define CITY_AGENT_FACTORY_H

#include <unordered_map>
#include <typeinfo>
#include <functional>
#include <mutex>

namespace dtsim {

using TypeInfoRef = std::reference_wrapper<const std::type_info>;

struct AgentTypeHasher {
	std::size_t operator()(TypeInfoRef code) const {
		return code.get().hash_code();
	}
};

struct AgentTypeEqualTo {
	bool operator()(TypeInfoRef one, TypeInfoRef two) const {
		return one.get() == two.get();
	}
};

struct AgentIdType {
	int agentType;

	AgentIdType() : agentType(0) {}
};

class CityAgentFactory {
private:
	typedef std::unordered_map<TypeInfoRef, AgentIdType*, AgentTypeHasher, AgentTypeEqualTo> AgentTypeMap;

	AgentTypeMap agentTypeMap;

	static int nextAgentType;
	static int nextAgentId;

	static CityAgentFactory* _instance;

	std::mutex _lock;

public:
	CityAgentFactory();
	virtual ~CityAgentFactory();

	static CityAgentFactory* instance();

	template<typename T>
	int getAgentId();

	template<typename T>
	int getAgentType();

	template<typename T>
	int findAgentType();
};

template<typename T>
int CityAgentFactory::getAgentId()
{
	std::lock_guard<std::mutex> guard(_lock);

	int nextId = -1;

	AgentTypeMap::iterator iter = agentTypeMap.find(typeid(T));
	if (iter != agentTypeMap.end()) {
		AgentIdType* agentIdType = iter->second;

		nextId = nextAgentId;
		nextAgentId++;
	}

	return nextId;
}

template<typename T>
int CityAgentFactory::getAgentType()
{
	std::lock_guard<std::mutex> guard(_lock);

	int type;

	AgentTypeMap::iterator iter = agentTypeMap.find(typeid(T));
	if (iter != agentTypeMap.end()) {
		type = iter->second->agentType;
	} else {
		type = nextAgentType++;

		AgentIdType* agentIdType = new AgentIdType();
		agentIdType->agentType = type;

		agentTypeMap[typeid(T)] = agentIdType;
	}

	return type;
}

template<typename T>
int CityAgentFactory::findAgentType()
{
	std::lock_guard<std::mutex> guard(_lock);

	AgentTypeMap::iterator iter = agentTypeMap.find(typeid(T));
	if (iter == agentTypeMap.end())
		return -1;
	else
		return iter->second->agentType;
}

} /* namespace dtsim */

#endif /* CITY_AGENT_FACTORY_H */
