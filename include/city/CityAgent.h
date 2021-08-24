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
 * CityAgent.h
 *
 * $Revision: 959 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_AGENT_H
#define CITY_AGENT_H

#include "repast_hpc/AgentId.h"
#include "CityGeometry.h"

namespace dtsim {

/**
 * \class CityAgent
 *
 * \brief
 * Base CityAgent implementation
 *
 * CityAgent inherits RepastHPC Agent class.
 * CityAgent contains RepastHPC AgentId and CityGeometry.
 * All agents added to CityContext must inherit CityAgent
 *
 * RepeastHPC Agent Id consists of four values:
 * 1) a numerical identifier assigned internally by CityAgentFactory
 * 2) the process on which the agent was created.
 * 3) a numerical identifier that indicates the agent's type assigned internally by CityAgentFactory
 * 4) the process on which the agent is a local agent.
 * Agent is displaced on geography space with CityGeometry object.
 */
class CityAgent: public repast::Agent {
private:
	/// RepastHPC AgentId
	repast::AgentId agentId;

	/// CityGeometry object
	CityGeometry* geometry;

	/// area code where this agent is located
	int areaCode;

	/// zone code where this agent is located
	int zoneCode;
	std::vector<int> zoneCodes;

public:
	CityAgent();
	virtual ~CityAgent();

	/// Non User API
	virtual repast::AgentId& getId() {
		return agentId;
	}

	/// Non User API
	virtual const repast::AgentId& getId() const {
		return agentId;
	}

	/// Non User API
	void setId(repast::AgentId& id) {
		agentId = id;
	}

	/// Returns a numerical identifier
	virtual int getAgentId() const {
		return agentId.id();
	}

	/// Set a area code
	virtual void setAreaCode(int code) {
		areaCode = code;
	}

	/// Returns a area code
	virtual int getAreaCode() {
		return areaCode;
	}

	/// Set a zone code
	virtual void setZoneCode(int code) {
		zoneCodes.push_back(code);
	}

	/// Set a zone codes
	virtual void setZoneCode(std::vector<int> zones) {                                        
		zoneCodes.insert(zoneCodes.end(), zones.begin(), zones.end());
	}

	/// Returns a zone code
	virtual int getZoneCode() {
		if(zoneCodes.size() == 0) {
			return -1;
		} else {
			return zoneCodes[0];
		}
	}

	/// Returns a zone code list
	virtual std::vector<int> getZoneCodes() {
		return zoneCodes;
	}

	/// Returns the start process id
	virtual int getAgentStartProc() const {
		return agentId.startingRank();
	}

	/// Returns a type
	virtual int getAgentType() const {
		return agentId.agentType();
	}

	/// Returns the current process id
	virtual int getAgentCurrentProc() const {
		return agentId.currentRank();
	}

	/// Returns the CityGeometry object
	virtual CityGeometry* getGeometry() const {
		return geometry;
	}

	virtual void setGeometry(CityGeometry* geom) {
		geometry = geom;
	}
};

} /* namespace dtsim */

#endif /* CITY_AGENT_H */
