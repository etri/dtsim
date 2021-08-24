/**
 *  Geography.h
 */

#ifndef GEOGRAPHY_H_
#define GEOGRAPHY_H_

#include <string>
#include <set>
#include <map>

#include "Projection.h"
#include "AgentId.h"

namespace repast {

/**
 * Geography implementation in RepastHPC
 */
template<typename T>
class Geography: public Projection<T> {
	typedef typename Projection<T>::RADIUS RADIUS;

public:
	Geography(std::string name);
	virtual ~Geography();

	virtual ProjectionInfoPacket* getProjectionInfo(AgentId id,
													bool secondaryInfo = false,
													std::set<AgentId>* secondaryIds = 0,
													int destProc = -1) { return nullptr; }
	virtual void updateProjectionInfo(ProjectionInfoPacket* pip, Context<T>* context) {}
	virtual void getRequiredAgents(std::set<AgentId>& agentsToTest,
								   std::set<AgentId>& agentsRequired,
								   RADIUS radius = Projection<T>::PRIMARY) {}
	virtual void getAgentsToPush(std::set<AgentId>& agentsToTest,
								 std::map<int, std::set<AgentId> >& agentsToPush) {}
	virtual bool keepsAgentsOnSyncProj() { return false; }
	virtual bool sendsSecondaryAgentsOnStatusExchange() { return false; }
	virtual void getInfoExchangePartners(std::set<int>& psToSendTo,
										 std::set<int>& psToReceiveFrom) {}
	virtual void getAgentStatusExchangePartners(std::set<int>& psToSendTo,
												std::set<int>& psToReceiveFrom) {}
	virtual void cleanProjectionInfo(std::set<AgentId>& agentsToKeep) {}
};

template<typename T>
Geography<T>::Geography(std::string name) : Projection<T>(name)
{}

template<typename T>
Geography<T>::~Geography()
{}

}

#endif /* GEOGRAPHY_H_ */
