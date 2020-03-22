#ifndef DEBUGGINGFUNCTIONS_H
#define DEBUGGINGFUNCTIONS_H

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"
#include "debuggingFunctions.h"

namespace ns3 {
    void printAllAgentsCosts(std::vector<Agent> &allAgents);
    void printCostMatrix(std::vector<std::vector<double>> &costMatrix);
    void printRequest(Agent &ag);
    void printNS3Vector(Vector &pos);
    void printKnownPositions(AgentNode &ag);
    void printSingleRequest(sendRequest* req);
    void printSinglePositionMessage(sendPosition* positionMessage);
    void printSmallCostMatrix(std::vector<std::vector<double>> &costMatrix);
    void prinAgentInfo(std::vector<Agent> &allAgents);
    void printGlobalAssignment(std::vector<std::vector<int>> &agentAssignment);
    void printGlobalConflicts(std::vector<bool> &conflicts);
    void printStillConsider(std::vector<bool> consider, std::vector<int> taskIds);
    void printTaskInfo(std::vector<Task> &allTasks);
    void printInstrumentAssignmentVec();
}

#endif /* DEBUGGINGFUNCTIONS_H */