#ifndef NEWMETHOD_H
#define NEWMETHOD_H

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"
#include "ns3/debuggingFunctions.h"

namespace ns3 {
    void calculateAllCosts(std::vector<TaskNode> &allTasks, std::vector<AgentNode> &allAgents);
    void fillAllLocalCosts(std::vector<AgentNode> &allAgents);
    void initializeAllRequests(std::vector<AgentNode> &allAgents);
    void addOwnRequestToRequestList(std::vector<AgentNode> &allAgents);
    void allSendRequestsScheduled(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface);
    void allSendPositionInfoScheduled(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface);
    void computeAllParitalAssignmentsHungarian(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks);
    void determineAssignedLocation(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks);
    void moveAllAgentsTowardsGoalStep(std::vector<AgentNode> &allAgents);
    void mergeAllPositionInfo(std::vector<AgentNode> &allAgents, std::vector<TaskNode> allTasks, Ipv4InterfaceContainer interface);
    void calculateCostsAndPrepareRequests(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface);
    void processAndMove(std::vector<AgentNode> &allAgents, std::vector<TaskNode> allTasks, Ipv4InterfaceContainer interface);
    
    // New heuristics
    void determineAllNeededInfoOriginal(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoStillMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoSelfMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoBothMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoDistance(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoDistanceMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoInferTask(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoInferTaskAndMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoOneHop(std::vector<AgentNode> &allAgents);
}

#endif /* NEWMETHOD_H */