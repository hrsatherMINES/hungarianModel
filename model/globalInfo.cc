#include "globalInfo.h"

namespace ns3 {
    std::vector<AgentNode> globalInfo::allAgents;
    std::vector<TaskNode> globalInfo::allTasks;
    std::vector<int> globalInfo::instrumentAssignment;
    int globalInfo::agentsPerClass;
    int globalInfo::numAgents;
    int globalInfo::numTasks;
    int globalInfo::probabilityDropped;
    int globalInfo::numMoves;
    int globalInfo::counter;
    int globalInfo::totalMessagesReceived;
    double globalInfo::maxPositionDistance;
}