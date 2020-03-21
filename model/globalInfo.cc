#include "globalInfo.h"

namespace ns3 {
    std::vector<AgentNode> globalInfo::allAgents;
    std::vector<TaskNode> globalInfo::allTasks;
    std::vector<int> globalInfo::instrument_assignment;
    int globalInfo::agents_per_class;
    int globalInfo::numAgents;
    int globalInfo::numTasks;
    int globalInfo::probabilityDropped;
    int globalInfo::numMoves;
    int globalInfo::counter;
    int globalInfo::totalMessagesReceived;
}