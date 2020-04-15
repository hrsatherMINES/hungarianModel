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
    int globalInfo::totalMessagesReceived;
    int globalInfo::testNumber;
    int globalInfo::minPosition;
    int globalInfo::maxPosition;
    double globalInfo::maxPositionDistance;

    // Variables to compute average
    int globalInfo::totalNumRequestMessages;
    int globalInfo::totalNumPositionMessages;
    double globalInfo::totalPercentageReceived;
    int globalInfo::totalNumberConflicts;
    double globalInfo::totalPercentOptimal;
}