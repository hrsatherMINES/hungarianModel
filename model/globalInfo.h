#ifndef GLOBALINFO_H
#define GLOBALINFO_H

#include "structs.h"

namespace ns3 {
  class globalInfo{
    public:
    static std::vector<AgentNode> allAgents;
    static std::vector<TaskNode> allTasks;
    static std::vector<int> instrumentAssignment;
    static int agentsPerClass;
    static int numAgents;
    static int numTasks;
    static int probabilityDropped;
    static int numMoves;
    static int counter;
    static int totalMessagesReceived;
    static double maxPositionDistance;

    private:
    globalInfo(){}
  };
}

#endif /* GLOBALINFO_H */