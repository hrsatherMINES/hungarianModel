#ifndef GLOBALINFO_H
#define GLOBALINFO_H

#include "structs.h"

namespace ns3 {
  class globalInfo{
    public:
    static std::vector<AgentNode> allAgents;
    static std::vector<TaskNode> allTasks;
    static std::vector<int> instrument_assignment;
    static int agents_per_class;
    static int numAgents;
    static int numTasks;
    static int probabilityDropped;
    static int numMoves;
    static int counter;
    static int totalMessagesReceived;

    private:
    globalInfo(){}
  };
}

#endif /* GLOBALINFO_H */