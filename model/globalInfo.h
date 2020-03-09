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

    private:
    globalInfo(){}
  };
}

#endif /* GLOBALINFO_H */