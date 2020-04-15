#ifndef STRUCTS_H
#define STRUCTS_H

#include "ns3/node.h"
#include "ns3/vector.h"

namespace ns3 {

    struct sendPosition;
    class TaskNode;
    class AgentNode;
    struct sendScores;
    struct sendRequest;
    struct conflictInfo;
    class Agent;
    class Task;

    struct sendPosition {
        int senderId;
        int infoId;
        Vector position;
        unsigned long int timeSent;
    };

    class TaskNode {
    public:
        Ptr<Node> node;
        Task* task;
    };

    class AgentNode {
    public:
        Ptr<Node> node;
        Agent* agent;
    };

    struct sendRequest {
        int senderId;
        int numHops;
        bool* request;
    };

    // Set up Task class
    class Task {
        public:

        int taskId;
        int instrumentRequirement;
        Vector taskLocation;

        // Methods
        void updateLocation(double x, double y, double z) {
            taskLocation = Vector (x, y, z);
        }
    };

}  // End namespace ns3

#endif /* STRUCTS_H */