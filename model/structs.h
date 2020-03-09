#ifndef STRUCTS_H
#define STRUCTS_H

#include "ns3/node.h"
#include "ns3/vector.h"

namespace ns3 {

    struct send_position;
    class TaskNode;
    class AgentNode;
    struct send_scores;
    struct send_request;
    struct conflict_info;
    class Agent;
    class Task;

    struct send_position{
        int sender_id;
        int info_id;
        Vector position;
        long long int time_sent;
    };

    class TaskNode{
    public:
        Ptr<Node> node;
        Task* task;
    };

    class AgentNode{
    public:
        Ptr<Node> node;
        Agent* agent;
    };

    struct send_scores{
        int sender_id;
        int info_id;
        double* scores;
        send_scores* next_message;
    };

    struct send_request{
        int sender_id;
        bool* request;
    };

    struct conflict_info{
        std::vector<int> agents_in_conflict;
        std::vector<int> tasks_in_conflict;
    };

    //Location information
    struct loc_range{
        int num_agents_in_range;
        int min_x;
        int min_y;
        int max_x;
        int max_y;
    };

    //Set up Task class
    class Task{
        public:

        int task_id;
        int instrument_requirement;
        Vector task_location;

        //methods
        void update_location(double x, double y, double z);
    };
}

#endif /* STRUCTS_H */