#ifndef AGENT_H
#define AGENT_H

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"

namespace ns3 {

    class Agent{
        public:

        //agent state
        int agent_id;
        Vector agent_position;
        Vector assigned_task_position;
        Vector movement_vec;
        int instrument_type;
        int component_membership;
        double distance_traveled;
        double distance_left;
        double* task_costs;
        bool* component_arr;
        double speed;

        //message totals
        int num_request_messages_sent;
        int num_score_info_messages_sent;
        int num_position_messages_sent;

        //received messages
        std::vector<send_request> received_requests;
        std::vector<send_scores> received_scores;
        std::vector<send_position> received_positions;
        double* received_times;

        bool* needed_info; //to keep track of what needed personally
        bool* info_requests; //to store request to
        bool* previous_sent_request;
        bool* known_info;
        bool* which_positions_to_send; //will act as a message buffer
        Vector* known_positions;
        bool** who_requested;
        int* partial_assignment;
        bool have_all_needed_info;
        conflict_info *c_info;

        // global info
        int numAgents;
        int numTasks;

        std::vector<std::vector<double>> localCostMatrix;

        //methods
        void print_position();
        void update_task_position(double x, double y, double z);
        void fill_in_agent_costs(std::vector<TaskNode> &all_ts);
        void print_agent_costs();
        void initialize_needed_info();
        void print_needed_info();
        void initialize_info_requests();
        void initialize_known_positions();
        void initialize_known_info();
        void print_known_info();
        void print_assigned_position();
        void initialize_partial_assignment();
        void initialize_cinfo();
        void initialize_received_times();
        void print_partial_assignment();
        void print_assigned_task_pos();
        void move_agent();
        send_request create_send_request();
        send_position create_send_position(int which_agent);
        void print_cinfo();
        void fillLocalCostMatrix();
        void set_speed(double speedIn);
        void set_num_agents(int numAgentsIn);
        void set_num_tasks(int numTasksIn);
    };
}

#endif /* AGENT_H */