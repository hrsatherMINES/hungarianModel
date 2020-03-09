/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef DISCONNECTED_H
#define DISCONNECTED_H

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/log.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/packet.h"
#include "ns3/internet-apps-module.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <limits.h>
#include "hungarian.h"
#include <cmath>
#include <stdio.h>
#include <chrono>
#include "ns3/agent.h"
#include "ns3/structs.h"
#include "ns3/hungarian.h"
#include "ns3/globalInfo.h"
#include "ns3/messageHandling.h"


namespace ns3 {
    std::vector<loc_range> create_loc_ranges();
    bool conflicts_exist(std::vector<AgentNode> &all_a);
    double euc_dist(Vector t_pos, Vector a_pos);
    Vector add_ns3Vectors(Vector v1, Vector v2);
    int get_instrument_type(int id);
    std::vector<Task> create_tasks();
    Ptr<UniformRandomVariable> generate_rand_stream();
    std::vector<Agent*> create_agents();
    void calculate_all_costs(std::vector<TaskNode> allTasks, std::vector<AgentNode> all_a);
    std::vector<std::vector<double>> create_cmatrix(std::vector<AgentNode> &all_a);
    std::vector<std::vector<bool>> create_init_comm_graph(std::vector<AgentNode> &all_a);
    int all_agents_visited(std::vector<bool> &visit_list);
    void determine_connected_components(std::vector<AgentNode> &all_a, std::vector<std::vector<bool>> &commGraph);
    void fill_in_component_arrs(std::vector<AgentNode> &all_a);
    void initialize_all_needed_info(std::vector<AgentNode> &all_a);
    void initialize_all_requests(std::vector<AgentNode> &all_a);
    double vector_magnitude(Vector &pos);
    void send_position_info(AgentNode &sender, AgentNode &receiver, int which_agent, Ipv4InterfaceContainer interface);
    void delete_request_message(send_request* mess);
    void delete_who_requested(bool** arr);
    void initial_request_sharing(std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface);
    bool compare_bool_arr(bool* prev, bool* req);
    bool check_if_agent_has_needed_info(Agent *ag);
    bool check_all_needed_info(std::vector<AgentNode> &all_a);
    void determine_position_messages_to_send(AgentNode *ag);
    void send_position_messages_in_buffer(int curr_ag, std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface);
    double compute_partial_assignment_hungarian(Agent &ag, std::vector<TaskNode> &all_tasks);
    void compute_all_parital_assignments_hungarian(std::vector<AgentNode> &all_ags, std::vector<TaskNode> &all_ts);
    void overwrite_output_file();
    void write_initial_positions_to_file(std::vector<Agent> &all_ags, std::vector<Task> &all_ts);
    void write_position_to_file(Agent &ag);
    void write_file_end();
    double vector_magnitude(double x, double y, double z);
    Vector diff_Vector(Vector &ag_pos, Vector &task_pos);
    Vector create_movement_vector(Vector &ag_pos, Vector &task_pos);
    void determine_assigned_location(std::vector<AgentNode> &all_ags, std::vector<TaskNode> &all_ts);
    bool move_agent_towards_goal(AgentNode &ag);
    void move_all_agents_towards_goal(std::vector<AgentNode> &all_ags);
    std::vector<std::vector<int>> global_assignment_info(std::vector<Agent> &all_a);
    std::vector<bool> determine_global_conflicts(std::vector<std::vector<int>> &g_assignment);
    std::vector<bool> determine_agents_with_conflicts(std::vector<std::vector<int>> &g_assignment, std::vector<bool> &conflicts);
    void create_agent_conflict_info(std::vector<std::vector<int>> &g_assignment, std::vector<bool> &conflicts, Agent &a, std::vector<Task> &all_tasks);
    void all_agent_conflict_info(std::vector<std::vector<int>> &g_assignment, std::vector<bool> &conflicts, std::vector<Agent> &all_a, std::vector<Task> &all_tasks);
    bool all_agents_assigned(std::vector<AgentNode> &all_a);
    void all_send_position_info(std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface);
    void all_send_all_position_info(std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface);
    void haydens_method(std::vector<TaskNode> allTasks, std::vector<AgentNode> all_a);
    void sams_method(std::vector<Agent> &all_a, std::vector<Task> &all_tasks);
    void move_reassigned_agents_towards_goal(std::vector<Agent> &all_ags, std::vector<bool> &conflicts);
    double total_dist_traveled(std::vector<Agent> &all_ags);
    void determine_to_send_all_information(Agent *ag);
    void move_all_agents_towards_goal_step(std::vector<AgentNode> &all_ags);
    double fRand(double fMin, double fMax);
    Vector GetPosition (Ptr<Node> node);
    void SetPosition (Ptr<Node> node, Vector position);
    void movePositions(Ptr<Node> node);
    void moveAllPositions(NodeContainer robots);
    void update_position(AgentNode* ag);
    void move_agent_towards_goal_step(AgentNode ag);
    void fillAllLocalCosts(std::vector<AgentNode> &all_a);
}

#endif /* DISCONNECTED_EXP_H */
