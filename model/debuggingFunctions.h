#ifndef DEBUGGINGFUNCTIONS_H
#define DEBUGGINGFUNCTIONS_H

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"
#include "debuggingFunctions.h"

namespace ns3 {
    void print_all_agent_costs(std::vector<Agent> &all_a);
    void print_cmatrix(std::vector<std::vector<double>> &c_mat);
    // void print_comm_g(std::vector<std::vector<bool>> &comm_graph);
    void print_component_membership(std::vector<Agent> &all_a);
    void print_membership_vec(Agent &ag);
    void print_request(Agent &ag);
    void print_ns3Vector(Vector &pos);
    void print_known_positions(AgentNode &ag);
    void print_single_request(send_request* req);
    // void print_received_requests(Agent &ag);
    void print_single_position_message(send_position* p_mess);
    // void print_received_position_messages(Agent &ag);
    void print_small_cost_mat(std::vector<std::vector<double>> &cmat);
    void print_agent_info(std::vector<Agent> &all_a);
    void print_global_assignment(std::vector<std::vector<int>> &g_assignment);
    void print_global_conflicts(std::vector<bool> &conflicts);
    void print_still_consider(std::vector<bool> consider, std::vector<int> task_ids);
    void print_loc_ranges(std::vector<loc_range> &ranges);
    void print_task_info(std::vector<Task> &all_ts);
    void print_instrument_assignment_vec();
}

#endif /* DEBUGGINGFUNCTIONS_H */