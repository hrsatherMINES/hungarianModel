//NOTE: Not added to wscript
#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"
#include "debuggingFunctions.h"

namespace ns3 {
    int orientation(Vector p, Vector q, Vector r);
    std::vector<Vector> convexHull(std::vector<Agent> &all_ags, int connected_component);
    void overwrite_hull_file(string filename);
    void output_hull(std::vector<Vector> hull, int component_id, string filename);
    int isLeft(Vector P0, Vector P1, Vector P2);
    int cn_PnPoly( Vector P, std::vector<Vector> V, int n );
    int wn_PnPoly( Vector P, std::vector<Vector> V, int n );
    // std::vector<std::vector<bool>> create_component_communication_graph(std::vector<AgentNode> &all_a, int start, int end);
    std::vector<bool> check_connectivity(std::vector<std::vector<bool>> comp_g, int n);
    void create_connected_components(std::vector<AgentNode> &all_a, int num_components, std::vector<loc_range> &all_ranges);
    double compute_reassignment(Agent &ag);
    // double range_expansion(std::vector<Agent> &all_ag, int curr_id, std::vector<Task> &all_tasks);
    bool task_in_search_range(Vector a_loc, Vector t_loc, double search_range);
    bool are_all_visited(std::vector<bool> &dfs_results);
    int first_unvisited(std::vector<bool> &dfs_results);
    // void output_comm_graph(std::vector<std::vector<bool>> &comm_g);
}

#endif /* CONVEXHULL_H */