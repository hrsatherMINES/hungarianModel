// This is code for impementing a convex hull which I did not need to do.
// Also some code here creates connected components which NS3 does for you already

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"

using namespace ns3;

int orientation(Vector p, Vector q, Vector r){
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

std::vector<Vector> convexHull(std::vector<Agent> &all_ags, int connected_component){
    int num_in_component=0;
    std::vector<Vector> points;
    for (unsigned long int i=0; i < all_ags.size(); i++){
        //std::cout << all_ags[i].component_membership << std::endl;
        if (all_ags[i].component_membership == connected_component){
            num_in_component++;
            points.push_back(all_ags[i].agent_position);
        }
    }

    // Initialize Result
    std::vector<Vector> hull;

    // There must be at least 3 points
    if (num_in_component < 3){
        std::cerr << "Only " << num_in_component << " members in component" << std::endl;
        return hull;
    }

    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < num_in_component; i++)
        if (points[i].x < points[l].x)
            l = i;

    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
    do{
        // Add current point to result
        hull.push_back(points[p]);

        // Search for a point 'q' such that orientation(p, x,
        // q) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q = (p+1)%num_in_component;
        for (int i = 0; i < num_in_component; i++){
           // If i is more counterclockwise than current q, then
           // update q
           if (orientation(points[p], points[i], points[q]) == 2)
               q = i;
        }

        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;

    } while (p != l);  // While we don't come to first point

    // // Print Result
    // for (int i = 0; i < int(hull.size()); i++)
    //     cout << "(" << hull[i].x << ", "
    //           << hull[i].y << ")\n";
    return hull;
}

void overwrite_hull_file(string filename){
    std::ofstream output (filename);
    output.close();
}

void output_hull(std::vector<Vector> hull, int component_id, string filename){
    std::fstream output (filename, output.out | output.app);
    int vec_size=hull.size();
    output << component_id << delim << vec_size << std::endl;
    for (int i=0; i < vec_size; i++){
       output << hull[i].x << delim << hull[i].y << delim << hull[i].z << std::endl;
    }
    output.close();
}

//Implementation modified from http://geomalgorithms.com/a03-_inclusion.html
// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
int isLeft(Vector P0, Vector P1, Vector P2){
    return ( (P1.x - P0.x) * (P2.y - P0.y)
            - (P2.x -  P0.x) * (P1.y - P0.y) );
}

// cn_PnPoly(): crossing number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  0 = outside, 1 = inside
int cn_PnPoly( Vector P, std::vector<Vector> V, int n ){
    int    cn = 0;    // the  crossing number counter

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {    // edge from V[i]  to V[i+1]
       if (((V[i].y <= P.y) && (V[i+1].y > P.y))     // an upward crossing
        || ((V[i].y > P.y) && (V[i+1].y <=  P.y))) { // a downward crossing
            // compute  the actual edge-ray intersect x-coordinate
            float vt = (float)(P.y  - V[i].y) / (V[i+1].y - V[i].y);
            if (P.x <  V[i].x + vt * (V[i+1].x - V[i].x)) // P.x < intersect
                 ++cn;   // a valid crossing of y=P.y right of P.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if  odd (in)
}

// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)
// Winding number =0 means the point is outside
// otherwise point is inside polygon
int wn_PnPoly( Vector P, std::vector<Vector> V, int n ){
    int  wn = 0;    // the  winding number counter
    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to  V[i+1]
        if (V[i].y <= P.y) {          // start y <= P.y
            if (V[i+1].y  > P.y)      // an upward crossing
                 if (isLeft( V[i], V[i+1], P) > 0)  // P left of  edge
                     ++wn;            // have  a valid up intersect
        }
        else {                        // start y > P.y (no test needed)
            if (V[i+1].y  <= P.y)     // a downward crossing
                 if (isLeft( V[i], V[i+1], P) < 0)  // P right of  edge
                     --wn;            // have  a valid down intersect
        }
    }
    return wn;
}

// std::vector<std::vector<bool>> create_component_communication_graph(std::vector<AgentNode> &all_a, int start, int end){
//     std::vector<std::vector<bool>> comm_g;
//     double dist;
//     for (int i=start; i < end; i++){
//         Vector curr_agent = all_a[i].agent->agent_position;
//         std::vector<bool> one_row;
//         for (int j=start; j < end; j++){
//             if (i!=j){
//                 dist=euc_dist(curr_agent, all_a[j].agent->agent_position);
//                 if (dist <= comm_thresh){
//                     one_row.push_back(true);
//                 }
//                 else one_row.push_back(false);
//             }
//             else one_row.push_back(false);
//         }
//         comm_g.push_back(one_row);
//     }
//     return comm_g;
// }

std::vector<bool> check_connectivity(std::vector<std::vector<bool>> comp_g, int n){
    std::vector<bool> visited;
    for(int i=0; i < n; i++){
        visited.push_back(false);
    }

    //create a queue and start with the first agent
    std::queue<int> q; //store agent ids in the queue
    q.push(0); //first agent
    visited[0]=true;

    //Now perform a BFS
    while (!q.empty()){
        int exp=q.front(); //save agent that we are exploring
        q.pop();

        for (int i=0; i < n; i++){
            if (visited[i]==false && comp_g[exp][i]){ //if the agent hasn't been visited and the current agent is connected to it in the communication graph
                q.push(i);
                visited[i]=true;
            }
        }
    }
    return visited;
}

void create_connected_components(std::vector<AgentNode> &all_a, int num_components, std::vector<loc_range> &all_ranges){
    // Ptr<UniformRandomVariable> x_rand_stream = CreateObject<UniformRandomVariable> ();
    // int curr_agent=0;
    // int starting_agent;
    // int x,y,z;
    // z=0;
    // for (int i=0; i < num_connected_components; i++){
    //     int numAgents_in_component = all_ranges[i].num_agents_in_range;
    //     starting_agent=curr_agent;
    //     //generate random streams for the component
    //     x_rand_stream->SetAttribute("Min", DoubleValue(all_ranges[i].min_x));
    //     x_rand_stream->SetAttribute("Max", DoubleValue(all_ranges[i].max_x));

    //     for (int j=0; j< numAgents_in_component; j++){
    //         x=x_rand_stream->GetInteger();
    //         y=rand_stream->GetInteger();
    //         all_a[curr_agent].agent->agent_position.x = x;
    //         all_a[curr_agent].agent->agent_position.y = y;
    //         all_a[curr_agent].agent->agent_position.z = z;
    //         update_position(&all_a[curr_agent]);
    //         curr_agent++;
    //     }

    //     //now ensure the component is connected
    //     std::vector<std::vector<bool>> comp_g;
    //     while(true){
    //         comp_g=create_component_communication_graph(all_a, starting_agent, curr_agent);
    //         std::vector<bool> dfs_result = check_connectivity(comp_g, numAgents_in_component);
    //         if (are_all_visited(dfs_result)){
    //             break;
    //         }
    //         //move some agents then
    //         int agent_to_move = first_unvisited(dfs_result);
    //         agent_to_move+=starting_agent;
    //         x=x_rand_stream->GetInteger();
    //         y=rand_stream->GetInteger();
    //         all_a[curr_agent].agent->agent_position.x = x;
    //         all_a[curr_agent].agent->agent_position.y = y;
    //         all_a[curr_agent].agent->agent_position.z = z;
    //         update_position(&all_a[agent_to_move]);
    //     }
    // }
}

double compute_reassignment(Agent &ag){
    std::vector<std::vector<double>> cost_mat;
    int mat_size = ag.c_info->agents_in_conflict.size();

    //create empty matrix of costs of all agents in conflict
    for (int i=0; i < mat_size; i++){
        std::vector<double> one_row (mat_size, INT_MAX);
        cost_mat.push_back(one_row);
    }

    // for (int i=0; i < mat_size; i++){
    //     int temp_ag = ag.c_info->agents_in_conflict[i];
    //     for (int j=0; j < mat_size; j++){
    //         int curr_task_id = ag.c_info->tasks_in_conflict[j];
    //         cost_mat[i][j] = costMatrix[temp_ag][curr_task_id];
    //     }
    // }

    //now comupute a new assignment using Hungarian method
    HungarianAlgorithm conflictHungarian;
    std::vector<int> unmappedResolution;
    double conflict_cost = conflictHungarian.Solve(cost_mat, unmappedResolution);

    //now map the assignment back to the actual
    for (int i=0; i < mat_size; i++){
        int temp_ag, temp_task;
        temp_ag=ag.c_info->agents_in_conflict[i];
        temp_task=ag.c_info->tasks_in_conflict[unmappedResolution[i]];
        ag.partial_assignment[temp_ag]=temp_task;
    }
    return conflict_cost;
}

//Range expansion implementation
// double range_expansion(std::vector<Agent> &all_ag, int curr_id, std::vector<Task> &all_tasks){
//     std::vector<std::vector<double>> cost_mat;
//     std::vector<int> tasks_to_consider; //tasks to consider that the agent is eligible for
//     std::vector<bool> still_consider;
//     std::vector<int> competing_agents; //agents of same instrument class in component
//     int curr_inst = all_ag[curr_id].instrument_type;

//     //figure out agents of same instrument class in the connected component
//     for (int i=0; i < globalInfo::allAgents.size(); i++){
//         //if (all_ag[curr_id].component_arr[i]){
//             if (all_ag[i].instrument_type==curr_inst){
//                 competing_agents.push_back(i);
//             }
//         //}

//         if (i == curr_id){
//             competing_agents.push_back(i);
//         }
//     }

//     //figure out which tasks to search through
//     for (int i=0; i < globalInfo::allAgents.size(); i++){
//         if (all_tasks[i].instrument_requirement==curr_inst){
//             tasks_to_consider.push_back(i);
//             still_consider.push_back(true);
//         }
//     }

//     int num_competing_agents = competing_agents.size();
//     int numTasks_to_consider = tasks_to_consider.size();
//     //std::cout << num_competing_agents << " " << numTasks_to_consider << std::endl;

//     //find enough eligible tasks
//     std::vector<int> eligible_tasks;
//     double range_to_search = comm_thresh;
//     int curr_size;
//     while(true){
//         for (int i=0; i < num_competing_agents; i++){
//             for (int j=0; j < numTasks_to_consider; j++){
//                 if (still_consider[j]){
//                     int ag_num=competing_agents[i];
//                     int task_num=tasks_to_consider[j];
//                     bool verdict=task_in_search_range(all_ag[ag_num].agent_position, all_tasks[task_num].task_location, range_to_search);
//                     if (verdict)
//                     {
//                         still_consider[j]=false; //no longer consider task
//                         eligible_tasks.push_back(task_num);
//                         curr_size = eligible_tasks.size();
//                         if (curr_size >= num_competing_agents)
//                         {
//                             break;
//                         }
//                     }
//                 }
//             }
//             curr_size = eligible_tasks.size();
//             if (curr_size >= num_competing_agents){
//                 break;
//             }
//         }

//         curr_size = eligible_tasks.size();
//         if (curr_size >= num_competing_agents){
//             break;
//         }
//         range_to_search+=comm_thresh; //expand search range
//     }

//     //create cost matrix out of tasks and agents
//     for (int i=0; i < num_competing_agents; i++){
//         std::vector<double> one_row (num_competing_agents, INT_MAX);
//         cost_mat.push_back(one_row);
//     }

//     for (int i=0; i < num_competing_agents; i++){
//         int ag_id=competing_agents[i];
//         for (int j=0; j < num_competing_agents; j++){
//             int task_num = eligible_tasks[j];
//             cost_mat[i][j]=costMatrix[ag_id][task_num];
//         }
//     }

//     HungarianAlgorithm rangeHungarian;
//     std::vector<int> unmappedResolution;
//     double range_cost = rangeHungarian.Solve(cost_mat, unmappedResolution);

//     //now map the assignment back to the actual
//     for (int i=0; i < num_competing_agents; i++){
//         int temp_ag, temp_task;
//         temp_ag=competing_agents[i];
//         temp_task=eligible_tasks[unmappedResolution[i]];
//         all_ag[curr_id].partial_assignment[temp_ag]=temp_task;
//     }
//     return range_cost;
// }

// void range_expansion_all_agents(std::vector<Agent> &all_ag, std::vector<Task> &all_tasks){
//     for(unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
//         range_expansion(all_ag, i, all_tasks);
//     }
// }

//is tasks within current range
bool task_in_search_range(Vector a_loc, Vector t_loc, double search_range){
    double curr_dist = euc_dist(a_loc, t_loc);
    if (curr_dist < search_range){
        return true;
    }
    else return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//Code to create x number of connected components

bool are_all_visited(std::vector<bool> &dfs_results){
    int num = dfs_results.size();
    for (int i=0; i < num; i++){
        if (dfs_results[i] == false){
            return false;
        }
    }
    return true;
}

int first_unvisited(std::vector<bool> &dfs_results){
    int n = dfs_results.size();
    for (int i=0; i < n; i++){
        if (!dfs_results[i]){
            return i;
        }
    }
    return 0; //shouldn't get here
}

// void output_comm_graph(std::vector<std::vector<bool>> &comm_g){
//     std::ofstream output ("TA_sim_comm_graph.csv");

//     for (int i=0; i < globalInfo::allAgents.size(); i++){
//         for (int j=0; j < globalInfo::allAgents.size(); j++){
//             output << comm_g[i][j];
//             if (j < globalInfo::allAgents.size()-1){
//                 output << delim;
//             }
//         }
//         output << std::endl;
//     }
//     output.close();
// }

// std::vector<loc_range> create_loc_ranges(){
//     std::vector<loc_range> all_ranges;

//     int working_area = floor((maxPosition-(comm_thresh*num_connected_components))/num_connected_components);
//     int x = 0;

//     //divide into vertical slices
//     for(int i=0; i < num_connected_components; i++){
//         loc_range curr;
//         //assign all of the vertical slice
//         curr.min_y=0;
//         curr.max_y=maxPosition;

//         //horizontal slice
//         curr.min_x=x;
//         curr.max_x=x+working_area;

//         curr.num_agents_in_range=comp_breakdown[i];

//         all_ranges.push_back(curr);

//         x=curr.max_x+comm_thresh;
//     }

//     return all_ranges;
// }

//determine initial communication graph
// std::vector<std::vector<bool>> create_init_comm_graph(std::vector<AgentNode> &all_a){
//     std::vector<std::vector<bool>> comm_g;
//     double dist;
//     for (unsigned long int i=0; i < all_a.size(); i++){
//         Vector curr_agent = all_a[i].agent->agent_position;
//         std::vector<bool> one_row;
//         for (unsigned long int j=0; j < all_a.size(); j++){
//             if (i != j){
//                 dist=euc_dist(curr_agent, all_a[j].agent->agent_position);
//                 if (dist <= comm_thresh){
//                     //std::cout << i << " can talk to " << j << std::endl;
//                     one_row.push_back(true);
//                 }
//                 else one_row.push_back(false);
//             }
//             else one_row.push_back(false);
//         }
//         comm_g.push_back(one_row);
//     }
//     return comm_g;
// }



//check if all agents have been visited
//returns index of first unvisited agent (-1 if all visited)
// int all_agents_visited(std::vector<bool> &visit_list){
//     for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
//         if (visit_list[i]==false){
//             return i;
//         }
//     }
//     return -1;
// }

//perform a bfs to determine connected components
//assign connected component variable for each agent as well
void determine_connected_components(std::vector<AgentNode> &all_a, std::vector<std::vector<bool>> &commGraph){
    std::vector<bool> visited_agents(all_a.size(), false); //initialize all as unvisited
    int component_label=0; //labeling will actually start at 0 due to ++ in for loop
    for (unsigned long int i=0; i < all_a.size(); i++){
        if (visited_agents[i]==false){ //need to start bfs from here
            component_label++;
            std::queue<int> bfs_q;
            bfs_q.push(i); //enqueue element
            while (!bfs_q.empty()){
                int curr_agent=bfs_q.front();
                bfs_q.pop(); //remove from queue

                //set current agent to connected component
                all_a[curr_agent].agent->component_membership=component_label;

                for (unsigned long int j=0; j < all_a.size(); j++){
                    //if there is a link and unvisited
                    if ((commGraph[curr_agent][j]) && (visited_agents[j]==false))
                    {
                        visited_agents[j]=true;
                        bfs_q.push(j); //add to queue
                    }
                }
            }
        }
    }
}

//fills in an array telling each agent who is part of their connected component
void fill_in_component_arrs(std::vector<AgentNode> &all_a){
    for (unsigned long int j=0; j < all_a.size(); j++){
        int curr_memb=all_a[j].agent->component_membership;
        //give each agent an empty array
        all_a[j].agent->component_arr = new bool [all_a.size()];

        //fill in the array
        for (unsigned long int i=0; i < all_a.size(); i++){
            if ((all_a[i].agent->component_membership==curr_memb) && (i!=j)){
                all_a[j].agent->component_arr[i]=true;
                //std::cout << i << " is in the same component as " << j << std::endl;
            }
            else{
                all_a[j].agent->component_arr[i]=false;
            }
        }
    }
}