#include "debuggingFunctions.h"

namespace ns3 {
    void print_all_agent_costs(std::vector<Agent> &all_a){
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << "Agent #" << i << ": ";
            all_a[i].print_agent_costs();
        }
    }

    void print_cmatrix(std::vector<std::vector<double>> &c_mat){
        std::cout << "Printing Cost Matrix\n";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            for (unsigned long int j=0; j < globalInfo::allTasks.size(); j++){
                if (c_mat[i][j]==INT_MAX){
                    std::cout << "x ";
                }
                else std::cout << c_mat[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    // void print_comm_g(std::vector<std::vector<bool>> &comm_graph){
    //     for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
    //         for (unsigned long int j=0; j < globalInfo::allAgents.size(); j++){
    //             std::cout << comm_graph[i][j] << " ";
    //         }
    //         std::cout << "\n";
    //     }
    //     std::cout << "\n";
    // }

    void print_component_membership(std::vector<Agent> &all_a){
        for (unsigned long int i=0; i < all_a.size(); i++){
            std::cout << "Agent: " << i << "\tComponent: " << all_a[i].component_membership << "\n";
        }
    }

    void print_membership_vec(Agent &ag){
        std::cout << "Agent #" << ag.agent_id << "\tMembership Vector: ";
        for (int i=0; i < ag.numAgents; i++){
            std::cout << ag.component_arr[i] << " ";
        }
        std::cout << "\n";
    }

    void print_request(Agent &ag){
        std::cout << "Agent #" << ag.agent_id << "\tRequest: ";
        for (int i=0; i < ag.numAgents; i++){
            std::cout << ag.info_requests[i] << " ";
        }
        std::cout << "\n";
    }

    void print_ns3Vector(Vector &pos){
        std::cout << " (" << pos.x << "," << pos.y << "," << pos.z << ")";
    }

    void print_known_positions(AgentNode &ag){
        std::cout << "Agent #" << ag.agent->agent_id << "\tKnown positions:";
        for (int i=0; i < ag.agent->numAgents; i++){
            print_ns3Vector(ag.agent->known_positions[i]);
        }
        std::cout << "\n";
    }

    void print_single_request(send_request* req){
        std::cout << "Request from Agent #" << req->sender_id << ":\t";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << req->request[i] << " ";
        }
        std::cout << std::endl;
    }

    // void print_received_requests(Agent &ag){
    //     send_request* curr_req = ag.received_requests;
    //     int req_count=0;

    //     while(curr_req !=NULL){
    //         req_count++;
    //         print_single_request(curr_req);
    //         //move on to next one
    //         curr_req=curr_req->next_message;
    //     }
    //     std::cout << "Received a total of " << req_count << " requests!\n";
    // }

    void print_single_position_message(send_position* p_mess){
        std::cout << "Message from Agent #" << p_mess->sender_id << " regarding Agent #" << p_mess->info_id << "\t Position: ";
        print_ns3Vector(p_mess->position);
        std::cout << std::endl;
    }

    // void print_received_position_messages(Agent &ag){
    //     send_position* curr_mess = ag.received_positions;
    //     int mess_count=0;

    //     std::cout << "Position messages received by Agent #" << ag.agent_id << "\n";
    //     while (curr_mess != NULL){
    //         mess_count++;
    //         print_single_position_message(curr_mess);
    //         curr_mess=curr_mess->next_message;
    //     }
    //     std::cout << "Received a total of " << mess_count << " positions!\n";
    // }

    void print_small_cost_mat(std::vector<std::vector<double>> &cmat){
        int curr_rsize, curr_csize;
        curr_rsize=cmat.size();
        for (int i=0; i < curr_rsize; i++){
            curr_csize=cmat[i].size();
            for (int j=0; j < curr_csize; j++){
                std::cout << cmat[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void print_agent_info(std::vector<Agent> &all_a){
        std::cout << "Agent Information\n";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << "Agent #" << all_a[i].agent_id << "\n" << "Class #" << all_a[i].instrument_type
                    << "\n" << "Location: (" << all_a[i].agent_position.x << ", " << all_a[i].agent_position.y
                    << ", " << all_a[i].agent_position.z << ")\n";
        }
        std::cout << std::endl;
    }

    void print_global_assignment(std::vector<std::vector<int>> &g_assignment){
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            int curr_size=g_assignment[i].size();
            std::cout << "Task " << i << ": ";
            for (int j=0; j < curr_size; j++){
                std::cout << g_assignment[i][j] << "\t";
            }
            std::cout << std::endl;
        }
    }

    void print_global_conflicts(std::vector<bool> &conflicts){
        std::cout << "Conflicts: " << std::endl;
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << conflicts[i] << " ";
        }
        std::cout << std::endl;
    }

    void print_still_consider(std::vector<bool> consider, std::vector<int> task_ids){
        int curr_size = consider.size();
        for (int i=0; i < curr_size; i++){
            std::cout << "(" << task_ids[i] << "," << consider[i] << ") ";
        }
        std::cout << std::endl;
    }

    // void print_loc_ranges(std::vector<loc_range> &ranges){
    //     std::cout << "Printing Location Ranges:" << std::endl;
    //     for (int i=0; i < num_connected_components; i++){
    //         cout << "Min x: " << ranges[i].min_x << "\tMax x: " << ranges[i].max_x << std::endl;
    //         cout << "Min y: " << ranges[i].min_y << "\tMax y: " << ranges[i].max_y << std::endl << std::endl;
    //     }
    // }

    void print_task_info(std::vector<Task> &all_ts){
        std::cout << "Task Information" << std::endl;
        for (unsigned long int i=0; i < globalInfo::allTasks.size(); i++){
            std::cout << "Task #" << all_ts[i].task_id << "\n" << "Class #" << all_ts[i].instrument_requirement << "\n" << "Location: (" << all_ts[i].task_location.x << ", " << all_ts[i].task_location.y << ", " << all_ts[i].task_location.z << ")\n";
        }
        std::cout << std::endl;
    }

    void print_instrument_assignment_vec(){
        std::cout << "Instrument Assignment: ";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << globalInfo::instrument_assignment[i] << " ";
        }
        std::cout << "\n";
    }
}