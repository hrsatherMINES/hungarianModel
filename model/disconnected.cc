/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/disconnected.h"


using namespace ns3;
//global/experiment variables

int num_connected_components;
std::vector<int> comp_breakdown; //how the connected components are broken down
std::vector<bool> agents_moving;

string delim = ",";

//For randomized (but reproducible) experiments
Ptr<UniformRandomVariable> rand_stream;
int seed;
int run;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("DisconnectedSwarmSim");




void Task::update_location(double x, double y, double z){
    task_location=Vector (x, y, z);
}

double euc_dist(Vector t_pos, Vector a_pos){
    double xdiff, ydiff, zdiff, dist;
    xdiff=t_pos.x-a_pos.x;
    ydiff=t_pos.y-a_pos.y;
    zdiff=t_pos.z-a_pos.z;
    dist=sqrt((xdiff*xdiff)+(ydiff*ydiff)+(zdiff*zdiff));
    return dist;
}

Vector add_ns3Vectors(Vector v1, Vector v2){
    Vector vec_sum;
    vec_sum.x=v1.x+v2.x;
    vec_sum.y=v1.y+v2.y;
    vec_sum.z=v1.z+v2.z;
    return vec_sum;
}

void update_position(AgentNode* ag){
    Vector pos = GetPosition(ag->node);
    pos.x = ag->agent->agent_position.x;
    // std::cout << pos.x << "HERE" << std::endl;
    pos.y = ag->agent->agent_position.y;
    pos.z = ag->agent->agent_position.z;
    SetPosition (ag->node, pos);
}


int get_instrument_type(int id){
    return id/globalInfo::agents_per_class;
}

std::vector<Task> create_tasks(){
    std::vector<Task> all_tasks;
    all_tasks.reserve(globalInfo::allTasks.size());
    double x, y, z;
    z=0.0;

    for (unsigned long int i=0; i < globalInfo::allTasks.size(); i++){
        all_tasks[i].task_id=i;
        x=rand_stream->GetInteger();
        y=rand_stream->GetInteger();
        all_tasks[i].update_location(x, y, z);
        all_tasks[i].instrument_requirement = get_instrument_type(i);
    }
    return all_tasks;
}

std::vector<Agent*> create_agents(){
    std::vector<Agent*> all_agents;
    for (int i=0; i < globalInfo::numAgents; i++){
        //set received messages to null
        Agent* tempAgent = new Agent();

        tempAgent->numAgents = globalInfo::numAgents;
        
        //no messages sent
        tempAgent->num_position_messages_sent=0;
        tempAgent->num_request_messages_sent=0;
        tempAgent->num_score_info_messages_sent=0;
        // //other initialization
        tempAgent->agent_id=i;
        tempAgent->distance_traveled=0; //initialize
        tempAgent->distance_left=0; //set to actual value after assignment
        tempAgent->instrument_type=get_instrument_type(i);
        tempAgent->initialize_partial_assignment();
        tempAgent->initialize_info_requests();
        tempAgent->initialize_known_positions();
        tempAgent->initialize_known_info();
        tempAgent->initialize_cinfo();
        tempAgent->initialize_received_times();
        tempAgent->have_all_needed_info=false;
        //construct 2D who_requested array
        bool** arr = new bool*[globalInfo::numAgents];
        for (int j=0; j < globalInfo::numAgents; j++){
            arr[j] = new bool[globalInfo::numAgents];
        }
        tempAgent->who_requested = arr;
        all_agents.push_back(tempAgent);
        globalInfo::instrument_assignment.at(i)=tempAgent->instrument_type;
        
    }
    return all_agents;
}

void fillAllLocalCosts(std::vector<AgentNode> &all_a){
    for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        all_a.at(i).agent->fillLocalCostMatrix();
    }
}

void calculate_all_costs(std::vector<TaskNode> allTasks, std::vector<AgentNode> all_a){
    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        all_a[i].agent->fill_in_agent_costs(allTasks);
    }
}

std::vector<std::vector<double>> create_cmatrix(std::vector<AgentNode> &all_a){
    //just copy data over into 2D vector for hungarian method
    std::vector<std::vector<double>> all_costs;
    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        std::vector<double> one_row;
        for (unsigned long int j=0; j < globalInfo::allTasks.size(); j++){
            one_row.push_back(all_a[i].agent->task_costs[j]);
        }
        all_costs.push_back(one_row);
    }
    return all_costs;
}



void initialize_all_needed_info(std::vector<AgentNode> &all_a){
    for (unsigned long int i=0; i < all_a.size(); i++){
        all_a[i].agent->initialize_needed_info();
    }
}

//fill in initial request array with needed info (more will be added once shared)
//also set up previous sent request
void initialize_all_requests(std::vector<AgentNode> &all_a){
    for (unsigned long int i=0; i < all_a.size(); i++){
        for (unsigned long int j=0; j < all_a.size(); j++){
            all_a[i].agent->info_requests[j]=all_a[i].agent->needed_info[j];
            all_a[i].agent->previous_sent_request[j]=all_a[i].agent->needed_info[j];
        }
    }
}

///////////////////////////////////////////////////////////////////////////
//Functions to free up memory

void delete_request_message(send_request* mess){
    //delete array first
    delete [] mess->request;
    //delete message
    delete mess;
}

void delete_who_requested(bool** arr){
    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        delete [] arr [i];
    }
    delete [] arr;
}

///////////////////////////////////////////////////////////////////////////
//Data dissemination related functions

//each agent will share its current request vector with each of its neighbors
void initial_request_sharing(std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface){
    for (unsigned long int i=0; i < all_a.size(); i++){
        all_a[i].agent->num_request_messages_sent++;
        for (unsigned long int j=0; j < all_a.size(); j++){
            //if (comm_g[i][j]){
              if(i != j){
                send_request_info(all_a[i], all_a[j], interface); //TODO SEND OVER NS3
              }
            //}
        }
    }

    //now everyone performs merge function on all received requests
    merge_all_received_requests(all_a);
}

double fRand(double fMin, double fMax){
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

Vector GetPosition (Ptr<Node> node){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void SetPosition (Ptr<Node> node, Vector position){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

void movePositions(Ptr<Node> node){
  Vector pos = GetPosition(node);
  pos.x += 1;
  pos.y += 1;
  SetPosition (node, pos);
}

void moveAllPositions(NodeContainer robots){
  for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
    movePositions(robots.Get(i));
  }
}

//check if request has changed since it was last sent
bool compare_bool_arr(bool* prev, bool* req){
    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        if (prev[i]!=req[i]) return false;
    }
    return true;
}

bool check_if_agent_has_needed_info(Agent *ag){
    if (ag->have_all_needed_info){
        return true;
    }
    else //iterate through and needed_info and compare to known_info to check if status needs to be updated
    {
        for (int i=0; i < ag->numAgents; i++){
            if (!(ag->needed_info[i])){
                continue; //move onto next
            }
            //if needed but doesn't know it then return false
            else if (ag->needed_info[i] && !ag->known_info[i]){
                return false;
            }
            //other case is that it has the info it needs
        }
        //if it gets here it has all the info it needs
        ag->have_all_needed_info=true;
        return true;
    }
}

bool check_all_needed_info(std::vector<AgentNode> &all_a){
    for (unsigned long int i=0; i < all_a.size(); i++){
        bool curr_verdict = check_if_agent_has_needed_info(all_a[i].agent);
        if (!curr_verdict){ //an agent does not have all needed info
            return false;
        }
    }
    return true; // only gets here if all agents report true
}

//determines if an agent has any position information that a neighbor requested
//fills the position message buffer with messages to send
void determine_position_messages_to_send(AgentNode ag){
    //to store which info agent needs to send
    //int curr_ag = ag.agent->agent_id;

    for (int i=0; i < ag.agent->numAgents; i++){
        //if(comm_g[curr_ag][i]){ //check communication
            for (int j=0; j < ag.agent->numAgents; j++){
                //if we have the info and a neighbor requested it
                if ((ag.agent->known_info[j]) && (ag.agent->who_requested[i][j])){
                    ag.agent->which_positions_to_send[j]=true;
                    //since we are going to send it to that particular agent
                    //mark their request as fulfilled
                    ag.agent->who_requested[i][j]=false;
                    ag.agent->info_requests[j]=false; //don't need to request info we have and have sent out
                }
            }
        //}
    }
}

//determines if an agent has any position information that a neighbor requested
//fills the position message buffer with messages to send
void determine_to_send_all_information(Agent *ag){
    //to store which info agent needs to send
    //int curr_ag = ag->agent_id;

    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        //if(comm_g[curr_ag][i]){
            for (unsigned long int j=0; j< globalInfo::allAgents.size(); j++){
                //if we have the info and a neighbor requested it
                if (ag->known_info[j]){
                    ag->which_positions_to_send[j]=true;
                    //since we are going to send it to that particular agent
                    //mark their request as fulfilled
                    ag->who_requested[i][j]=false;
                    ag->info_requests[j]=false; //don't need to request info we have and have sent out
                }
            }
        //}
    }
}

void send_position_messages_in_buffer(int curr_ag, std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface){
    for (unsigned long int i=0; i < all_a.size(); i++){
        if (all_a[curr_ag].agent->which_positions_to_send[i]){
            all_a[curr_ag].agent->which_positions_to_send[i]=false;
            all_a[curr_ag].agent->num_position_messages_sent++;

            //broadcast message to neighbors
            for(unsigned long int j=0; j < all_a.size(); j++){
                //if (comm_g[curr_ag][j]){//if j is a neighbor
                if((int)j != curr_ag){
                    send_position_info(all_a[curr_ag], all_a[j], i, interface);
                }
                //}
            }
        }
    }
}

void all_send_position_info(std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface){
    //have all agents determine what to send and then send it out
    for (unsigned long int i=0; i < all_a.size(); i++){
        determine_position_messages_to_send(all_a[i]);
        send_position_messages_in_buffer(i, all_a, interface);
    }
    for (unsigned long int i=0; i < all_a.size(); i++){
        merge_received_positions(all_a[i].agent);
    }
}

void all_send_all_position_info(std::vector<AgentNode> &all_a, Ipv4InterfaceContainer interface){
    //have all agents send everything they know. Good for debugging
    for (unsigned long int i=0; i < all_a.size(); i++){
        determine_to_send_all_information(all_a[i].agent);
        send_position_messages_in_buffer(i, all_a, interface);
    }
    for (unsigned long int i=0; i < all_a.size(); i++){
        merge_received_positions(all_a[i].agent);
    }
}


////////////////////////////////////////////////////////////////////////
//Partial Assignment related functions

//Once information has been disseminated need to compute partial assignments
double compute_partial_assignment_hungarian(Agent *ag, std::vector<TaskNode> &all_tasks){
    std::vector<vector<double>> cost_mat;
    std::vector<int> task_locs; //will store indices of tasks
    std::vector<int> agents_needed; //will store indices of agents

    int curr_instrument = ag->instrument_type;
    for (int i=0; i < ag->numAgents; i++){
        if (all_tasks[i].task->instrument_requirement==curr_instrument){
            task_locs.push_back(i);
        }
    }

    //fill in agents_needed vector
    //Basically just add agents of the same instrument class
    //determine if that information is known when filling in matrix
    int curr_inst = ag->instrument_type;
    for (int i=0; i < ag->numAgents; i++){
        if (globalInfo::instrument_assignment[i]==curr_inst){
            agents_needed.push_back(i);
        }
    }
    
    //will be numAgents_in_instrument_class x numTasks_for_class big (square)
    for (int i=0; i < globalInfo::agents_per_class; i++){
        std::vector<double> one_row (globalInfo::agents_per_class, INT_MAX);
        cost_mat.push_back(one_row);
    }

    int num_ts = task_locs.size();
    int num_needed= agents_needed.size();
    //std::cout << numTasks << " " << num_needed << std::endl;
    //now fill in the smaller cost matrix using the big one
    //at this point all agents should have the information they need to compute the scores they need
    //to save time (rather than to recompute) just grab info from costMatrix (used for optimal)
    for (int i=0; i < num_needed; i++){
        int curr_ag_id =agents_needed[i];
        if (ag->known_info[curr_ag_id]){
            int curr_task_id;
            for (int j=0; j < num_ts; j++){
                curr_task_id=task_locs[j];

                cost_mat[i][j]=ag->localCostMatrix[curr_ag_id][curr_task_id];
            }
        }
    }
    //check to see that matrix is filled in correctly
    //print_small_cost_mat(cost_mat);

    //now use the smaller matrix to compute an assignment using hungarian method
    HungarianAlgorithm partialHungarian;
    std::vector<int> unmapped_partialAssignment;
    double partial_cost = partialHungarian.Solve(cost_mat, unmapped_partialAssignment);

    //now map the partial assignment back to the actual
    for (int i = 0; i < num_needed; i++){
        int curr_ag, curr_task;
        curr_ag=agents_needed[i];
        curr_task= task_locs[unmapped_partialAssignment[i]];
        if (ag->known_info[curr_ag]){
            ag->partial_assignment[curr_ag]=curr_task;
        }
    }

    return partial_cost;
}

void compute_all_parital_assignments_hungarian(std::vector<AgentNode> &all_ags, std::vector<TaskNode> &all_ts){
    for (unsigned long int i=0; i < all_ags.size(); i++){
        compute_partial_assignment_hungarian(all_ags[i].agent, all_ts);
    }
}
//////////////////////////////////////////////////////////////////////////
//Output file related functions
// void overwrite_output_file(){
//     std::ofstream output ("TA_sim.csv");
//     output <<  numAgents << delim << numTasks << delim << num_instrument_classes << std::endl;
//     output.close();
// }

void write_initial_positions_to_file(std::vector<Agent> &all_ags, std::vector<Task> &all_ts){
    std::fstream output ("TA_sim.csv", output.out | output.app);

    //first write the positions of tasks
    for (unsigned long int i=0; i < globalInfo::allTasks.size(); i++){
        Vector pos = all_ts[i].task_location;
        output << all_ts[i].task_id << delim << all_ts[i].instrument_requirement << delim << pos.x << delim << pos.y << delim << pos.z << std::endl;
    }

    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        Vector pos = all_ags[i].agent_position;
        output << all_ags[i].agent_id << delim << all_ags[i].instrument_type << delim << pos.x << delim << pos.y<< delim << pos.z << std::endl;
    }
    output.close();
}

void write_position_to_file(Agent &ag){
    std::fstream output ("TA_sim.csv", output.out | output.app);
    Vector pos = ag.agent_position;
    output << ag.agent_id << delim << pos.x << delim << pos.y<< delim << pos.z << std::endl;
    output.close();
}

void write_file_end(){
    std::fstream output ("TA_sim.csv", output.out | output.app);
    output << -1 << endl;
    output.close();
}


//////////////////////////////////////////////////////////////////////////
//Movement related functions
double vector_magnitude(double x, double y, double z){
    double magnitude = x*x+y*y+z*z;
    magnitude=sqrt(magnitude);
    return magnitude;
}

Vector diff_Vector(Vector &ag_pos, Vector &task_pos){
    double xdiff, ydiff, zdiff;
    xdiff=task_pos.x-ag_pos.x;
    ydiff=task_pos.y-ag_pos.y;
    zdiff=task_pos.z-ag_pos.z;
    return Vector(xdiff,ydiff,zdiff);
}

Vector create_movement_vector(Vector &ag_pos, Vector &task_pos){
    Vector diff = diff_Vector(ag_pos, task_pos);

    double magnitude = vector_magnitude(diff.x, diff.y, diff.z);
    if(magnitude < 0.01){  // If on position
      diff.x = 0;
      diff.y = 0;
      diff.z = 0;
    }
    else{
      diff.x=diff.x/magnitude;
      diff.y=diff.y/magnitude;
      diff.z=diff.z/magnitude;
    }

    return diff;
}

void determine_assigned_location(std::vector<AgentNode> &all_ags, std::vector<TaskNode> &all_ts){
    //fill in each agent's assigned position and movement vector
    for (unsigned long int i=0; i < all_ags.size(); i++){
        int curr_id = all_ags[i].agent->agent_id;
        int assigned_task = all_ags[i].agent->partial_assignment[curr_id];
        all_ags[i].agent->assigned_task_position=all_ts[assigned_task].task->task_location;
        all_ags[i].agent->movement_vec=create_movement_vector(all_ags[i].agent->agent_position, all_ags[i].agent->assigned_task_position);
        Vector diff=diff_Vector(all_ags[i].agent->agent_position, all_ags[i].agent->assigned_task_position);
        double dist_left=vector_magnitude(diff.x, diff.y, diff.z); //initialize distance left to travel
        all_ags[i].agent->distance_left=dist_left;
    }
}

void move_agent_towards_goal_step(AgentNode ag){
    Vector current_pos = ag.agent->agent_position;

    double x_movement = ag.agent->movement_vec.x * ag.agent->speed;
    double y_movement = ag.agent->movement_vec.y * ag.agent->speed;
    double z_movement = ag.agent->movement_vec.z * ag.agent->speed;

    // X position
    if(abs(current_pos.x - ag.agent->assigned_task_position.x) <= abs(x_movement)){
      current_pos.x = ag.agent->assigned_task_position.x;
    }
    else{
      current_pos.x += x_movement;
    }
    // Y position
    if(abs(current_pos.y - ag.agent->assigned_task_position.y) <= abs(y_movement)){
      current_pos.y = ag.agent->assigned_task_position.y;
    }
    else{
      current_pos.y += y_movement;
    }
    // Z positions
    if(abs(current_pos.z - ag.agent->assigned_task_position.z) <= abs(z_movement)){
      current_pos.z += ag.agent->assigned_task_position.z;
    }
    else{
      current_pos.z += z_movement;
    }

    int distance_for_step = sqrt(x_movement*x_movement + y_movement*y_movement + z_movement*z_movement);
    ag.agent->distance_traveled += distance_for_step;
    ag.agent->agent_position = current_pos;
    ag.agent->known_positions[ag.agent->agent_id] = current_pos;

    update_position(&ag);
}

void move_all_agents_towards_goal_step(std::vector<AgentNode> &all_ags){
    for (unsigned long int i=0; i < all_ags.size(); i++){
        move_agent_towards_goal_step(all_ags[i]);
        //write_position_to_file(all_ags[i]);
    }
    //write_file_end();
}


////////////////////////////////////////////////
//Determine assignment conflicts
std::vector<std::vector<int>> global_assignment_info(std::vector<Agent> &all_a){
    std::vector<std::vector<int>> num_assigned; //stores who is assigned to each task
    for (unsigned long int i=0; i < all_a.size(); i++){
        std::vector<int> temp;
        num_assigned.push_back(temp);
    }

    for (unsigned long int i=0; i < all_a.size(); i++){
        int assigned_task = all_a[i].partial_assignment[i];
        num_assigned[assigned_task].push_back(i);
    }

    return num_assigned;
}

std::vector<bool> determine_global_conflicts(std::vector<std::vector<int>> &g_assignment){
    std::vector<bool> conflicts;
    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        int curr_size = g_assignment[i].size();
        if (curr_size != 1){
            conflicts.push_back(true);
        }
        else{
            conflicts.push_back(false);
        }
    }
    return conflicts;
}

std::vector<bool> determine_agents_with_conflicts(std::vector<std::vector<int>> &g_assignment, std::vector<bool> &conflicts){
    std::vector<bool> agents_in_conflict;
    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        agents_in_conflict.push_back(false);
    }

    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        if (conflicts[i]){
            int curr_size = g_assignment[i].size();
            for (int j=0; j < curr_size; j++){
                agents_in_conflict[g_assignment[i][j]] = true;
            }
        }
    }

    return agents_in_conflict;
}

bool conflicts_exist(std::vector<AgentNode> &all_a){
    std::vector<int> allPoints;
    bool conflictsExist = false;
    for(unsigned long int i = 0; i < all_a.size(); i++){
        for(unsigned long int j = 0; j < all_a.size(); j++){
              if(all_a[i].agent->agent_position.x == all_a[j].agent->agent_position.x
                && all_a[i].agent->agent_position.y == all_a[j].agent->agent_position.y
                && all_a[i].agent->agent_position.z == all_a[j].agent->agent_position.z
                && i != j){
                  std::cout << "Conflict at " << i << " with " << j << " at "
                      << "(" << all_a[j].agent->agent_position.x << "," << all_a[j].agent->agent_position.y
                      << "," << all_a[j].agent->agent_position.z << ") "<< std::endl;
                  conflictsExist = true;
              }
        }
    }
    return conflictsExist;
}

void create_agent_conflict_info(std::vector<std::vector<int>> &g_assignment, std::vector<bool> &conflicts, Agent &a, std::vector<Task> &all_tasks){
    int a_type = a.instrument_type;
    for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
        //std::cout << i << std::endl;
        if (conflicts[i]){
            if (all_tasks[i].instrument_requirement==a_type){ //if there is a conflict with the same instrument type
                //std::cout << "true" << std::endl;
                //std::cout << a.c_info->tasks_in_conflict.size() << std::endl;
                a.c_info->tasks_in_conflict.push_back(i);
                //std::cout << "Pushed " << i << " to tasks_in_conflict!" << std::endl;
                int curr_size = g_assignment[i].size();
                for (int j=0; j < curr_size; j++){
                    a.c_info->agents_in_conflict.push_back(g_assignment[i][j]);
                    //std::cout << "Pushed " << g_assignment[i][j] << " to agents_in_conflict!" << std::endl;
                }
            }
        }
    }
}

void all_agent_conflict_info(std::vector<std::vector<int>> &g_assignment, std::vector<bool> &conflicts, std::vector<Agent> &all_a, std::vector<Task> &all_tasks){
    for (unsigned long int i=0; i < all_a.size(); i++){
        //std::cout << "Running for Agent " << i << std::endl;
        create_agent_conflict_info(g_assignment, conflicts, all_a[i], all_tasks);
    }
}

bool all_agents_assigned(std::vector<AgentNode> &all_a){
    for(unsigned long int i = 0; i < all_a.size(); i++){
        if(all_a[i].agent->assigned_task_position.x != all_a[i].agent->agent_position.x
          || all_a[i].agent->assigned_task_position.y != all_a[i].agent->agent_position.y
          || all_a[i].agent->assigned_task_position.z != all_a[i].agent->agent_position.z)
          {
            return false;
        }
    }
    return true;
}


////////////////////////////////////////////////
//determine distance traveled
double total_dist_traveled(std::vector<Agent> &all_ags){
    double total_dist=0;
    for (unsigned long int i=0; i < all_ags.size(); i++){
        total_dist+=all_ags[i].distance_traveled;
    }
    return total_dist;
}




}
