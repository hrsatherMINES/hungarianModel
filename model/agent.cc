#include "agent.h"

using namespace ns3;

// Original heuristic. just ask if same type
void Agent::determine_needed_info_original(){
    int type=instrument_type;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrument_assignment[i]==type && i != agent_id){
            needed_info[i]=true;
        }
        else{
            needed_info[i]=false;
        }
    }
}

// Only ask info from another node if node is not assigned
void Agent::determine_needed_info_self_not_assigned(){
    int type=instrument_type;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrument_assignment[i]==type && i != agent_id && !isAssigned()){
            needed_info[i]=true;
        }
        else{
            needed_info[i]=false;
        }
    }
}

// Only ask info from another node if you know its still moving
void Agent::determine_needed_info_still_moving(){
    int type=instrument_type;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrument_assignment[i]==type && i != agent_id && !agentAssigned(i)){
            needed_info[i]=true;
        }
        else{
            needed_info[i]=false;
        }
    }
}

// Only ask info from another node if node is not assigned AND itself is still moving
void Agent::determine_needed_info_both_moving(){
    int type=instrument_type;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrument_assignment[i]==type && i != agent_id && !isAssigned() && !agentAssigned(i)){
            needed_info[i]=true;
        }
        else{
            needed_info[i]=false;
        }
    }
}

// Only ask if node is close to itself
void Agent::determine_needed_info_distance(){
    int type=instrument_type;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrument_assignment[i]==type && i != agent_id && !isAssigned() && !agentAssigned(i)){
            needed_info[i]=true;
        }
        else{
            needed_info[i]=false;
        }
    }
}

bool Agent::isAssigned(){
    // Assume moving if any vector is above half its speed
    return abs(agent_position.x - assigned_task_position.x) < speed/2
            && abs(agent_position.y - assigned_task_position.y) < speed/2
            && abs(agent_position.z - assigned_task_position.z) < speed/2;
}

bool Agent::isClose(int which_agent){
    // If don't know info, assume close
}

bool Agent::agentAssigned(int which_agent){
    for(int i = 0; i < numTasks; i++){
        Vector tempTaskLocation = globalInfo::allTasks.at(i).task->task_location;
        // If the movement is less than half its speed, assume it did not move
        if(abs(tempTaskLocation.x - known_positions[which_agent].x) < speed/2
                && abs(tempTaskLocation.y - known_positions[which_agent].y) < speed/2
                && abs(tempTaskLocation.z - known_positions[which_agent].z) < speed/2){
            return true;
        }
    }
    return false;
}

void Agent::fillLocalCostMatrix(){
    vector<vector<double>> tempCostMatrix;
    for(int i = 0; i < numTasks; i++){
        vector<double> agentCosts;
        for(int j = 0; j < numTasks; j++){
            agentCosts.push_back(euc_dist(known_positions[i], globalInfo::allTasks.at(j).task->task_location));
        }
        tempCostMatrix.push_back(agentCosts);
    }
    localCostMatrix = tempCostMatrix;
}

void Agent::print_position(){
    std::cout << "Agent #" << agent_id << ": Position (" << agent_position.x << "," << agent_position.y << "," << agent_position.z << ") ";
}

void Agent::print_assigned_position(){
    std::cout << " Assigned Position: (" << assigned_task_position.x << "," << assigned_task_position.y << "," << assigned_task_position.z << ") " << std::endl;
}

void Agent::initialize_cinfo(){
    conflict_info *temp = new conflict_info;
    c_info =temp;
}

void Agent::print_cinfo(){
    int a_size=c_info->agents_in_conflict.size();
    int t_size=c_info->tasks_in_conflict.size();
    if ((a_size == 0) && (t_size==0)){
        std::cout << "No conflict for agent " << agent_id << std::endl;
        return;
    }
    else{
        std::cout << "Conflicts for agent " << agent_id << std::endl;
    }
    std::cout << "Agents in conflict: ";
    for (int i=0; i < a_size; i++){
        std::cout << c_info->agents_in_conflict[i] << " ";
    }
    std::cout << "\nTasks in conflict: ";
    for (int j=0; j < t_size; j++){
        std::cout << c_info->agents_in_conflict[j] << " ";
    }
}

void Agent::update_task_position(double x, double y, double z){
    assigned_task_position = Vector(x,y,z);
}

void Agent::fill_in_agent_costs(std::vector<TaskNode> &all_ts){
    task_costs= new double [numTasks];
    for (int i=0; i < numTasks; i++){
        if (instrument_type==all_ts[i].task->instrument_requirement){
            task_costs[i]=euc_dist(all_ts[i].task->task_location, agent_position);
        }
        else task_costs[i]=INT_MAX;
    }
}

void Agent::print_agent_costs(){
    std::cout << "costs:" << std::endl;
    for (int i=0; i < numTasks; i++){
        if (task_costs[i]==INT_MAX){
            std::cout << "x ";
        }
        else std::cout << task_costs[i] << " ";
    }
    std::cout << std::endl;
}

void Agent::set_speed(double speedIn){
    speed = speedIn;
}

void Agent::set_num_agents(int numAgentsIn){
    numAgents = numAgentsIn;
}

void Agent::set_num_tasks(int numTasksIn){
    numTasks = numTasksIn;
}

void Agent::initialize_received_times(){
    received_times = new unsigned long int [numAgents];
    for (int i=0; i < numAgents; i++){
        received_times[i]=0;
    }
}

void Agent::initialize_needed_info(){
    needed_info = new bool [numAgents];
    for (int i=0; i < numAgents; i++){
        received_times[i]=false;
    }
}

void Agent::initialize_sent_times(){
    sent_times = new unsigned long*[globalInfo::numAgents];
    for (int j=0; j < globalInfo::numAgents; j++){
        sent_times[j] = new unsigned long[globalInfo::numAgents];
    }
    // Initialize with false
    for (int j=0; j < globalInfo::numAgents; j++){
        for (int i=0; i < globalInfo::numAgents; i++){
            sent_times[j][i] = 0;
        }
    }
}

void Agent::initialize_info_requests(){
    info_requests = new bool [numAgents];
    previous_sent_request = new bool [numAgents];
    memset(info_requests, false, numAgents);
}

void Agent::initialize_known_positions(){
    known_positions = new Vector [numAgents];
    known_positions[agent_id]=agent_position;
}

// Also initialize message buffer
void Agent::initialize_known_info(){
    known_info = new bool [numAgents];
    memset(known_info, false, numAgents);
    which_positions_to_send = new bool [numAgents];
    memset(which_positions_to_send, false, numAgents);
    known_info[agent_id]=true;
}

void Agent::initialize_partial_assignment(){
    //std::cout << "Initalizing partial assignment" << endl;
    partial_assignment = new int [numAgents];
    for (int i=0; i < numAgents; i++){
        partial_assignment[i]=-1;
    }
}

void Agent::print_partial_assignment(){
    std::cout << "Agent #" << agent_id << "\tPartial Assignment: ";
    for (int i=0; i < numAgents; i++){
        std::cout << partial_assignment[i] << " ";
    }
    std::cout << "\n";
}

void Agent::print_known_info(){
    std::cout << "Agent #" << agent_id << "\tKnown Info:\t";
    for (int i=0; i < numAgents; i++){
        std::cout << known_info[i] << " ";
    }
    std::cout << "\n";
}

void Agent::print_needed_info(){
    std::cout << "Agent #" << agent_id << "\tNeeded Info:\t";
    for (int i=0; i < numAgents; i++){
        std::cout << needed_info[i] << " ";
    }
    std::cout << "\n";
}

void Agent::print_assigned_task_pos(){
    std::cout << "Agent #" << agent_id << "\tAssigned Task Position:\t";
    std::cout << "("<< assigned_task_position.x << "," << assigned_task_position.y << "," << assigned_task_position.z << ")";
    std::cout << "\n";
}

void Agent::move_agent(){
    // double xnew, ynew, znew;
    // xnew=agent_position.x+movement_vec.x;
    // ynew=agent_position.y+movement_vec.y;
    // znew=agent_position.z+movement_vec.z;
    // //distance_traveled++;
    // update_position(xnew, ynew, znew);
}

send_request Agent::create_send_request(){
    send_request to_send;
    to_send.sender_id=agent_id;
    bool* request_to_send = new bool[numAgents];
    // Copy request info over
    for (int i=0; i < numAgents; i++){
        request_to_send[i]=info_requests[i];
    }
    to_send.request=request_to_send;
    to_send.num_hops=0;
    return to_send;
}

send_position Agent::create_send_position(int which_agent){
    send_position to_send;
    to_send.info_id=which_agent;
    to_send.sender_id=agent_id;
    to_send.position=known_positions[which_agent];
    if(which_agent == agent_id){
        std::chrono::milliseconds ms =
            std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
        to_send.time_sent = ms.count();
        to_send.position=agent_position;
    }
    else{
        to_send.time_sent=received_times[which_agent];
    }
    
    return to_send;
}