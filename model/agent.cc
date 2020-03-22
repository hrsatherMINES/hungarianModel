#include "agent.h"

using namespace ns3;

// Original heuristic. just ask if same type
void Agent::determineNeededInfoOriginal(){
    int type=instrumentType;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == type
                                    && i != agentId){
            neededInfo[i]=true;
        }
        else{
            neededInfo[i]=false;
        }
    }
}

// Only ask info from another node if node is not assigned
void Agent::determineNeededInfoSelfNotAssigned(){
    int type=instrumentType;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == type
                                    && i != agentId
                                    && !isAssigned()){
            neededInfo[i]=true;
        }
        else{
            neededInfo[i]=false;
        }
    }
}

// Only ask info from another node if you know its still moving
void Agent::determineNeededInfoStillMoving(){
    int type=instrumentType;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == type
                                    && i != agentId
                                    && !agentAssigned(i)){
            neededInfo[i]=true;
        }
        else{
            neededInfo[i]=false;
        }
    }
}

// Only ask info from another node if node is not assigned AND itself is still moving
void Agent::determineNeededInfoBothMoving(){
    int type=instrumentType;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == type
                                    && i != agentId
                                    && !isAssigned()
                                    && !agentAssigned(i)){
            neededInfo[i]=true;
        }
        else{
            neededInfo[i]=false;
        }
    }
}

// Only ask if node is close to itself
void Agent::determineNeededInfoDistance(){
    int type=instrumentType;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == type
                                    && i != agentId
                                    && isClose(i)){
            neededInfo[i]=true;
        }
        else{
            neededInfo[i]=false;
        }
    }
}

// Only ask if node is close to itself and consider if moving
void Agent::determineNeededInfoDistanceMoving(){
    int type=instrumentType;
    for (int i=0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == type
                                    && i != agentId
                                    && isClose(i)
                                    && !isAssigned()
                                    && !agentAssigned(i)){
            neededInfo[i]=true;
        }
        else{
            neededInfo[i]=false;
        }
    }
}

bool Agent::isAssigned(){
    // Assume moving if any vector is above half its speed
    return abs(agentPosition.x - assignedTaskPosition.x) < speed/2
            && abs(agentPosition.y - assignedTaskPosition.y) < speed/2
            && abs(agentPosition.z - assignedTaskPosition.z) < speed/2;
}

bool Agent::isClose(int whichAgent){
    // If we don't know the position, ask for it for sure. TODO: Only ask a few times
    if((knownPositions[whichAgent].x == 0.0) 
                        && (knownPositions[whichAgent].y == 0.0) 
                        && (knownPositions[whichAgent].z == 0.0)){
        return true;
    }
    // Else, calculate probability that we ask for position based off how close the last reading was.
    // the closer, the more likely we are to ask for it
    double randomPercent = randomDouble(0.0, 1.0);
    double distanceBetweenAgents = euclideanDistance(agentPosition, knownPositions[whichAgent]);
    double percentLikelyToSend = 1 - (distanceBetweenAgents / globalInfo::maxPositionDistance);
    if(randomPercent < percentLikelyToSend) return true;
    return false;
}

bool Agent::agentAssigned(int whichAgent){
    for(int i = 0; i < numTasks; i++){
        Vector tempTaskLocation = globalInfo::allTasks.at(i).task->taskLocation;
        // If the movement is less than half its speed, assume it did not move
        if(abs(tempTaskLocation.x - knownPositions[whichAgent].x) < speed/2
                && abs(tempTaskLocation.y - knownPositions[whichAgent].y) < speed/2
                && abs(tempTaskLocation.z - knownPositions[whichAgent].z) < speed/2){
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
            agentCosts.push_back(euclideanDistance(knownPositions[i], globalInfo::allTasks.at(j).task->taskLocation));
        }
        tempCostMatrix.push_back(agentCosts);
    }
    localCostMatrix = tempCostMatrix;
}

void Agent::printPosition(){
    std::cout << "Agent #" << agentId << ": Position (" << agentPosition.x << "," << agentPosition.y << "," << agentPosition.z << ") ";
}

void Agent::printAssignedPosition(){
    std::cout << " Assigned Position: (" << assignedTaskPosition.x << "," << assignedTaskPosition.y << "," << assignedTaskPosition.z << ") " << std::endl;
}

void Agent::updateTaskPosition(double x, double y, double z){
    assignedTaskPosition = Vector(x,y,z);
}

void Agent::fillInAgentCosts(std::vector<TaskNode> &allTasks){
    taskCosts= new double [numTasks];
    for (int i=0; i < numTasks; i++){
        if (instrumentType==allTasks[i].task->instrumentRequirement){
            taskCosts[i]=euclideanDistance(allTasks[i].task->taskLocation, agentPosition);
        }
        else taskCosts[i]=INT_MAX;
    }
}

void Agent::printAgentCosts(){
    std::cout << "costs:" << std::endl;
    for (int i=0; i < numTasks; i++){
        if (taskCosts[i]==INT_MAX){
            std::cout << "x ";
        }
        else std::cout << taskCosts[i] << " ";
    }
    std::cout << std::endl;
}

void Agent::setSpeed(double speedIn){
    speed = speedIn;
}

void Agent::setNumAgents(int numAgentsIn){
    numAgents = numAgentsIn;
}

void Agent::setNumTasks(int numTasksIn){
    numTasks = numTasksIn;
}

void Agent::initializeReceivedTimes(){
    receivedTimes = new unsigned long int [numAgents];
    for (int i=0; i < numAgents; i++){
        receivedTimes[i]=0;
    }
}

void Agent::initializeNeededInfo(){
    neededInfo = new bool [globalInfo::numAgents];
    for (int i=0; i < globalInfo::numAgents; i++){
        neededInfo[i]=false;
    }
}

void Agent::initializeSentTimes(){
    sentTimes = new unsigned long*[globalInfo::numAgents];
    for (int j=0; j < globalInfo::numAgents; j++){
        sentTimes[j] = new unsigned long[globalInfo::numAgents];
    }
    // Initialize with false
    for (int j=0; j < globalInfo::numAgents; j++){
        for (int i=0; i < globalInfo::numAgents; i++){
            sentTimes[j][i] = 0;
        }
    }
}

void Agent::initializeInfoRequests(){
    infoRequests = new bool [numAgents];
    previousSentRequest = new bool [numAgents];
    memset(infoRequests, false, numAgents);
}

void Agent::initializeKnownPositions(){
    knownPositions = new Vector [numAgents];
    knownPositions[agentId]=agentPosition;
}

// Also initialize message buffer
void Agent::initializeKnownInfo(){
    knownInfo = new bool [numAgents];
    memset(knownInfo, false, numAgents);
    whichPositionsToSend = new bool [numAgents];
    memset(whichPositionsToSend, false, numAgents);
    knownInfo[agentId]=true;
}

void Agent::initializePartialAssignment(){
    //std::cout << "Initalizing partial assignment" << endl;
    partialAssignment = new int [numAgents];
    for (int i=0; i < numAgents; i++){
        partialAssignment[i]=-1;
    }
}

void Agent::printPartialAssignment(){
    std::cout << "Agent #" << agentId << "\tPartial Assignment: ";
    for (int i=0; i < numAgents; i++){
        std::cout << partialAssignment[i] << " ";
    }
    std::cout << "\n";
}

void Agent::printKnownInfo(){
    std::cout << "Agent #" << agentId << "\tKnown Info:\t";
    for (int i=0; i < numAgents; i++){
        std::cout << knownInfo[i] << " ";
    }
    std::cout << "\n";
}

void Agent::printNeededInfo(){
    std::cout << "Agent #" << agentId << "\tNeeded Info:\t";
    for (int i=0; i < numAgents; i++){
        std::cout << neededInfo[i] << " ";
    }
    std::cout << "\n";
}

void Agent::printAssignedTaskPos(){
    std::cout << "Agent #" << agentId << "\tAssigned Task Position:\t";
    std::cout << "("<< assignedTaskPosition.x << "," << assignedTaskPosition.y << "," << assignedTaskPosition.z << ")";
    std::cout << "\n";
}

sendRequest Agent::createSendRequest(){
    sendRequest toSend;
    toSend.senderId=agentId;
    bool* requestToSend = new bool[numAgents];
    // Copy request info over
    for (int i=0; i < numAgents; i++){
        requestToSend[i]=infoRequests[i];
    }
    toSend.request=requestToSend;
    toSend.numHops=0;
    return toSend;
}

sendPosition Agent::createSendPosition(int whichAgent){
    sendPosition toSend;
    toSend.infoId=whichAgent;
    toSend.senderId=agentId;
    toSend.position=knownPositions[whichAgent];
    if(whichAgent == agentId){
        std::chrono::milliseconds ms =
            std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
        toSend.timeSent = ms.count();
        toSend.position=agentPosition;
    }
    else{
        toSend.timeSent=receivedTimes[whichAgent];
    }
    
    return toSend;
}