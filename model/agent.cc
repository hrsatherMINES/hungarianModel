#include "agent.h"

using namespace ns3;

// Original heuristic. just ask if same type
void Agent::determineNeededInfoOriginal(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask info from another node if node is not assigned
void Agent::determineNeededInfoSelfNotAssigned(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && !isAssigned()){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask info from another node if you know its still moving
void Agent::determineNeededInfoStillMoving(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && !agentAssigned(i)){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask info from another node if node is not assigned AND itself is still moving
void Agent::determineNeededInfoBothMoving(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && !isAssigned()
                                    && !agentAssigned(i)){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask if node is close to itself
void Agent::determineNeededInfoDistance(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && isClose(i)){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask if node is close to itself and consider if moving
void Agent::determineNeededInfoDistanceMoving(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && isClose(i)
                                    && !isAssigned()
                                    && !agentAssigned(i)){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask if node is moving toward task that we are moving to
void Agent::determineNeededInfoInferTask(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && movingTowardSameTask(i)){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask if node is moving toward task that we are moving to. Don't ask if not moving
void Agent::determineNeededInfoInferTaskAndMoving(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && isClose(i)
                                    && !isAssigned()
                                    && !agentAssigned(i)
                                    && movingTowardSameTask(i)){
            neededInfo[i] = true;
            positionsToAsk[i] = true;
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

// Only ask if node is moving toward task that we are moving to. Don't ask if not moving
void Agent::determineNeededInfoOneHop(){
    for (int i = 0; i < numAgents; i++){
        if(globalInfo::instrumentAssignment[i] == instrumentType
                                    && i != agentId
                                    && !isAssigned()
                                    && !agentAssigned(i)){
            positionsToAsk[i] = true;
            if(agentIsOneHop(i)){
                neededInfo[i] = false;
            }
            else{
                neededInfo[i] = true;
            }
        }
        else{
            neededInfo[i] = false;
            positionsToAsk[i] = false;
        }
    }
}

bool Agent::movingTowardSameTask(int whichAgent){
    // If we don't know where an agent is, ask for its location
    if((previousKnownPositions[whichAgent].x == 0.0)
            && (previousKnownPositions[whichAgent].y == 0.0)
            && (previousKnownPositions[whichAgent].z == 0.0)){
        // The longer that we go without getting info, the less likely we should
        // ask for position because we can assume it is too far away
        int randNum = rand() % 100;
        if(randNum < 100 - 2 * globalInfo::numMoves){
            return true;
        }
        return false;
    }
    // Find which task the agent is moving to. Might be able to be done with Hungarian Method as well
    double largestMoveToTask = INT_MIN;
    int taskMovingTo = 0;
    for(int i = 0; i < numTasks; i++){
        Vector previousPosition = previousKnownPositions[whichAgent];
        double previousDistanceToTask = euclideanDistance(previousPosition, globalInfo::allTasks[i].task->taskLocation);
        Vector newPosition = knownPositions[whichAgent];
        double newDistanceToTask = euclideanDistance(newPosition, globalInfo::allTasks[i].task->taskLocation);
        double changeInDistance = previousDistanceToTask - newDistanceToTask;
        if(changeInDistance > largestMoveToTask){
            largestMoveToTask = changeInDistance;
            taskMovingTo = i;
        }
    }
    if(abs(globalInfo::allTasks[taskMovingTo].task->taskLocation.x - assignedTaskPosition.x) < speed
                && abs(globalInfo::allTasks[taskMovingTo].task->taskLocation.y - assignedTaskPosition.y) < speed
                && abs(globalInfo::allTasks[taskMovingTo].task->taskLocation.z - assignedTaskPosition.z) < speed){
        return true;
    }
    // If not moving towards task, ask stochastically
    int randNum = rand() % 100;
    if(randNum < globalInfo::probabilityDropped){
        return true;
    }
    return false;
}

bool Agent::isAssigned(){
    // Assume moving if any vector is above half its speed
    return abs(agentPosition.x - assignedTaskPosition.x) < speed/2
            && abs(agentPosition.y - assignedTaskPosition.y) < speed/2
            && abs(agentPosition.z - assignedTaskPosition.z) < speed/2;
}

bool Agent::isClose(int whichAgent){
    // If we don't know the position, ask for it for sure
    if((knownPositions[whichAgent].x == 0.0)
            && (knownPositions[whichAgent].y == 0.0)
            && (knownPositions[whichAgent].z == 0.0)){
        // The longer that we go without getting info, the less likely we should
        // ask for position because we can assume it is too far away
        int randNum = rand() % 100;
        if(randNum < 100 - 2 * globalInfo::numMoves){
            return true;
        }
        return false;
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
    for(int i = 0; i < numAgents; i++){
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

bool Agent::assignmentChanged(){
    if(((previousAssignedTaskPosition.x != assignedTaskPosition.x)
                || (previousAssignedTaskPosition.y != assignedTaskPosition.y)
                || (previousAssignedTaskPosition.z != assignedTaskPosition.z))
            &&((previousAssignedTaskPosition.x != 0.0)
                && (previousAssignedTaskPosition.y != 0.0))){
        return true;
    }
    return false;
}

void Agent::fillInAgentCosts(std::vector<TaskNode> &allTasks){
    taskCosts = new double [numTasks];
    for (int i = 0; i < numTasks; i++){
        if (instrumentType == allTasks[i].task->instrumentRequirement){
            taskCosts[i] = euclideanDistance(allTasks[i].task->taskLocation, agentPosition);
        }
        else taskCosts[i] = INT_MAX;
    }
}

void Agent::printAgentCosts(){
    std::cout << "costs:" << std::endl;
    for (int i = 0; i < numTasks; i++){
        if (taskCosts[i] == INT_MAX){
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

void Agent::initializeLastTimeHeardFrom(){
    lastTimeHeardFrom = new unsigned long int [numAgents];
    for (int i = 0; i < numAgents; i++){
        lastTimeHeardFrom[i] = 0;
    }
}

bool Agent::agentIsOneHop(int whichAgent){
    std::chrono::milliseconds ms =
            std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
    long unsigned int currentMillis = ms.count();
    // If we have no info, assume true 50%
    if(lastTimeHeardFrom[whichAgent] == 0){
        // int randNum = rand() % 100;
        // if(randNum < globalInfo::probabilityDropped){
        //     return true;
        // }
        return false;
    }
    // Assume one hop if heard from agent less than 5 seconds ago
    if(currentMillis - lastTimeHeardFrom[whichAgent] < 5000){
        return true;
    }
    return false;
}

void Agent::initializeReceivedTimes(){
    receivedTimes = new unsigned long int [numAgents];
    for (int i = 0; i < numAgents; i++){
        receivedTimes[i] = 0;
    }
}

void Agent::initializeNeededInfo(){
    neededInfo = new bool [globalInfo::numAgents];
    for (int i = 0; i < globalInfo::numAgents; i++){
        neededInfo[i] = false;
    }
}

void Agent::initializePositionsToAsk(){
    positionsToAsk = new bool [globalInfo::numAgents];
    for (int i = 0; i < globalInfo::numAgents; i++){
        positionsToAsk[i] = false;
    }
}

void Agent::initializeSentPositions(){
    sentPositions = new double*[globalInfo::numAgents];
    for (int j = 0; j < globalInfo::numAgents; j++){
        sentPositions[j] = new double[globalInfo::numAgents];
    }
    // Initialize with false
    for (int j = 0; j < globalInfo::numAgents; j++){
        for (int i = 0; i < globalInfo::numAgents; i++){
            sentPositions[j][i] = 0.0;
        }
    }
}

void Agent::initializeSentRequests(){
    sentRequests = new long int*[globalInfo::numAgents];
    for (int j = 0; j < globalInfo::numAgents; j++){
        sentRequests[j] = new long int[globalInfo::numAgents];
    }
    // Initialize with false
    for (int j = 0; j < globalInfo::numAgents; j++){
        for (int i = 0; i < globalInfo::numAgents; i++){
            sentRequests[j][i] = 0;
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
    for(int i = 0; i < numAgents; i++){
        knownPositions[i] = Vector(0.0, 0.0, 0.0);
    }
    knownPositions[agentId] = agentPosition;
}

void Agent::initializePreviousKnownPositions(){
    previousKnownPositions = new Vector [numAgents];
    for(int i = 0; i < numAgents; i++){
        previousKnownPositions[i] = Vector(0.0, 0.0, 0.0);
    }
    previousKnownPositions[agentId] = agentPosition;
}

void Agent::initializeAssignedTaskPosition(){
    assignedTaskPosition = Vector(0.0, 0.0, 0.0);
}

void Agent::initializePreviousAssignedTaskPosition(){
    previousAssignedTaskPosition = Vector(0.0, 0.0, 0.0);
}

// Also initialize message buffer
void Agent::initializeKnownInfo(){
    knownInfo = new bool [numAgents];
    memset(knownInfo, false, numAgents);
    whichPositionsToSend = new bool [numAgents];
    memset(whichPositionsToSend, false, numAgents);
    knownInfo[agentId] = true;
}

void Agent::initializePartialAssignment(){
    //std::cout << "Initalizing partial assignment" << endl;
    partialAssignment = new int [numAgents];
    for (int i = 0; i < numAgents; i++){
        partialAssignment[i] = -1;
    }
}

void Agent::printPartialAssignment(){
    std::cout << "Agent #" << agentId << "\tPartial Assignment: ";
    for (int i = 0; i < numAgents; i++){
        std::cout << partialAssignment[i] << " ";
    }
    std::cout << "\n";
}

void Agent::printKnownInfo(){
    std::cout << "Agent #" << agentId << "\tKnown Info:\t";
    for (int i = 0; i < numAgents; i++){
        std::cout << knownInfo[i] << " ";
    }
    std::cout << "\n";
}

void Agent::printNeededInfo(){
    std::cout << "Agent #" << agentId << "\tNeeded Info:\t";
    for (int i = 0; i < numAgents; i++){
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
    toSend.senderId = agentId;
    bool* requestToSend = new bool[numAgents];
    // Copy request info over
    for (int i = 0; i < numAgents; i++){
        requestToSend[i] = infoRequests[i];
    }
    toSend.request = requestToSend;
    toSend.numHops = 0;
    return toSend;
}

sendPosition Agent::createSendPosition(int whichAgent){
    sendPosition toSend;
    toSend.infoId = whichAgent;
    toSend.senderId = agentId;
    toSend.position = knownPositions[whichAgent];
    if(whichAgent == agentId){
        std::chrono::milliseconds ms =
            std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
        toSend.timeSent = ms.count();
        toSend.position = agentPosition;
    }
    else{
        toSend.timeSent = receivedTimes[whichAgent];
    }
    
    return toSend;
}