/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/disconnected.h"

namespace ns3{

NS_LOG_COMPONENT_DEFINE("DisconnectedSwarmSim");

double euclideanDistance(Vector taskPosition, Vector agentPosition){
    double xdiff, ydiff, zdiff, dist;
    xdiff = taskPosition.x - agentPosition.x;
    ydiff = taskPosition.y - agentPosition.y;
    zdiff = taskPosition.z - agentPosition.z;
    dist = sqrt((xdiff * xdiff) + (ydiff * ydiff) + (zdiff * zdiff));
    return dist;
}

Vector addNS3Vectors(Vector v1, Vector v2){
    Vector sumVector;
    sumVector.x = v1.x + v2.x;
    sumVector.y = v1.y + v2.y;
    sumVector.z = v1.z + v2.z;
    return sumVector;
}

void updatePosition(AgentNode* ag){
    Vector pos = getPosition(ag->node);
    pos.x = ag->agent->agentPosition.x;
    pos.y = ag->agent->agentPosition.y;
    pos.z = ag->agent->agentPosition.z;
    setPosition (ag->node, pos);
}

int getInstrumentType(int id){
    return id/globalInfo::agentsPerClass;
}

std::vector<Task> createTasks(){
    std::vector<Task> allTasks;
    allTasks.reserve(globalInfo::allTasks.size());
    double x, y, z;
    z = 0.0;

    for (unsigned long int i = 0; i < globalInfo::allTasks.size(); i++){
        allTasks[i].taskId = i;
        Ptr<UniformRandomVariable> randomStream;
        x = randomStream->GetInteger();
        y = randomStream->GetInteger();
        allTasks[i].updateLocation(x, y, z);
        allTasks[i].instrumentRequirement = getInstrumentType(i);
    }
    return allTasks;
}

bool** createWhoRequested(){
    bool** arr = new bool*[globalInfo::numAgents];
    for (int j=0; j < globalInfo::numAgents; j++){
        arr[j] = new bool[globalInfo::numAgents];
    }
    // Initialize with false
    for (int j=0; j < globalInfo::numAgents; j++){
        for (int i = 0; i < globalInfo::numAgents; i++){
            arr[j][i] = false;
        }
    }
    return arr;
}

std::vector<Agent*> createAgents(){
    std::vector<Agent*> allAgents;
    for (int i = 0; i < globalInfo::numAgents; i++){
        // Set received messages to null
        Agent* tempAgent = new Agent();
        // No messages sent
        tempAgent->numPositionMessagesSent = 0;
        tempAgent->numRequestMessagesSent = 0;
        tempAgent->numAgents = globalInfo::numAgents;
        // Other initialization
        tempAgent->agentId = i;
        tempAgent->distanceTraveled = 0; //initialize
        tempAgent->distanceLeft = 0; //set to actual value after assignment
        tempAgent->instrumentType = getInstrumentType(i);
        tempAgent->initializePartialAssignment();
        tempAgent->initializeInfoRequests();
        tempAgent->initializePreviousKnownPositions();
        tempAgent->initializeKnownPositions();
        tempAgent->initializeNeededInfo();
        tempAgent->initializePositionsToAsk();
        tempAgent->initializeKnownInfo();
        tempAgent->initializeLastTimeHeardFrom();
        tempAgent->initializeReceivedTimes();
        tempAgent->initializeSentPositions();
        tempAgent->initializeSentRequests();
        tempAgent->haveAllNeededInfo = false;
        tempAgent->initializeAssignedTaskPosition();
        tempAgent->initializePreviousAssignedTaskPosition();
        // Construct 2D whoRequested array
        tempAgent->whoRequested = createWhoRequested();
        allAgents.push_back(tempAgent);
        globalInfo::instrumentAssignment.at(i) = tempAgent->instrumentType;
    }
    return allAgents;
}

std::vector<std::vector<double>> createCostMatrix(std::vector<AgentNode> &allAgents){
    // Just copy data over into 2D vector for hungarian method
    std::vector<std::vector<double>> allCosts;
    for (unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        std::vector<double> oneRow;
        for (unsigned long int j = 0; j < globalInfo::allTasks.size(); j++){
            oneRow.push_back(allAgents[i].agent->taskCosts[j]);
        }
        allCosts.push_back(oneRow);
    }
    return allCosts;
}

void allBroadcastPositionIfChanged(std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        if (allAgents.at(i).agent->assignmentChanged()){
            // Broadcast position messages to neighbors
            for(unsigned long int j = 0; j < allAgents.size(); j++){
                if(j != i){
                    sendPositionInfo(allAgents[i], allAgents[j], i, interface);
                }
            }
        }
    }
}

double randomDouble(double fMin, double fMax){
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

Vector getPosition (Ptr<Node> node){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void setPosition (Ptr<Node> node, Vector position){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

void movePositions(Ptr<Node> node){
  Vector pos = getPosition(node);
  pos.x += 1;
  pos.y += 1;
  setPosition (node, pos);
}

void moveAllPositions(NodeContainer robots){
  for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
    movePositions(robots.Get(i));
  }
}

// Check if request has changed since it was last sent
bool compareBoolArr(bool* prev, bool* req){
    for (unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        if (prev[i]!=req[i]) return false;
    }
    return true;
}

// Determines if an agent has any position information that a neighbor requested
// Fills the position message buffer with messages to send
void determinePositionMessagesToSend(AgentNode &ag){
    for (int i = 0; i < ag.agent->numAgents; i++){
        for (int j = 0; j < ag.agent->numAgents; j++){
            // If we have the info and a neighbor requested it
            if ((ag.agent->knownInfo[j]) && (ag.agent->whoRequested[i][j])){
                ag.agent->whichPositionsToSend[j] = true;
                // Since we are going to send it to that particular agent
                // Mark their request as fulfilled
                ag.agent->whoRequested[i][j] = false;
                ag.agent->infoRequests[j] = false;  // Don't need to request info we have and have sent out
            }
        }
    }
}

void sendPositionMessagesInBuffer(int currentAgent, std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        if (allAgents[currentAgent].agent->whichPositionsToSend[i]){
            allAgents[currentAgent].agent->whichPositionsToSend[i] = false;
            // Broadcast position messages to neighbors
            for(unsigned long int j = 0; j < allAgents.size(); j++){
                if((int)j != currentAgent){
                    sendPositionInfo(allAgents[currentAgent], allAgents[j], i, interface);
                }
            }
        }
    }
}

void allDetermineAndSendPositionMessages(AgentNode &ag, std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface){
    for (int i = 0; i < ag.agent->numAgents; i++){
        for (int j = 0; j < ag.agent->numAgents; j++){
            // If we have the info and a neighbor requested it
            if ((ag.agent->knownInfo[j]) && (ag.agent->whoRequested[i][j])){
                // Since we are going to send it to that particular agent
                // Mark their request as fulfilled
                ag.agent->whoRequested[i][j] = false;
                ag.agent->infoRequests[j] = false;  // Don't need to request info we have and have sent out
                // Don't send if to itself
                if(ag.agent->agentId != i){
                    sendPositionInfo(allAgents[ag.agent->agentId], allAgents[i], j, interface);
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////
// Partial Assignment related functions

// Once information has been disseminated need to compute partial assignments
double computePartialAssignmentHungarian(Agent *ag, std::vector<TaskNode> &allTasks){
    std::vector<vector<double>> costMatrix;
    std::vector<int> taskLocations;  // Will store indices of tasks
    std::vector<int> agentsNeeded;  // Will store indices of agents

    int currentInstrument = ag->instrumentType;
    for (int i = 0; i < ag->numAgents; i++){
        if (allTasks[i].task->instrumentRequirement == currentInstrument){
            taskLocations.push_back(i);
        }
    }

    // Fill in agentsNeeded vector
    // Basically just add agents of the same instrument class
    // Determine if that information is known when filling in matrix
    for (int i = 0; i < ag->numAgents; i++){
        if (globalInfo::instrumentAssignment[i] == currentInstrument){
            agentsNeeded.push_back(i);
        }
    }
    
    // Will be numAgents_in_instrument_class x numTasks_for_class big (square)
    for (int i = 0; i < globalInfo::agentsPerClass; i++){
        std::vector<double> oneRow (globalInfo::agentsPerClass, INT_MAX);
        costMatrix.push_back(oneRow);
    }

    int numTasks = taskLocations.size();
    int numNeeded = agentsNeeded.size();
    //std::cout << numTasks << " " << numNeeded << std::endl;
    // Now fill in the smaller cost matrix using the big one
    // At this point all agents should have the information they need to compute the scores they need
    // To save time (rather than to recompute) just grab info from costMatrix (used for optimal)
    for (int i = 0; i < numNeeded; i++){
        int currentAgentId = agentsNeeded[i];
        if (ag->knownInfo[currentAgentId]){
            int currentTaskId;
            for (int j = 0; j < numTasks; j++){
                currentTaskId = taskLocations[j];

                costMatrix[i][j] = ag->localCostMatrix[currentAgentId][currentTaskId];
            }
        }
    }
    // Check to see that matrix is filled in correctly
    //printSmallCostMatrix(costMatrix);

    // Now use the smaller matrix to compute an assignment using hungarian method
    HungarianAlgorithm partialHungarian;
    std::vector<int> unmappedPartialAssignment;
    double partialCost = partialHungarian.Solve(costMatrix, unmappedPartialAssignment);

    // Now map the partial assignment back to the actual
    for (int i = 0; i < numNeeded; i++){
        int currentAgent, currentTask;
        currentAgent = agentsNeeded[i];
        currentTask = taskLocations[unmappedPartialAssignment[i]];
        if (ag->knownInfo[currentAgent]){
            ag->partialAssignment[currentAgent] = currentTask;
        }
    }

    return partialCost;
}

//////////////////////////////////////////////////////////////////////////
// Movement related functions
double vectorMagnitude(double x, double y, double z){
    double magnitude = (x * x) + (y * y) + (z * z);
    magnitude=sqrt(magnitude);
    return magnitude;
}

Vector differenceVector(Vector &agentPosition, Vector &taskPosition){
    double xdiff, ydiff, zdiff;
    xdiff = taskPosition.x-agentPosition.x;
    ydiff = taskPosition.y-agentPosition.y;
    zdiff = taskPosition.z-agentPosition.z;
    return Vector(xdiff,ydiff,zdiff);
}

Vector createMovementVector(Vector &agentPosition, Vector &taskPosition){
    Vector diff = differenceVector(agentPosition, taskPosition);

    double magnitude = vectorMagnitude(diff.x, diff.y, diff.z);
    if(magnitude < 0.01){  // If on position
      diff.x = 0;
      diff.y = 0;
      diff.z = 0;
    }
    else{
      diff.x = diff.x/magnitude;
      diff.y = diff.y/magnitude;
      diff.z = diff.z/magnitude;
    }

    return diff;
}

void moveAgentTowardsGoalStep(AgentNode ag){
    Vector currentTaskPosition = ag.agent->agentPosition;

    double movementX = ag.agent->movementVector.x * ag.agent->speed;
    double movementY = ag.agent->movementVector.y * ag.agent->speed;
    double movementZ = ag.agent->movementVector.z * ag.agent->speed;

    // X position
    if(abs(currentTaskPosition.x - ag.agent->assignedTaskPosition.x) <= abs(movementX)){
      currentTaskPosition.x = ag.agent->assignedTaskPosition.x;
    }
    else{
      currentTaskPosition.x += movementX;
    }
    // Y position
    if(abs(currentTaskPosition.y - ag.agent->assignedTaskPosition.y) <= abs(movementY)){
      currentTaskPosition.y = ag.agent->assignedTaskPosition.y;
    }
    else{
      currentTaskPosition.y += movementY;
    }
    // Z positions
    if(abs(currentTaskPosition.z - ag.agent->assignedTaskPosition.z) <= abs(movementZ)){
      currentTaskPosition.z += ag.agent->assignedTaskPosition.z;
    }
    else{
      currentTaskPosition.z += movementZ;
    }

    int stepDistance = sqrt(movementX*movementX + movementY*movementY + movementZ*movementZ);
    ag.agent->distanceTraveled += stepDistance;
    ag.agent->agentPosition = currentTaskPosition;
    ag.agent->knownPositions[ag.agent->agentId] = currentTaskPosition;

    updatePosition(&ag);
}

////////////////////////////////////////////////
// Checking if done

bool conflictsExist(std::vector<AgentNode> &allAgents){
    std::vector<int> allPoints;
    bool conflictsExist = false;
    for(unsigned long int i = 0; i < allAgents.size(); i++){
        for(unsigned long int j = 0; j < allAgents.size(); j++){
              if(allAgents[i].agent->agentPosition.x == allAgents[j].agent->agentPosition.x
                && allAgents[i].agent->agentPosition.y == allAgents[j].agent->agentPosition.y
                && allAgents[i].agent->agentPosition.z == allAgents[j].agent->agentPosition.z
                && i != j){
                  std::cout << "Conflict at " << i << " with " << j << " at "
                      << "(" << allAgents[j].agent->agentPosition.x << "," << allAgents[j].agent->agentPosition.y
                      << "," << allAgents[j].agent->agentPosition.z << ") "<< std::endl;
                  conflictsExist = true;
              }
        }
    }
    return conflictsExist;
}

bool allAgentsAssigned(std::vector<AgentNode> &allAgents){
    for(unsigned long int i = 0; i < allAgents.size(); i++){
        if(allAgents[i].agent->assignedTaskPosition.x != allAgents[i].agent->agentPosition.x
          || allAgents[i].agent->assignedTaskPosition.y != allAgents[i].agent->agentPosition.y
          || allAgents[i].agent->assignedTaskPosition.z != allAgents[i].agent->agentPosition.z)
          {
            return false;
        }
    }
    return true;
}

void checkIfDone(std::vector<AgentNode> &allAgents){
    if(allAgentsAssigned(allAgents)){
        if(conflictsExist(allAgents)) std::cout << "Conflicts Exist" << std::endl;
        else std::cout << "SUCCESSFUL: No conflicts, all agents are assigned" << std::endl;
        // Print result info
        double distanceTraveled = totalDistanceTraveled(allAgents);
        int numRequestMessages = totalNumRequestMessagesSent(allAgents);
        int numPositionMessages = totalNumPositionMessagesSent(allAgents);
        std::cout << "Total distance moved: " << distanceTraveled << std::endl;
        std::cout << "Total request messages: " << numRequestMessages << std::endl;
        std::cout << "Total position messsages: " << numPositionMessages << std::endl;
        std::cout << "Total messsages: " << numRequestMessages + numPositionMessages << std::endl;
        std::cout << "Percentage received: " << 100.0 * (1.0 * globalInfo::totalMessagesReceived) / (1.0*numRequestMessages + 1.0*numPositionMessages) << "%" << std::endl;
        std::cout << "Number of moves: " << globalInfo::numMoves << std::endl;
        exit(0);
        return;
    }
}

////////////////////////////////////////////////
// Find totals
double totalDistanceTraveled(std::vector<AgentNode> &allAgents){
    double totalDistance = 0;
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        totalDistance += allAgents[i].agent->distanceTraveled;
    }
    return totalDistance;
}

int totalNumPositionMessagesSent(std::vector<AgentNode> &allAgents){
    int totalNumPositionMessages = 0;
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        totalNumPositionMessages += allAgents[i].agent->numPositionMessagesSent;
    }
    return totalNumPositionMessages;
}

int totalNumRequestMessagesSent(std::vector<AgentNode> &allAgents){
    int totalNumRequestMessages = 0;
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        totalNumRequestMessages += allAgents[i].agent->numRequestMessagesSent;
    }
    return totalNumRequestMessages;
}

} //end namespace ns3
