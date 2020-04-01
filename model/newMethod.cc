#include "newMethod.h"

namespace ns3 {
// Different heuristics for determining which info is needed
void determineAllNeededInfoOriginal(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoOriginal();
    }
}

void determineAllNeededInfoStillMoving(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoStillMoving();
    }
}

void determineAllNeededInfoSelfMoving(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoSelfNotAssigned();
    }
}

void determineAllNeededInfoBothMoving(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoBothMoving();
    }
}

void determineAllNeededInfoDistance(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoDistance();
    }
}

void determineAllNeededInfoDistanceMoving(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoDistanceMoving();
    }
}

void determineAllNeededInfoInferTask(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoInferTask();
    }
}

void determineAllNeededInfoInferTaskAndMoving(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoInferTaskAndMoving();
    }
}

void determineAllNeededInfoOneHop(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        allAgents[i].agent->determineNeededInfoOneHop();
    }
}

// End of Heuristics
void calculateAllCosts(std::vector<TaskNode> &allTasks, std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        allAgents[i].agent->fillInAgentCosts(allTasks);
    }
}

void fillAllLocalCosts(std::vector<AgentNode> &allAgents){
    for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        allAgents.at(i).agent->fillLocalCostMatrix();
    }
}

void calculateCostsAndPrepareRequests(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface){
    globalInfo::numMoves++;

    // Process costs
    calculateAllCosts(allTasks, allAgents);
    fillAllLocalCosts(allAgents);
    
    // Different heuristics for requests CHANGE HERE ***
    determineAllNeededInfoOneHop(allAgents);
    // CHANGE HERE *************************************

    // Prepare requests
    initializeAllRequests(allAgents);
    // Add own request to request list
    addOwnRequestToRequestList(allAgents);

    // Schedule request messages
    Simulator::Schedule(Seconds(0.5), &allSendRequestsScheduled, allAgents, allTasks, interface);
}

// Fill in initial request array with needed info (more will be added once shared)
// Also set up previous sent request
void initializeAllRequests(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        for (unsigned long int j = 0; j < allAgents.size(); j++){
            allAgents[i].agent->infoRequests[j] = allAgents[i].agent->neededInfo[j];
            allAgents[i].agent->previousSentRequest[j] = allAgents[i].agent->neededInfo[j];
        }
    }
}

void addOwnRequestToRequestList(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        sendRequest newRequest = allAgents[i].agent->createSendRequest();
        allAgents[i].agent->receivedRequests.push_back(newRequest);
    }
}

// Each agent will share its current request vector with each of its neighbors
void allSendRequestsScheduled(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        for (unsigned long int j = 0; j < allAgents.size(); j++){
            if(i != j){
                if(allAgents[i].agent->positionsToAsk[j]){
                    Simulator::Schedule(Seconds(0.5), &sendRequestInfo, allAgents[i], allAgents[j], interface);
                }
            }
        }
    }
    // Schedule merging positions
    Simulator::Schedule(Seconds(0.5), &mergeAllPositionInfo, allAgents, allTasks, interface);
}

void mergeAllPositionInfo(std::vector<AgentNode> &allAgents, std::vector<TaskNode> allTasks, Ipv4InterfaceContainer interface){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        mergeReceivedPositions(allAgents[i].agent);
    }
    // Schedule sending all positionInfo
    Simulator::Schedule(Seconds(0.5), &allSendPositionInfoScheduled, allAgents, allTasks, interface);
}

void allSendPositionInfoScheduled(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface){
    // Have all agents determine what to send and then send it out
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        // determinePositionMessagesToSend(allAgents[i]);
        // Simulator::Schedule(Seconds(0.5), &sendPositionMessagesInBuffer, i, allAgents, interface);
        //allDetermineAndSendPositionMessages(allAgents.at(i), allAgents, interface);
        Simulator::Schedule(Seconds(0.5), &allDetermineAndSendPositionMessages, allAgents.at(i), allAgents, interface);
    }
    Simulator::Schedule(Seconds(0.5), &processAndMove, allAgents, allTasks, interface);
}

void processAndMove(std::vector<AgentNode> &allAgents, std::vector<TaskNode> allTasks, Ipv4InterfaceContainer interface){
    // Merge all requests
    mergeAllReceivedRequests(allAgents);

    // Determine assignments
    computeAllParitalAssignmentsHungarian(allAgents, allTasks);
    determineAssignedLocation(allAgents, allTasks);

    // If assignment changed, broadcast position to neighbors
    allBroadcastPositionIfChanged(allAgents, interface);
      
    // Move all agents towards their assigned location
    moveAllAgentsTowardsGoalStep(allAgents);

    // Check if all tasks assigned
    checkIfDone(allAgents);
    globalInfo::counter++;

    // Print agent info debugging
    // for(unsigned long int i = 0; i < allAgents.size(); i++){
    //     if(i == 1){
    //       allAgents[i].agent->printPosition();
    //       std::cout << " ";
    //       //allAgents[i].agent->printAssignedPosition();
    //       //allAgents.at(i).agent->printNeededInfo();
    //       printKnownPositions(allAgents[i]);
    //     }
    // }
    // std::cout << std::endl;
    //std::cout << "Number of dropped packets: " << totalNumRequestMessagesSent(allAgents) + totalNumPositionMessagesSent(allAgents) - globalInfo::totalMessagesReceived << std::endl;
    Simulator::Schedule(Seconds(0.5), &calculateCostsAndPrepareRequests, allAgents, allTasks, interface);
}

void computeAllParitalAssignmentsHungarian(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        computePartialAssignmentHungarian(allAgents[i].agent, allTasks);
    }
}

void determineAssignedLocation(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks){
    // Fill in each agent's assigned position and movement vector
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        int currentId = allAgents[i].agent->agentId;
        int assignedTask = allAgents[i].agent->partialAssignment[currentId];
        allAgents[i].agent->previousAssignedTaskPosition = allAgents[i].agent->assignedTaskPosition;
        allAgents[i].agent->assignedTaskPosition = allTasks[assignedTask].task->taskLocation;
        allAgents[i].agent->movementVector = createMovementVector(allAgents[i].agent->agentPosition, allAgents[i].agent->assignedTaskPosition);
        Vector diff = differenceVector(allAgents[i].agent->agentPosition, allAgents[i].agent->assignedTaskPosition);
        double distanceLeft = vectorMagnitude(diff.x, diff.y, diff.z);  // Initialize distance left to travel
        allAgents[i].agent->distanceLeft = distanceLeft;
    }
}

void moveAllAgentsTowardsGoalStep(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < allAgents.size(); i++){
        moveAgentTowardsGoalStep(allAgents[i]);
    }
}
} // End namespace ns3