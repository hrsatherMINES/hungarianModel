#include "newMethod.h"

namespace ns3 {
// Different heuristics for determining which info is needed
void determineAllNeededInfoOriginal(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoOriginal();
    }
}

void determineAllNeededInfoStillMoving(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoStillMoving();
    }
}

void determineAllNeededInfoSelfMoving(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoSelfNotAssigned();
    }
}

void determineAllNeededInfoBothMoving(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoBothMoving();
    }
}

void determineAllNeededInfoDistance(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoDistance();
    }
}

void determineAllNeededInfoDistanceMoving(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoDistanceMoving();
    }
}

void determineAllNeededInfoInferTask(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoInferTask();
    }
}

void determineAllNeededInfoInferTaskAndMoving(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoInferTaskAndMoving();
    }
}

void determineAllNeededInfoOneHop(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        allAgents.at(i).agent->determineNeededInfoOneHop();
    }
}

// End of Heuristics
void calculateAllCosts(std::vector<TaskNode> &allTasks, std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < globalInfo::allAgents.size(); i++) {
        allAgents.at(i).agent->fillInAgentCosts(allTasks);
    }
}

void fillAllLocalCosts(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < globalInfo::allAgents.size(); i++) {
        allAgents.at(i).agent->fillLocalCostMatrix();
    }
}

void calculateCostsAndPrepareRequests(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface) {
    globalInfo::numMoves++;

    // Process costs
    calculateAllCosts(allTasks, allAgents);
    fillAllLocalCosts(allAgents);
    
    // Different heuristics for requests CHANGE HERE ***
    determineAllNeededInfoInferTaskAndMoving(allAgents);
    // CHANGE HERE *************************************

    // Prepare requests
    initializeAllRequests(allAgents);
    // Add own request to request list
    addOwnRequestToRequestList(allAgents);

    // Schedule request messages
    Simulator::Schedule(Seconds(0.5), &allSendRequests, allAgents, allTasks, interface);
}

// Fill in initial request array with needed info (more will be added once shared)
// Also set up previous sent request
void initializeAllRequests(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        for (unsigned long int j = 0; j < allAgents.size(); j++) {
            allAgents.at(i).agent->infoRequests[j] = allAgents.at(i).agent->neededInfo[j];
        }
    }
}

void addOwnRequestToRequestList(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        sendRequest newRequest = allAgents.at(i).agent->createSendRequest();
        allAgents.at(i).agent->receivedRequests.push_back(newRequest);
    }
}

// Each agent will share its current request vector with each of its neighbors
void allSendRequests(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        for (unsigned long int j = 0; j < allAgents.size(); j++) {
            if (i != j) {
                if (allAgents.at(i).agent->positionsToAsk[j]) {
                    Simulator::Schedule(Seconds(0.5), &sendRequestInfo, allAgents.at(i), allAgents.at(j), interface);
                }
            }
        }
    }
    // Schedule merging requests
    Simulator::Schedule(Seconds(0.5), &mergeAllReceivedRequests, allAgents, allTasks, interface);
}

void mergeAllReceivedRequests(std::vector<AgentNode> &allAgents, std::vector<TaskNode> allTasks, Ipv4InterfaceContainer interface) {
    for (unsigned long int i = 0; i < globalInfo::allAgents.size(); i++) {
        mergeReceivedRequests(allAgents.at(i).agent);
    }
    // Schedule sending positions
    Simulator::Schedule(Seconds(0.5), &allSendPositionInfo, allAgents, allTasks, interface);
}

void allSendPositionInfo(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks, Ipv4InterfaceContainer interface) {
    // Have all agents determine what to send and then send it out
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        Simulator::Schedule(Seconds(0.5), &allDetermineAndSendPositionMessages, allAgents.at(i), allAgents, interface);
    }
    Simulator::Schedule(Seconds(0.5), &mergeAllPositionInfo, allAgents, allTasks, interface);
}

void mergeAllPositionInfo(std::vector<AgentNode> &allAgents, std::vector<TaskNode> allTasks, Ipv4InterfaceContainer interface) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        mergeReceivedPositions(allAgents.at(i).agent);
    }
    // Schedule sending all positionInfo
    Simulator::Schedule(Seconds(0.5), &processAndMove, allAgents, allTasks, interface);
}

void processAndMove(std::vector<AgentNode> &allAgents, std::vector<TaskNode> allTasks, Ipv4InterfaceContainer interface) {
    // Determine assignments
    computeAllParitalAssignmentsHungarian(allAgents, allTasks);\
    determineAssignedLocation(allAgents, allTasks);
    
    // If assignment changed, broadcast position to neighbors
    allBroadcastPositionIfChanged(allAgents, interface);
    
    // Move all agents towards their assigned location
    moveAllAgentsTowardsGoalStep(allAgents);

    // Check if all tasks assigned
    if (checkIfDone(allAgents)) {
        giveAllAgentsRandomPositions(globalInfo::allAgents);
        updateAllPositions(globalInfo::allAgents);
        giveAllTasksRandomPositions(globalInfo::allTasks);
        updateAllTasks(globalInfo::allTasks);
        resetAgents(globalInfo::allAgents);

        Simulator::Schedule(Seconds(1), &runTests, globalInfo::allTasks, globalInfo::allAgents, interface);
        return;
    }
        
    Simulator::Schedule(Seconds(0.5), &calculateCostsAndPrepareRequests, allAgents, allTasks, interface);

    // Print agent info debugging
    // for (unsigned long int i = 0; i < allAgents.size(); i++) {
    //     if (i == 1) {
    //       allAgents.at(i).agent->printPosition();
    //       std::cout << " ";
    //       //allAgents.at(i).agent->printAssignedPosition();
    //       //allAgents.at(i).agent->printNeededInfo();
    //       printKnownPositions(allAgents.at(i));
    //     }
    // }
    // std::cout << std::endl;
    //std::cout << "Number of dropped packets: " << totalNumRequestMessagesSent(allAgents) + totalNumPositionMessagesSent(allAgents) - globalInfo::totalMessagesReceived << std::endl;
}

void computeAllParitalAssignmentsHungarian(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        computePartialAssignmentHungarian(allAgents.at(i).agent, allTasks);
    }
}

void determineAssignedLocation(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks) {
    // Fill in each agent's assigned position and movement vector
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        int currentId = allAgents.at(i).agent->agentId;
        int assignedTask = allAgents.at(i).agent->partialAssignment[currentId];
        allAgents.at(i).agent->previousAssignedTaskPosition = allAgents.at(i).agent->assignedTaskPosition;
        allAgents.at(i).agent->assignedTaskPosition = allTasks[assignedTask].task->taskLocation;
        allAgents.at(i).agent->movementVector = createMovementVector(allAgents.at(i).agent->agentPosition, allAgents.at(i).agent->assignedTaskPosition);
        Vector diff = differenceVector(allAgents.at(i).agent->agentPosition, allAgents.at(i).agent->assignedTaskPosition);
        double distanceLeft = vectorMagnitude(diff.x, diff.y, diff.z);  // Initialize distance left to travel
        allAgents.at(i).agent->distanceLeft = distanceLeft;
    }
}

void moveAllAgentsTowardsGoalStep(std::vector<AgentNode> &allAgents) {
    for (unsigned long int i = 0; i < allAgents.size(); i++) {
        moveAgentTowardsGoalStep(allAgents.at(i));
    }
}
} // End namespace ns3