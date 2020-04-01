#ifndef AGENT_H
#define AGENT_H

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"

namespace ns3 {

    class Agent{
        public:

        // Agent state
        int agentId;
        Vector agentPosition;
        Vector assignedTaskPosition;
        Vector previousAssignedTaskPosition;
        Vector movementVector;
        int instrumentType;
        double distanceTraveled;
        double distanceLeft;
        double* taskCosts;
        double speed;

        // Message totals
        int numRequestMessagesSent;
        int numPositionMessagesSent;

        // Received messages
        std::vector<sendRequest> receivedRequests;
        std::vector<sendPosition> receivedPositions;
        unsigned long int* receivedTimes;
        unsigned long int* lastTimeHeardFrom;
        double** sentPositions;
        long int** sentRequests;

        bool* neededInfo;  // To keep track of what needed personally
        bool* positionsToAsk;
        bool* infoRequests;  // To store request to
        bool* previousSentRequest;
        bool* knownInfo;
        bool* whichPositionsToSend;  // Will act as a message buffer
        Vector* knownPositions;
        Vector* previousKnownPositions;
        bool** whoRequested;
        int* partialAssignment;
        bool haveAllNeededInfo;

        // Global info
        int numAgents;
        int numTasks;

        std::vector<std::vector<double>> localCostMatrix;

        // Methods
        void printPosition();
        void fillInAgentCosts(std::vector<TaskNode> &allTasks);
        void printAgentCosts();
        void printNeededInfo();
        void initializeInfoRequests();
        void initializeLastTimeHeardFrom();
        void initializePreviousKnownPositions();
        void initializeKnownPositions();
        void initializeKnownInfo();
        void printKnownInfo();
        void printAssignedPosition();
        void initializePartialAssignment();
        void initializeReceivedTimes();
        void initializeNeededInfo();
        void initializePositionsToAsk();
        void initializeSentPositions();
        void initializeSentRequests();
        void printPartialAssignment();
        void printAssignedTaskPos();
        sendRequest createSendRequest();
        sendPosition createSendPosition(int whichAgent);
        void fillLocalCostMatrix();
        void setSpeed(double speedIn);
        void setNumAgents(int numAgentsIn);
        void setNumTasks(int numTasksIn);
        bool agentAssigned(int whichAgent);
        bool isAssigned();
        bool isClose(int whichAgent);
        bool movingTowardSameTask(int whichAgent);
        void initializeAssignedTaskPosition();
        void initializePreviousAssignedTaskPosition();
        bool assignmentChanged();
        bool agentIsOneHop(int whichAgent);

        // Different heuristics
        void determineNeededInfoOriginal();
        void determineNeededInfoStillMoving();
        void determineNeededInfoSelfNotAssigned();
        void determineNeededInfoBothMoving();
        void determineNeededInfoDistance();
        void determineNeededInfoDistanceMoving();
        void determineNeededInfoInferTask();
        void determineNeededInfoInferTaskAndMoving();
        void determineNeededInfoOneHop();
    };
}

#endif /* AGENT_H */