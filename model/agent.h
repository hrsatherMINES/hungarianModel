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
        unsigned long int** sentTimes;

        bool* neededInfo;  // To keep track of what needed personally
        bool* infoRequests;  // To store request to
        bool* previousSentRequest;
        bool* knownInfo;
        bool* whichPositionsToSend;  // Will act as a message buffer
        Vector* knownPositions;
        bool** whoRequested;
        int* partialAssignment;
        bool haveAllNeededInfo;

        // Global info
        int numAgents;
        int numTasks;

        std::vector<std::vector<double>> localCostMatrix;

        // Methods
        void printPosition();
        void updateTaskPosition(double x, double y, double z);
        void fillInAgentCosts(std::vector<TaskNode> &allTasks);
        void printAgentCosts();
        void printNeededInfo();
        void initializeInfoRequests();
        void initializePreviousKnownPositions();
        void initializeKnownPositions();
        void initializeKnownInfo();
        void printKnownInfo();
        void printAssignedPosition();
        void initializePartialAssignment();
        void initializeReceivedTimes();
        void initializeNeededInfo();
        void initializeSentTimes();
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

        // Different heuristics
        void determineNeededInfoOriginal();
        void determineNeededInfoStillMoving();
        void determineNeededInfoSelfNotAssigned();
        void determineNeededInfoBothMoving();
        void determineNeededInfoDistance();
        void determineNeededInfoDistanceMoving();
    };
}

#endif /* AGENT_H */