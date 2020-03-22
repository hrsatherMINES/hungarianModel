/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef DISCONNECTED_H
#define DISCONNECTED_H

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/log.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/packet.h"
#include "ns3/internet-apps-module.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <limits.h>
#include "hungarian.h"
#include <cmath>
#include <stdio.h>
#include <chrono>
#include "ns3/agent.h"
#include "ns3/structs.h"
#include "ns3/hungarian.h"
#include "ns3/globalInfo.h"
#include "ns3/messageHandling.h"


namespace ns3 {
    double euclideanDistance(Vector taskPosition, Vector agentPosition);
    Vector addNS3Vectors(Vector v1, Vector v2);
    int getInstrumentType(int id);
    std::vector<Task> createTasks();
    std::vector<Agent*> createAgents();
    void calculateAllCosts(std::vector<TaskNode> allTasks, std::vector<AgentNode> allAgents);
    std::vector<std::vector<double>> createCostMatrix(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfo(std::vector<AgentNode> &allAgents);
    void initializeAllRequests(std::vector<AgentNode> &allAgents);
    void sendPositionInfo(AgentNode &sender, AgentNode &receiver, int whichAgent, Ipv4InterfaceContainer interface);
    bool compareBoolArr(bool* prev, bool* req);
    void determinePositionMessagesToSend(AgentNode *ag);
    void sendPositionMessagesInBuffer(int currentAgent, std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface);
    double computePartialAssignmentHungarian(Agent &ag, std::vector<TaskNode> &allTasks);
    void computeAllParitalAssignmentsHungarian(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks);
    double vectorMagnitude(double x, double y, double z);
    Vector differenceVector(Vector &agentPosition, Vector &taskPosition);
    Vector createMovementVector(Vector &agentPosition, Vector &taskPosition);
    void determineAssignedLocation(std::vector<AgentNode> &allAgents, std::vector<TaskNode> &allTasks);
    bool allAgentsAssigned(std::vector<AgentNode> &allAgents);
    void allSendPositionInfo(std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface);
    void moveAllAgentsTowardsGoalStep(std::vector<AgentNode> &allAgents);
    double randomDouble(double fMin, double fMax);
    Vector getPosition (Ptr<Node> node);
    void setPosition (Ptr<Node> node, Vector position);
    void movePositions(Ptr<Node> node);
    void moveAllPositions(NodeContainer robots);
    void updatePosition(AgentNode* ag);
    void moveAgentTowardsGoalStep(AgentNode ag);
    void fillAllLocalCosts(std::vector<AgentNode> &allAgents);
    int totalNumPositionMessagesSent(std::vector<AgentNode> allAgents);
    int totalNumRequestMessagesSent(std::vector<AgentNode> allAgents);
    double totalDistanceTraveled(std::vector<AgentNode> allAgents);
    void checkIfDone(std::vector<AgentNode> allAgents);
    void addOwnRequestToRequestList(std::vector<AgentNode> allAgents);
    void allSendRequests(std::vector<AgentNode> allAgents, Ipv4InterfaceContainer interface);
    bool conflictsExist(std::vector<AgentNode> &allAgents);
    bool** createWhoRequested();
    bool compareBoolArr(bool* prev, bool* req);

    // New heuristics
    void determineAllNeededInfoOriginal(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoStillMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoSelfMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoBothMoving(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoDistance(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfoDistanceMoving(std::vector<AgentNode> &allAgents);
}

#endif /* DISCONNECTED_EXP_H */
