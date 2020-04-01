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
#include "newMethod.h"

namespace ns3 {
    double euclideanDistance(Vector taskPosition, Vector agentPosition);
    Vector addNS3Vectors(Vector v1, Vector v2);
    int getInstrumentType(int id);
    std::vector<Task> createTasks();
    std::vector<Agent*> createAgents();
    std::vector<std::vector<double>> createCostMatrix(std::vector<AgentNode> &allAgents);
    void determineAllNeededInfo(std::vector<AgentNode> &allAgents);
    void sendPositionInfo(AgentNode &sender, AgentNode &receiver, int whichAgent, Ipv4InterfaceContainer interface);
    bool compareBoolArr(bool* prev, bool* req);
    void determinePositionMessagesToSend(AgentNode &ag);
    void sendPositionMessagesInBuffer(int currentAgent, std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface);
    double computePartialAssignmentHungarian(Agent *ag, std::vector<TaskNode> &allTasks);
    double vectorMagnitude(double x, double y, double z);
    Vector differenceVector(Vector &agentPosition, Vector &taskPosition);
    Vector createMovementVector(Vector &agentPosition, Vector &taskPosition);
    bool allAgentsAssigned(std::vector<AgentNode> &allAgents);
    double randomDouble(double fMin, double fMax);
    Vector getPosition (Ptr<Node> node);
    void setPosition (Ptr<Node> node, Vector position);
    void movePositions(Ptr<Node> node);
    void moveAllPositions(NodeContainer robots);
    void updatePosition(AgentNode* ag);
    void moveAgentTowardsGoalStep(AgentNode ag);
    int totalNumPositionMessagesSent(std::vector<AgentNode> &allAgents);
    int totalNumRequestMessagesSent(std::vector<AgentNode> &allAgents);
    double totalDistanceTraveled(std::vector<AgentNode> &allAgents);
    bool conflictsExist(std::vector<AgentNode> &allAgents);
    bool** createWhoRequested();
    bool compareBoolArr(bool* prev, bool* req);
    void checkIfDone(std::vector<AgentNode> &allAgents);
    void allBroadcastPositionIfChanged(std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface);
    void allDetermineAndSendPositionMessages(AgentNode &ag, std::vector<AgentNode> &allAgents, Ipv4InterfaceContainer interface);
}

#endif /* DISCONNECTED_H */
