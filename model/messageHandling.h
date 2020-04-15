#ifndef MESSAGEHANDLING_H
#define MESSAGEHANDLING_H

#include "hungarian.h"
#include "disconnected.h"
#include "ns3/vector.h"
#include "ns3/structs.h"
#include "ns3/globalInfo.h"
#include <thread>

namespace ns3 {

    void ReceivePacket(Ptr<Socket> socket);
    void SendMessage(const char* data, int sizeOfMessage, Ptr<Node> sourceNode, Ptr<Node> desitinationNode, Ipv4InterfaceContainer interface);
    char* serializePositionInfo(sendPosition position);
    sendPosition deserializePositionInfo(std::string serialized);
    char* serializeRequestInfo(sendRequest request);
    sendRequest deserializeRequestInfo(std::string serialized);
    void sendRequestInfo(AgentNode &sender, AgentNode &receiver, Ipv4InterfaceContainer interface);
    void sendPositionInfo(AgentNode &sender, AgentNode &receiver, int whichAgent, Ipv4InterfaceContainer interface);
    void mergeReceivedRequests(Agent *ag);
    void mergeAllReceivedRequests(std::vector<AgentNode> &allAgents);
    void mergeReceivedPositions(Agent *ag);
    
    bool requestsEqual(sendRequest request1, sendRequest request2);
    bool isNewRequest(vector<sendRequest> allRequests, sendRequest request);
    bool positionsEqual(sendPosition position1, sendPosition position2);
    bool isNewPosition(vector<sendRequest> allRequests, sendRequest request);
    bool requestsAreNull(char* serializedMessage);
    bool alreadySentInfo(AgentNode &sender, int destinationId, sendPosition message);
    double hashedPosition(sendPosition thisPosition);
    
}

#endif /* AGENT_H */