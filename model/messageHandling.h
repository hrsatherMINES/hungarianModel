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
    char* serializePositionInfo(send_position position);
    send_position deserializePositionInfo(std::string serialized);
    char* serializeRequestInfo(send_request request);
    send_request deserializeRequestInfo(std::string serialized);
    void send_request_info(AgentNode &sender, AgentNode &receiver, Ipv4InterfaceContainer interface);
    void send_position_info(AgentNode &sender, AgentNode &receiver, int which_agent, Ipv4InterfaceContainer interface);
    void merge_received_requests(Agent *ag);
    void merge_all_received_requests(std::vector<AgentNode> &all_a);
    void merge_received_positions(Agent *ag);
    bool requestsEqual(send_request request1, send_request request2);
    bool isNewRequest(vector<send_request> allRequests, send_request request);
    bool positionsEqual(send_position position1, send_position position2);
    bool isNewPosition(vector<send_request> allRequests, send_request request);
    bool requests_are_null(char* serializedMessage);
}

#endif /* AGENT_H */