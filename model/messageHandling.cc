#include "messageHandling.h"

namespace ns3 {
void ReceivePacket(Ptr<Socket> socket){
    globalInfo::totalMessagesReceived++;
    Ptr<Node> node = socket->GetNode();
    Ptr<Packet> packet = socket->Recv();
    if(socket == NULL || node == NULL || packet == NULL){
        std::cout << "Received NULL" << std::endl;
        return;
    }
    uint8_t *buf = new uint8_t[packet->GetSize()];
    packet->CopyData(buf, packet->GetSize());
    std::string msg = std::string(reinterpret_cast<const char *>(buf), packet->GetSize());
    delete[] buf;
    // Received request for position
    if(msg[0] == 'R'){
        sendRequest newRequest = deserializeRequestInfo(msg);
        // Record who sent the info if it is one hop
        if(newRequest.numHops == 0){
            std::chrono::milliseconds ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
            globalInfo::allAgents.at(node->GetId()).agent->lastTimeHeardFrom[newRequest.senderId] = ms.count();
        }
        // Make sure unique before saving
        if(isNewRequest(globalInfo::allAgents.at(node->GetId()).agent->receivedRequests, newRequest)){
            // Increment number of hops
            newRequest.numHops++;
            globalInfo::allAgents.at(node->GetId()).agent->receivedRequests.push_back(newRequest);
            // Debug
            if(node->GetId() == 6) std::cout << node->GetId() << " received request " << msg << std::endl;
        }
    }
    // Received position
    else if(msg[0] == 'P'){
        sendPosition newPosition = deserializePositionInfo(msg);
        // Record who sent the info
        std::chrono::milliseconds ms =
                std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
        globalInfo::allAgents.at(node->GetId()).agent->lastTimeHeardFrom[newPosition.senderId] = ms.count();
        // Add position to info
        globalInfo::allAgents.at(node->GetId()).agent->receivedPositions.push_back(newPosition);
        //if((node->GetId() == 1 && newPosition.senderId == 3) || (node->GetId() == 3 && newPosition.senderId == 1))
        if(node->GetId() == 6) std::cout << node->GetId() << " received position " << msg << std::endl;
    }
}

void SendMessage(const char* data, int sizeOfMessage, Ptr<Node> sourceNode, Ptr<Node> desitinationNode, Ipv4InterfaceContainer interface){
    TypeId id = UdpSocketFactory::GetTypeId ();
    Ptr<Socket> source = Socket::CreateSocket (sourceNode, id);
    InetSocketAddress remote = InetSocketAddress (interface.GetAddress(desitinationNode->GetId()), 80);
    if(source->Connect(remote) == -1){
        std::cout << "Error connecting" << std::endl;
        return;
    }
    Ptr<Packet> packet = Create<Packet>(reinterpret_cast<const uint8_t*>(data), sizeOfMessage);
    if(source->Send(packet) == -1){
        std::cout << "Error sending" << std::endl;
        return;
    }
    
    //std::cout << "sending " << data << " from " << sourceNode->GetId() << " to " << desitinationNode->GetId() << std::endl;
    if(sourceNode->GetId() == 6) std::cout << "sending " << data << " from " << sourceNode->GetId() << " to " << desitinationNode->GetId() << std::endl;
}

char* serializePositionInfo(sendPosition position){
    std::string serialized = "";
    serialized += "P@";
    serialized += to_string(position.senderId);
    serialized += "@";
    serialized += to_string(position.infoId);
    serialized += "@";
    serialized += to_string(position.position.x);
    serialized += "@";
    serialized += to_string(position.position.y);
    serialized += "@";
    serialized += to_string(position.position.z);
    serialized += "@";
    serialized += to_string(position.timeSent);
    serialized += "@";
    serialized += "\0";
    // Convert to char*
    char *cstr = new char[serialized.length()];
    strcpy(cstr, serialized.c_str());
    return cstr;
}

sendPosition deserializePositionInfo(std::string serialized){
    sendPosition newPosition;
    // Split on @ delimeter
    std::string temp;
    size_t pos = 0;
    vector<std::string> items;
    while ((pos = serialized.find("@")) != std::string::npos) {
        temp = serialized.substr(0, pos);
        items.push_back(temp);
        serialized.erase(0, pos + 1);
    }
    // Fill in info
    newPosition.senderId = stoi(items.at(1));
    newPosition.infoId = stoi(items.at(2));
    newPosition.position.x = stod(items.at(3));
    newPosition.position.y = stod(items.at(4));
    newPosition.position.z = stod(items.at(5));
    newPosition.timeSent = stoll(items.at(6));
    return newPosition;
}

char* serializeRequestInfo(sendRequest request){
    std::string serialized = "";
    serialized += "R@";
    serialized += to_string(request.senderId);
    serialized += "@";
    for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        if(request.request[i] == 1){
            serialized += "1";
        }
        else if(request.request[i] == 0){
            serialized += "0";
        }
    }
    serialized += "@";
    serialized += to_string(request.numHops);
    serialized += "@";
    serialized += "\0";
    // Convert to char*
    char *cstr = new char[serialized.length()];
    strcpy(cstr, serialized.c_str());
    return cstr;
}

sendRequest deserializeRequestInfo(std::string serialized){
    sendRequest newRequest;
    std::string temp;
    size_t pos = 0;
    vector<std::string> items;
    while ((pos = serialized.find("@")) != std::string::npos) {
        temp = serialized.substr(0, pos);
        items.push_back(temp);
        serialized.erase(0, pos + 1);
    }
    // Extract id
    newRequest.senderId = stoi(items.at(1));
    // Extract requests
    bool* requestToSend = new bool[globalInfo::allAgents.size()];
    for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        requestToSend[i] = (int)(items.at(2)[i]) - '0';
    }
    newRequest.request = requestToSend;
    // Extract num hops
    newRequest.numHops = stoi(items.at(3));
    return newRequest;
}

bool requestsAreNull(char* serializedMessage){
    for(int i = 0; serializedMessage[i] != '\0'; i++){
        if(serializedMessage[i] == '1') return false;
    }
    return true;
}

long int hashedRequest(sendRequest thisRequest){
    int requestArray = 0;
    for(int i = 0; i < globalInfo::numAgents; i++){
        if(thisRequest.request) requestArray += pow(2, i);
    }
    return ((long int)thisRequest.senderId << globalInfo::numAgents) + requestArray;
}

bool alreadySentRequest(AgentNode &sender, int destinationId, sendRequest message){
    for(int i = 0; i < sender.agent->numAgents; i++){
        if(sender.agent->sentRequests[destinationId][message.senderId] == hashedRequest(message)){
            return true;
        }
    }
    return false;
}

void sendRequestInfo(AgentNode &sender, AgentNode &receiver, Ipv4InterfaceContainer interface){
    // // Drop if node is not one hop
    // if(!sender.agent->agentIsOneHop(receiver.agent->agentId)){
    //     return;
    // }
    // Send all requests in request vector
    for(unsigned long int i = 0; i < sender.agent->receivedRequests.size(); i++){
        sendRequest message = sender.agent->receivedRequests.at(i);
        // Don't send if request is for itself to itself
        if((message.senderId == sender.agent->agentId)
                && (i != sender.agent->receivedRequests.size() - 1)){
            continue;
        }
        // Don't send if request is to who sent it
        if(message.senderId == receiver.agent->agentId){
            continue;
        }
        // If message is its own request message, make true for request. Reset later
        bool flippedRequest = false;
        if(message.senderId == sender.agent->agentId){
            if(!sender.agent->neededInfo[receiver.agent->agentId] && sender.agent->positionsToAsk[receiver.agent->agentId]){
                message.request[receiver.agent->agentId] = true;
                flippedRequest = true;
            }
        }
        // Serialize message
        char* serializedMessage = serializeRequestInfo(message);
        int messageSize = 0;
        for(int j = 0; serializedMessage[j] != '\0'; j++){
            messageSize++;
        }
        // Reset request
        if(flippedRequest){
            message.request[receiver.agent->agentId] = false;
        }
        // Check if no requests
        if(requestsAreNull(serializedMessage)) continue;
        // Check if already sent request. Drop stochastically
        if(alreadySentRequest(sender, receiver.agent->agentId, message)){
            int randNum = rand() % 100;
            if(randNum < globalInfo::probabilityDropped){
                continue;
            }
        }
        // Send request
        // If request is one hop, send for sure, else, stochastically send
        if(message.numHops > 0){
            int randNum = rand() % 100;
            if(randNum < globalInfo::probabilityDropped){
                continue;
            }
        }
        sender.agent->numRequestMessagesSent++;
        SendMessage(serializedMessage, messageSize + 1, sender.node, receiver.node,  interface);
        delete[] serializedMessage;
        // Update last info sent
        sender.agent->sentRequests[receiver.agent->agentId][message.senderId] = hashedRequest(message);
    }
}
    
void printDebug(vector<vector<size_t>> input){
    for(long unsigned int i = 0; i < input.size(); i++){
        for(long unsigned int j = 0; j < input.at(i).size(); j++){
            std::cout << input.at(i).at(j) << " : ";
        }
        std::cout << std::endl;
    }
}

double hashedPosition(sendPosition thisPosition){
    return thisPosition.position.x + thisPosition.position.y + thisPosition.position.z;
}

bool alreadySentInfo(AgentNode &sender, int destinationId, sendPosition message){
    if(sender.agent->sentPositions[destinationId][message.infoId] == hashedPosition(message)){
        return true;
    }
    return false;
}

void sendPositionInfo(AgentNode &sender, AgentNode &receiver, int whichAgent, Ipv4InterfaceContainer interface){
    sendPosition toSend = sender.agent->createSendPosition(whichAgent);
    char* serializedMessage = serializePositionInfo(toSend);
    int messageSize = 0;
    for(int i = 0; serializedMessage[i] != '\0'; i++){
        messageSize++;
    }
    // Don't send if info is for node that we would send to
    if(toSend.infoId == receiver.agent->agentId) return;
    // If already sent same info to same destination previously, don't send stochastically
    if(alreadySentInfo(sender, receiver.agent->agentId, toSend)){
        int randNum = rand() % 100;
        if(randNum < globalInfo::probabilityDropped){
            return;
        }
    }
    // Drop stochastically if not sending directly to agent who requested
    if(sender.agent->agentId != whichAgent){
        int randNum = rand() % 100;
        if(randNum < globalInfo::probabilityDropped){
            return;
        }
    }
    // Drop if node is not one hop
    // if(!sender.agent->agentIsOneHop(receiver.agent->agentId)){
    //     return;
    // }
    // Send
    sender.agent->numPositionMessagesSent++;
    SendMessage(serializedMessage, messageSize + 1, sender.node, receiver.node, interface);
    delete[] serializedMessage;
    // Update last info sent
    sender.agent->sentPositions[receiver.agent->agentId][toSend.infoId] = hashedPosition(toSend);
    //printDebug(sender.agent->sentPositions);
}

///////////////////////////////////////////////////////////////////////////
// Merge related functions

void mergeReceivedRequests(Agent *ag){
    for(unsigned long int i = 0; i < ag->receivedRequests.size(); i++){
        int whoSent = ag->receivedRequests.at(i).senderId;
        for(unsigned long int j=0; j < globalInfo::allAgents.size(); j++){
            // Need to setup who requested matrix
            if (ag->receivedRequests.at(i).request[j]){
                ag->infoRequests[j] = true;
                ag->whoRequested[whoSent][j] = true;
            }
            else{
                // Fill in who request matrix with false
                ag->whoRequested[whoSent][j] = false;
            }
        }
    }
    // Remove all processed requests
    ag->receivedRequests.clear();
}

void mergeAllReceivedRequests(std::vector<AgentNode> &allAgents){
    for (unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        mergeReceivedRequests(allAgents[i].agent);
    }
}

void mergeReceivedPositions(Agent *ag){
    for(unsigned long int i = 0; i < ag->receivedPositions.size(); i++){
        int whoSent = ag->receivedPositions.at(i).senderId;
        int otherAgentId = ag->receivedPositions.at(i).infoId;
        if (ag->receivedTimes[otherAgentId] < ag->receivedPositions.at(i).timeSent){
            // if(ag->agentId == 1)  (std::cout << "updated " << ag->receivedPositions.at(i).infoId << " to "
            //             << ag->receivedPositions.at(i).position <<std::endl);
            ag->knownInfo[otherAgentId] = true;
            ag->previousKnownPositions[otherAgentId] = ag->knownPositions[otherAgentId];
            ag->knownPositions[otherAgentId] = ag->receivedPositions.at(i).position;
            ag->infoRequests[otherAgentId] = false;
            ag->receivedTimes[otherAgentId] = ag->receivedPositions.at(i).timeSent;
        }
        // Neighbor who sent informnation should no longer be requesting it
        ag->whoRequested[whoSent][otherAgentId] = false;
    }
    // Remove all processed requests
    ag->receivedPositions.clear();
    // Update own info
    std::chrono::milliseconds ms =
            std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
    ag->receivedTimes[ag->agentId] = ms.count();
    ag->knownPositions[ag->agentId] = ag->agentPosition;
}



///////////////////////////////////////////////////////////////////////////
// Not needed but useful for debugging

bool requestsEqual(sendRequest request1, sendRequest request2){
    bool idEqual = request1.senderId == request2.senderId;
    for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
        if(request1.request[i] != request2.request[i]){
            return false;
        }
    }
    return idEqual;
}

bool isNewRequest(vector<sendRequest> allRequests, sendRequest request){
    for(unsigned long int i = 0; i < allRequests.size(); i++){
        if(requestsEqual(allRequests.at(i), request)){
            return false;
        }
    }
    return true;
}

// bool positionsEqual(sendPosition position1, sendPosition position2){
//     bool idsEqual = (position1.infoId == position2.infoId)
//                 && (position1.senderId == position2.senderId);
//     for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
//         if(position1.request[i] != position2.request[i]){
//             return false;
//         }
//     }
//     return idEqual;
// }

bool isNewPosition(vector<sendRequest> allRequests, sendRequest request){
    for(unsigned long int i = 0; i < allRequests.size(); i++){
        if(requestsEqual(allRequests.at(i), request)){
            return false;
        }
    }
    return true;
}
} // End namespace ne3