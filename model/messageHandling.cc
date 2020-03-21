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
        // if(node->GetId() == 3) std::cout << node->GetId() << " received " << msg << std::endl;
        std::cout << node->GetId() << " received " << msg << std::endl;
        delete[] buf;
        //std::cout << "received" << msg << std::endl;
        // Received request for position
        if(msg[0] == 'R'){
            send_request newRequest = deserializeRequestInfo(msg);
            // make sure unique
            if(isNewRequest(globalInfo::allAgents.at(node->GetId()).agent->received_requests, newRequest)){
                // Increment number of hops
                newRequest.num_hops++;
                globalInfo::allAgents.at(node->GetId()).agent->received_requests.push_back(newRequest);
            }
        }
        // Received position
        else if(msg[0] == 'P'){
            send_position newPosition = deserializePositionInfo(msg);
            globalInfo::allAgents.at(node->GetId()).agent->received_positions.push_back(newPosition);
            
            //std::cout << "received" << msg << std::endl;
            //if((node->GetId() == 1 && newPosition.sender_id == 3) || (node->GetId() == 3 && newPosition.sender_id == 1))
            //if(node->GetId() == 1 && newPosition.sender_id == 8) std::cout << node->GetId() << " received position " << msg << std::endl;
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
        
        std::cout << "sending " << data << " from " << sourceNode->GetId() << " to " << desitinationNode->GetId() << std::endl;
        //if(sourceNode->GetId() == 1 && desitinationNode->GetId() == 3) std::cout << "sending " << data << " from " << sourceNode->GetId() << " to " << desitinationNode->GetId() << std::endl;
    }

    char* serializePositionInfo(send_position position){
        std::string serialized = "";
        serialized += "P@";
        serialized += to_string(position.sender_id);
        serialized += "@";
        serialized += to_string(position.info_id);
        serialized += "@";
        serialized += to_string(position.position.x);
        serialized += "@";
        serialized += to_string(position.position.y);
        serialized += "@";
        serialized += to_string(position.position.z);
        serialized += "@";
        serialized += to_string(position.time_sent);
        serialized += "@";
        serialized += "\0";
        // Convert to char*
        char *cstr = new char[serialized.length()];
        strcpy(cstr, serialized.c_str());
        return cstr;
    }

    send_position deserializePositionInfo(std::string serialized){
        send_position newPosition;
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
        newPosition.sender_id = stoi(items.at(1));
        newPosition.info_id = stoi(items.at(2));
        newPosition.position.x = stod(items.at(3));
        newPosition.position.y = stod(items.at(4));
        newPosition.position.z = stod(items.at(5));
        newPosition.time_sent = stoll(items.at(6));
        return newPosition;
    }

    char* serializeRequestInfo(send_request request){
        std::string serialized = "";
        serialized += "R@";
        serialized += to_string(request.sender_id);
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
        serialized += to_string(request.num_hops);
        serialized += "@";
        serialized += "\0";
        // Convert to char*
        char *cstr = new char[serialized.length()];
        strcpy(cstr, serialized.c_str());
        return cstr;
    }

    send_request deserializeRequestInfo(std::string serialized){
        send_request newRequest;
        std::string temp;
        size_t pos = 0;
        vector<std::string> items;
        while ((pos = serialized.find("@")) != std::string::npos) {
            temp = serialized.substr(0, pos);
            items.push_back(temp);
            serialized.erase(0, pos + 1);
        }
        // Extract id
        newRequest.sender_id = stoi(items.at(1));
        // Extract requests
        bool* request_to_send = new bool[globalInfo::allAgents.size()];
        for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
            request_to_send[i] = (int)(items.at(2)[i]) - '0';
        }
        newRequest.request = request_to_send;
        // Extract num hops
        newRequest.num_hops = stoi(items.at(3));
        return newRequest;
    }

    bool requests_are_null(char* serializedMessage){
        for(int i = 0; serializedMessage[i] != '\0'; i++){
            if(serializedMessage[i] == '1') return false;
        }
        return true;
    }

    void send_request_info(AgentNode &sender, AgentNode &receiver, Ipv4InterfaceContainer interface){
        // Send all requests in vector
        for(unsigned long int i = 0; i < sender.agent->received_requests.size(); i++){
            // Don't send if request is for itself to itself
            if((sender.agent->received_requests.at(i).sender_id == sender.agent->agent_id)
                    && (i != sender.agent->received_requests.size() - 1)){
                continue;
            }
            // Don't send if request is to who sent it
            if(sender.agent->received_requests.at(i).sender_id == receiver.agent->agent_id){
                continue;
            }
            // Serialize message
            char* serializedMessage = serializeRequestInfo(sender.agent->received_requests.at(i));
            int messageSize = 0;
            for(int j = 0; serializedMessage[j] != '\0'; j++){
                messageSize++;
            }
            // Check if no requests
            if(requests_are_null(serializedMessage)) return;

            // Send request
            // If request is one hop, send for sure, else, stochastically send
            if(sender.agent->received_requests.at(i).num_hops != 1){
                int randNum = rand()%100;
                if(randNum < globalInfo::probabilityDropped){
                    return;
                }
            }
            sender.agent->num_request_messages_sent++;
            SendMessage(serializedMessage, messageSize + 1, sender.node, receiver.node,  interface);
            delete[] serializedMessage;
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

    bool alreadySentInfo(AgentNode &sender, int destinationId, send_position message){
        for(int i = 0; i < sender.agent->numAgents; i++){
            if(sender.agent->sent_times[destinationId][message.info_id] == message.time_sent){
                return true;
            }
        }
        return false;
    }

    void send_position_info(AgentNode &sender, AgentNode &receiver, int which_agent, Ipv4InterfaceContainer interface){
        send_position to_send = sender.agent->create_send_position(which_agent);
        char* serializedMessage = serializePositionInfo(to_send);
        int messageSize = 0;
        for(int i = 0; serializedMessage[i] != '\0'; i++){
            messageSize++;
        }
        // Don't send if info is for node that we would send to
        if(to_send.info_id == receiver.agent->agent_id) return;
        // Dont send if already sent same info to same destination previously
        if(alreadySentInfo(sender, receiver.agent->agent_id, to_send)){
            return;
        }
        // Drop stochastically if num hops away
        if(sender.agent->agent_id != which_agent){
            int randNum = rand()%100;
            if(randNum < globalInfo::probabilityDropped){
                return;
            }
        }
        // Send
        sender.agent->num_position_messages_sent++;
        SendMessage(serializedMessage, messageSize + 1, sender.node, receiver.node, interface);
        delete[] serializedMessage;
        // Update last info sent
        sender.agent->sent_times[receiver.agent->agent_id][to_send.info_id] = to_send.time_sent;
        //printDebug(sender.agent->sent_times);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Merge related functions

    void merge_received_requests(Agent *ag){
        for(unsigned long int i = 0; i < ag->received_requests.size(); i++){
            int who_sent = ag->received_requests.at(i).sender_id;
            for(unsigned long int j=0; j < globalInfo::allAgents.size(); j++){
                // Need to setup who requested matrix
                if (ag->received_requests.at(i).request[j]){
                    ag->info_requests[j]=true;
                    ag->who_requested[who_sent][j]=true;
                }
                else{
                    // Fill in who request matrix with false
                    ag->who_requested[who_sent][j]=false;
                }
            }
        }
        // Remove all processed requests
        ag->received_requests.clear();
    }

    void merge_all_received_requests(std::vector<AgentNode> &all_a){
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            merge_received_requests(all_a[i].agent);
        }
    }

    void merge_received_positions(Agent *ag){
        for(unsigned long int i = 0; i < ag->received_positions.size(); i++){
            int who_sent = ag->received_positions.at(i).sender_id;
            int p_id = ag->received_positions.at(i).info_id;
            if (ag->received_times[p_id] < ag->received_positions.at(i).time_sent){
                // if(ag->agent_id == 1)  (std::cout << "updated " << ag->received_positions.at(i).info_id << " to "
                //             << ag->received_positions.at(i).position <<std::endl);
                ag->known_info[p_id]=true;
                ag->known_positions[p_id] = ag->received_positions.at(i).position;
                ag->info_requests[p_id] = false;
                ag->received_times[p_id] = ag->received_positions.at(i).time_sent;
            }
            // Neighbor who sent informnation should no longer be requesting it
            ag->who_requested[who_sent][p_id]=false;
        }
        // Remove all processed requests
        ag->received_positions.clear();
        // Update own info
        std::chrono::milliseconds ms =
                std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
        ag->received_times[ag->agent_id] = ms.count();
        ag->known_positions[ag->agent_id] = ag->agent_position;
    }



    ///////////////////////////////////////////////////////////////////////////
    // Not needed but useful for debugging

    bool requestsEqual(send_request request1, send_request request2){
        bool idEqual = request1.sender_id == request2.sender_id;
        for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
            if(request1.request[i] != request2.request[i]){
                return false;
            }
        }
        return idEqual;
    }

    bool isNewRequest(vector<send_request> allRequests, send_request request){
        for(unsigned long int i = 0; i < allRequests.size(); i++){
            if(requestsEqual(allRequests.at(i), request)){
                return false;
            }
        }
        return true;
    }

    // bool positionsEqual(send_position position1, send_position position2){
    //     bool idsEqual = (position1.info_id == position2.info_id)
    //                 && (position1.sender_id == position2.sender_id);
    //     for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
    //         if(position1.request[i] != position2.request[i]){
    //             return false;
    //         }
    //     }
    //     return idEqual;
    // }

    bool isNewPosition(vector<send_request> allRequests, send_request request){
        for(unsigned long int i = 0; i < allRequests.size(); i++){
            if(requestsEqual(allRequests.at(i), request)){
                return false;
            }
        }
        return true;
    }
}