#include "messageHandling.h"

namespace ns3 {
    void ReceivePacket(Ptr<Socket> socket){
    Ptr<Node> node = socket->GetNode();
    Ptr<Packet> packet = socket->Recv();
    if(socket == NULL || node == NULL || packet == NULL) return;
    uint8_t *buf = new uint8_t[packet->GetSize()];
    packet->CopyData(buf, packet->GetSize());
    std::string msg = std::string(reinterpret_cast<const char *>(buf), packet->GetSize());
    delete[] buf;
    //std::cout << "received" << msg << std::endl;
    // Received request for position
    if(msg[0] == 'R'){
        send_request newRequest = deserializeRequestInfo(msg);
        // make sure unique
        if(isNewRequest(globalInfo::allAgents.at(node->GetId()).agent->received_requests, newRequest)){
        globalInfo::allAgents.at(node->GetId()).agent->received_requests.push_back(newRequest);
        }
        
        //std::cout << "Request" << msg << std::endl;
        //if(node->GetId() == 1 ||  node->GetId() == 8) std::cout << node->GetId() << " received " << msg << std::endl;
    }
    // Received position
    else if(msg[0] == 'P'){
        send_position newPosition = deserializePositionInfo(msg);
        globalInfo::allAgents.at(node->GetId()).agent->received_positions.push_back(newPosition);
        
        //std::cout << "received" << msg << std::endl;
        //if((node->GetId() == 1 && newPosition.sender_id == 3) || (node->GetId() == 3 && newPosition.sender_id == 1))
        if(node->GetId() == 1) std::cout << node->GetId() << " position " << msg << std::endl;
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
    
    if(sourceNode->GetId() == 8) std::cout << "sending " << data << " from " << sourceNode->GetId() << " to " << desitinationNode->GetId() << std::endl;
    }

    void send_new_requests(std::vector<AgentNode> &all_a){
        for (unsigned long int i=0; i < all_a.size(); i++){
            //if the agent has a new updated request
            if (!compare_bool_arr(all_a[i].agent->previous_sent_request, all_a[i].agent->info_requests )){
                //update previous sent vector to be the one that is about to be sent
                for (unsigned long int j=0; j < all_a.size(); j++){
                    all_a[i].agent->previous_sent_request[j]=all_a[i].agent->info_requests[j];
                }

                //send a broadcast to all neighbors
                all_a[i].agent->num_request_messages_sent++;
                for (unsigned long int j=0; j < all_a.size(); j++){
                    //if(comm_g[i][j]){
                        //send_request_info(all_a[i], all_a[j]);
                    //}
                }
            }
        }
        //done sending request messages for the current round
        //process the messages now
        merge_all_received_requests(all_a);
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
        // convert to char*
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
        // fill in info
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
        serialized += "\0";
        // convert to char*
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
        // extract id
        newRequest.sender_id = stoi(items.at(1));
        // extract requests
        bool* request_to_send = new bool[globalInfo::allAgents.size()];
        for(unsigned long int i = 0; i < globalInfo::allAgents.size(); i++){
            request_to_send[i] = (int)(items.at(2)[i]) - '0';
        }
        newRequest.request = request_to_send;
        return newRequest;
    }

    void send_request_info(AgentNode &sender, AgentNode &receiver, Ipv4InterfaceContainer interface){
        send_request newRequest = sender.agent->create_send_request();
        sender.agent->received_requests.insert(sender.agent->received_requests.begin(), newRequest);
        for(unsigned long int i = 0; i < sender.agent->received_requests.size(); i++){
            
            char* serializedMessage = serializeRequestInfo(sender.agent->received_requests.at(i));
            int messageSize = 0;
            for(int i = 0; serializedMessage[i] != '\0'; i++){
                messageSize++;
            }
            SendMessage(serializedMessage, messageSize + 1, sender.node, receiver.node,  interface);
            delete[] serializedMessage;
        }
    }

    void send_position_info(AgentNode &sender, AgentNode &receiver, int which_agent, Ipv4InterfaceContainer interface){
        send_position to_send = sender.agent->create_send_position(which_agent);
        char* serializedMessage = serializePositionInfo(to_send);
        int messageSize = 0;
        for(int i = 0; serializedMessage[i] != '\0'; i++){
            messageSize++;
        }
        SendMessage(serializedMessage, messageSize + 1, sender.node, receiver.node, interface);
        delete[] serializedMessage;

        //std::cout << "Sent position from " << sender.agent_id << " to " << receiver.agent_id << " regarding " << which_agent << std::endl;
    }

    ///////////////////////////////////////////////////////////////////////////
    //merge related functions

    void merge_received_requests(Agent *ag){
        for(unsigned long int i = 0; i < ag->received_requests.size(); i++){
            int who_sent = ag->received_requests.at(i).sender_id;
            for(unsigned long int j=0; j < globalInfo::allAgents.size(); j++){
                //need to setup who requested matrix
                if (ag->received_requests.at(i).request[j]){
                    ag->info_requests[j]=true;
                    ag->who_requested[who_sent][j]=true;
                }
                else{
                    //fill in who request matrix with false
                    ag->who_requested[who_sent][j]=false;
                }
            }
        }
        // remove all processed requests
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
            //update if new message
            if (ag->received_times[p_id] < ag->received_positions.at(i).time_sent){
                ag->known_info[p_id]=true;
                ag->known_positions[p_id] = ag->received_positions.at(i).position;
                ag->info_requests[p_id] = false;
                ag->received_times[p_id] = ag->received_positions.at(i).time_sent;
            }
            //neighbor who sent informnation should no longer be requesting it
            ag->who_requested[who_sent][p_id]=false;
        }
        // remove all processed requests
        ag->received_positions.clear();
        // update own info
        std::chrono::milliseconds ms =
                std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() );
        ag->received_times[ag->agent_id] = ms.count();
        ag->known_positions[ag->agent_id] = ag->agent_position;
    }



    ///////////////////////////////////////////////////////////////////////////
    //Not needed but useful for debugging

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