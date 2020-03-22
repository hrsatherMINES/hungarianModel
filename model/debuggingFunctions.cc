#include "debuggingFunctions.h"

namespace ns3 {
    void printAllAgentsCosts(std::vector<Agent> &allAgents){
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << "Agent #" << i << ": ";
            allAgents[i].printAgentCosts();
        }
    }

    void printCostMatrix(std::vector<std::vector<double>> &costMatrix){
        std::cout << "Printing Cost Matrix\n";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            for (unsigned long int j=0; j < globalInfo::allTasks.size(); j++){
                if (costMatrix[i][j]==INT_MAX){
                    std::cout << "x ";
                }
                else std::cout << costMatrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void printRequest(Agent &ag){
        std::cout << "Agent #" << ag.agentId << "\tRequest: ";
        for (int i=0; i < ag.numAgents; i++){
            std::cout << ag.infoRequests[i] << " ";
        }
        std::cout << "\n";
    }

    void printNS3Vector(Vector &pos){
        std::cout << " (" << pos.x << "," << pos.y << "," << pos.z << ")";
    }

    void printKnownPositions(AgentNode &ag){
        std::cout << "Agent #" << ag.agent->agentId << "\tKnown positions:";
        for (int i=0; i < ag.agent->numAgents; i++){
            printNS3Vector(ag.agent->knownPositions[i]);
        }
        std::cout << "\n";
    }

    void printSingleRequest(sendRequest* req){
        std::cout << "Request from Agent #" << req->senderId << ":\t";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << req->request[i] << " ";
        }
        std::cout << std::endl;
    }

    void printSinglePositionMessage(sendPosition* positionMessage){
        std::cout << "Message from Agent #" << positionMessage->senderId << " regarding Agent #" << positionMessage->infoId << "\t Position: ";
        printNS3Vector(positionMessage->position);
        std::cout << std::endl;
    }

    void printSmallCostMatrix(std::vector<std::vector<double>> &costMatrix){
        int currentSizeR, currentSizeC;
        currentSizeR=costMatrix.size();
        for (int i=0; i < currentSizeR; i++){
            currentSizeC=costMatrix[i].size();
            for (int j=0; j < currentSizeC; j++){
                std::cout << costMatrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void prinAgentInfo(std::vector<Agent> &allAgents){
        std::cout << "Agent Information\n";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << "Agent #" << allAgents[i].agentId << "\n" << "Class #" << allAgents[i].instrumentType
                    << "\n" << "Location: (" << allAgents[i].agentPosition.x << ", " << allAgents[i].agentPosition.y
                    << ", " << allAgents[i].agentPosition.z << ")\n";
        }
        std::cout << std::endl;
    }

    void printGlobalAssignment(std::vector<std::vector<int>> &agentAssignment){
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            int currentSize=agentAssignment[i].size();
            std::cout << "Task " << i << ": ";
            for (int j=0; j < currentSize; j++){
                std::cout << agentAssignment[i][j] << "\t";
            }
            std::cout << std::endl;
        }
    }

    void printGlobalConflicts(std::vector<bool> &conflicts){
        std::cout << "Conflicts: " << std::endl;
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << conflicts[i] << " ";
        }
        std::cout << std::endl;
    }

    void printStillConsider(std::vector<bool> consider, std::vector<int> taskIds){
        int currentSize = consider.size();
        for (int i=0; i < currentSize; i++){
            std::cout << "(" << taskIds[i] << "," << consider[i] << ") ";
        }
        std::cout << std::endl;
    }

    void printTaskInfo(std::vector<Task> &allTasks){
        std::cout << "Task Information" << std::endl;
        for (unsigned long int i=0; i < globalInfo::allTasks.size(); i++){
            std::cout << "Task #" << allTasks[i].taskId << "\n" << "Class #" << allTasks[i].instrumentRequirement << "\n" << "Location: (" << allTasks[i].taskLocation.x << ", " << allTasks[i].taskLocation.y << ", " << allTasks[i].taskLocation.z << ")\n";
        }
        std::cout << std::endl;
    }

    void printInstrumentAssignmentVec(){
        std::cout << "Instrument Assignment: ";
        for (unsigned long int i=0; i < globalInfo::allAgents.size(); i++){
            std::cout << globalInfo::instrumentAssignment[i] << " ";
        }
        std::cout << "\n";
    }
}