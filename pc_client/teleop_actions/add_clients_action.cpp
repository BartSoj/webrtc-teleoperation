#include "add_clients_action.hpp"

#include <iostream>
#include <memory>

bool AddClientsAction::loop()
{
    std::string id;
    std::cout << "Enter a remote ID to send an offer:" << std::endl;
    std::cin >> id;
    std::cin.ignore();

    if(id.empty()) return false;

    if(id == teleoperation->getLocalId())
    {
        std::cout << "Invalid remote ID (This is the local ID)" << std::endl;
        return true;
    }

    std::cout << "Offering to " + id << std::endl;
    PeerConnection::Configuration config;
    config.rtcConfig = teleoperation->getConfig();
    config.wws = teleoperation->getWebSocket();
    config.localId = teleoperation->getLocalId();
    config.remoteId = id;
    auto pc = std::make_shared<PeerConnection>(config);

    // We are the offerer, so create a data channel to initiate the process
    std::cout << "Creating DataChannel" << std::endl;
    pc->createDataChannel();

    teleoperation->addPeerConnection(id, pc);

    return true;
}