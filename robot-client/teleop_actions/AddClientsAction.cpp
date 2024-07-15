#include "AddClientsAction.hpp"

#include <iostream>
#include <memory>

bool AddClientsAction::loop() {
    std::string id;
    std::cout << "Enter a remote ID to send an offer:" << std::endl;
    std::cin >> id;
    std::cin.ignore();

    if (id.empty())
        return false;

    if (id == teleoperation->getLocalId()) {
        std::cout << "Invalid remote ID (This is the local ID)" << std::endl;
        return true;
    }

    std::cout << "Offering to " + id << std::endl;
    auto pc = std::make_shared<PeerConnection>(teleoperation->getConfig(), teleoperation->getWebSocket(),
                                               teleoperation->getLocalId(), id);

    // We are the offerer, so create a data channel to initiate the process
    const std::string label = "test";
    std::cout << "Creating DataChannel with label \"" << label << "\"" << std::endl;
    pc->createDataChannel();

    teleoperation->addPeerConnection(id, pc);

    return true;
}