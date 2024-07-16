#include "../origin_webrtc_teleop/src/teleop_client/Teleoperation.hpp"
#include "teleop_actions/Action.hpp"
#include "teleop_actions/AddClientsAction.hpp"
#include "teleop_actions/SendCounterAction.hpp"
#include "teleop_actions/StreamVideoAction.hpp"


#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc, char **argv) try {
    auto teleoperation = std::make_shared<Teleoperation>("origin-1");
    teleoperation->startSignaling();

    Action *streamVideoAction = new StreamVideoAction(teleoperation);
    Action *sendCounterAction = new SendCounterAction(teleoperation);
    Action *addClientsAction = new AddClientsAction(teleoperation);

    std::thread streamVideoThread([streamVideoAction]() { streamVideoAction->execute(); });
    std::thread sendCounterThread([sendCounterAction]() { sendCounterAction->execute(); });
    std::thread addClientsThread([addClientsAction]() { addClientsAction->execute(); });

    streamVideoThread.join();
    sendCounterThread.join();
    addClientsThread.join();

    teleoperation->close();

    return 0;

} catch (const std::exception &e) {
    std::cout << "Error: " << e.what() << std::endl;
    return -1;
}
