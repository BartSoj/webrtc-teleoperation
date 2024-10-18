#include <iostream>
#include <stdexcept>

#include "teleop_actions/action.hpp"
#include "teleop_actions/add_clients_action.hpp"
#include "teleop_actions/send_counter_action.hpp"
#include "teleop_actions/stream_video_action.hpp"
#include "teleop_actions/stream_video_cv_action.hpp"
#include "teleop_client/teleoperation.hpp"

int main(int argc, char **argv)
try
{
    auto teleoperation = std::make_shared<Teleoperation>("pc-client");
    teleoperation->startSignaling();

    Action *streamVideoAction = new StreamVideoCvAction(teleoperation);
    Action *sendCounterAction = new SendCounterAction(teleoperation);
    Action *addClientsAction = new AddClientsAction(teleoperation);

    std::thread streamVideoThread([streamVideoAction]() { streamVideoAction->execute(); });
    std::thread sendCounterThread([sendCounterAction]() { sendCounterAction->execute(); });
    std::thread addClientsThread([addClientsAction]() { addClientsAction->execute(); });

    streamVideoThread.join();
    sendCounterThread.join();
    addClientsThread.join();

    return 0;
}
catch(const std::exception &e)
{
    std::cout << "Error: " << e.what() << std::endl;
    return -1;
}
