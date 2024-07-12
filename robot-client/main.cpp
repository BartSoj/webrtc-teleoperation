#include "Teleoperation.hpp"

#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc, char **argv) try {
    Teleoperation teleoperation("origin-1");
    teleoperation.startSignaling();

    std::thread streamVideoThread(&Teleoperation::streamVideoLoop, &teleoperation);
    std::thread sendCounterThread(&Teleoperation::sendCounterLoop, &teleoperation);
    std::thread addClientsThread(&Teleoperation::addClientsLoop, &teleoperation);

    streamVideoThread.join();
    sendCounterThread.join();
    addClientsThread.join();

    teleoperation.close();

    return 0;

} catch (const std::exception &e) {
    std::cout << "Error: " << e.what() << std::endl;
    return -1;
}
