#include "SendCounterAction.hpp"

#include <iostream>
#include <thread>

using namespace std::chrono_literals;

void SendCounterAction::init() {
    counter = 0;
}

bool SendCounterAction::loop() {
    std::string message = "Counter: " + std::to_string(counter);
    std::cout << "Sending message " << counter << std::endl;
    teleoperation->broadcastMessage(message);
    std::this_thread::sleep_for(1s);
    counter++;

    if (counter >= 1000)
        return false;
    return true;
}
