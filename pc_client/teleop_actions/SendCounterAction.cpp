#include "SendCounterAction.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

using namespace std::chrono_literals;
using json = nlohmann::json;

void SendCounterAction::init() { counter = 0; }

bool SendCounterAction::loop()
{
    json logMessage;
    logMessage["type"] = "log";
    logMessage["message"] = "Counter: " + std::to_string(counter);

    std::string jsonString = logMessage.dump();

    std::cout << "Sending message: " << jsonString << std::endl;
    teleoperation->broadcastMessage(jsonString);

    std::this_thread::sleep_for(1s);
    counter++;

    if(counter >= 1000) return false;
    return true;
}
