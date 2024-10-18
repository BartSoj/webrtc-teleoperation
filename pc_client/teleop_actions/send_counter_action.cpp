#include "send_counter_action.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

using namespace std::chrono_literals;
using json = nlohmann::json;

void SendCounterAction::init() { counter_ = 0; }

bool SendCounterAction::loop()
{
    json logMessage;
    logMessage["type"] = "log";
    logMessage["message"] = "Counter: " + std::to_string(counter_);

    std::string jsonString = logMessage.dump();

    std::cout << "Sending message: " << jsonString << std::endl;
    teleoperation->broadcastMessage(jsonString);

    std::this_thread::sleep_for(1s);
    counter_++;

    if(counter_ >= 1000) return false;
    return true;
}
