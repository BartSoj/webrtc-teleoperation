#include "Action.hpp"

#include <utility>

Action::Action(shared_ptr<Teleoperation> teleoperation) : teleoperation(std::move(teleoperation)) {}

void Action::execute()
{
    init();
    bool proceed = true;
    while(proceed)
    {
        proceed = loop();
    }
    cleanup();
}