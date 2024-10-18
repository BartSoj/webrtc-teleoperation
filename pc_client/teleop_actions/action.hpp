#pragma once

#include "teleop_client/teleoperation.hpp"

class Action
{
public:
    Action(shared_ptr<Teleoperation> teleoperation);

    void execute();

protected:
    virtual void init() {}

    virtual bool loop() = 0;

    virtual void close() {}

    shared_ptr<Teleoperation> teleoperation;
};