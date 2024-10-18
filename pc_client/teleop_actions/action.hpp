#ifndef LIBDATACHANNEL_APP_ACTION_H
#define LIBDATACHANNEL_APP_ACTION_H

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

#endif  // LIBDATACHANNEL_APP_ACTION_H