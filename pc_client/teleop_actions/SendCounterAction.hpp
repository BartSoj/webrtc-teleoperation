#ifndef LIBDATACHANNEL_APP_SENDCOUNTERACTION_H
#define LIBDATACHANNEL_APP_SENDCOUNTERACTION_H

#include "Action.hpp"

class SendCounterAction : public Action
{
public:
    using Action::Action;

protected:
    void init() override;

    bool loop() override;

private:
    int counter;
};

#endif  // LIBDATACHANNEL_APP_SENDCOUNTERACTION_H
