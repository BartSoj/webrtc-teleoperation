#pragma once

#include "action.hpp"

class SendCounterAction : public Action
{
public:
    using Action::Action;

protected:
    void init() override;

    bool loop() override;

private:
    int counter_;
};