#pragma once

#include "action.hpp"

class AddClientsAction : public Action
{
public:
    using Action::Action;

protected:
    bool loop() override;
};