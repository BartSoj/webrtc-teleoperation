#ifndef LIBDATACHANNEL_APP_ADDCLIENTS_H
#define LIBDATACHANNEL_APP_ADDCLIENTS_H

#include "Action.hpp"


class AddClientsAction : public Action {
public:
    using Action::Action;

protected:
    bool loop() override;

};


#endif //LIBDATACHANNEL_APP_ADDCLIENTS_H
