#ifndef LIBDATACHANNEL_APP_STREAMVIDEOACTION_H
#define LIBDATACHANNEL_APP_STREAMVIDEOACTION_H

#include "Action.hpp"

typedef int SOCKET;

class StreamVideoAction : public Action
{
public:
    using Action::Action;

protected:
    void init() override;

    bool loop() override;

    const rtc::SSRC SSRC = 42;
    const static int BUFFER_SIZE = 2048;
    SOCKET sock;
    char buffer[BUFFER_SIZE];
    long int len;
};

#endif  // LIBDATACHANNEL_APP_STREAMVIDEOACTION_H
