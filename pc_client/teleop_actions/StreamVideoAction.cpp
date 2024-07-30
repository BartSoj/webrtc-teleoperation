#include "StreamVideoAction.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <iostream>

void StreamVideoAction::init()
{
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(6000);

    if(bind(sock, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("Failed to bind UDP socket on 127.0.0.1:6000");

    int rcvBufSize = 212992;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&rcvBufSize), sizeof(rcvBufSize));
}

bool StreamVideoAction::loop()
{
    if((len = recv(sock, buffer, BUFFER_SIZE, 0)) >= 0)
    {
        if(static_cast<long unsigned int>(len) < sizeof(rtc::RtpHeader)) return true;

        auto rtp = reinterpret_cast<rtc::RtpHeader *>(buffer);
        rtp->setSsrc(SSRC);

        for(const auto &[id, pc] : teleoperation->getPeerConnectionMap())
        {
            pc->sendVideo(reinterpret_cast<const std::byte *>(buffer), len, 0);
        }
        return true;
    }
    return false;
}