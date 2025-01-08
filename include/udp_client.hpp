#pragma once

#include "sockets.hpp"

class UDPClient : public Socket
{
public:
    UDPClient(u_short port = 8000, const std::string &ip_address = "127.0.0.1") : Socket(SocketType::TYPE_DGRAM)
    {
        set_address(ip_address);
        set_port(port);
        std::cout << "UDP Client created." << std::endl;
    }

    ssize_t send_message(const char *message, size_t length)
    {
        return sendto(m_socket, message, length, 0, reinterpret_cast<sockaddr *>(&m_addr), sizeof(m_addr));
    }
};