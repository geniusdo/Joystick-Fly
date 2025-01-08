#pragma once

#include <string>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <unistd.h>    

#define INVALID_SOCKET (SOCKET)(~0)
#define SOCKET_ERROR (-1)
typedef int SOCKET;

enum err_code
{
    socket_bind_err = 3,
    socket_accept_err = 4,
    connection_err = 5,
    message_send_err = 6,
    receive_err = 7,
};

class Socket
{
public:
    enum class SocketType
    {
        TYPE_STREAM = SOCK_STREAM,
        TYPE_DGRAM = SOCK_DGRAM
    };

protected:
    explicit Socket(SocketType socket_type) : m_socket(), m_addr()
    {

        m_socket = socket(AF_INET, static_cast<int>(socket_type), 0);
        if (m_socket == INVALID_SOCKET)
        {
            throw std::runtime_error("Could not create socket");
        }

        m_addr.sin_family = AF_INET;
    }
    ~Socket() = default;

    SOCKET m_socket;
    sockaddr_in m_addr;

    void set_port(u_short port)
    {
        m_addr.sin_port = htons(port);
    }

    int set_address(const std::string &ip_address)
    {
        return inet_pton(AF_INET, ip_address.c_str(), &m_addr.sin_addr);
    }
};
