#pragma once

#include <mbed.h>

#include "netsocket/NetworkInterface.h"
#include "netsocket/TCPSocket.h"

class MbedMqttNetwork {
    public:
    explicit MbedMqttNetwork(NetworkInterface* iface)
    : network(iface) {}

    int connect(const char* host, int port) {
        socket.close();

        nsapi_error_t err = socket.open(network);
        if (err != NSAPI_ERROR_OK) {
            return err;
        }

        socket.set_timeout(SOCKET_TIMEOUT_MS);

        SocketAddress address;
        err = network->gethostbyname(host, &address);
        if (err != NSAPI_ERROR_OK) {
            socket.close();
            return err;
        }

        address.set_port(port);
        err = socket.connect(address);
        if (err != NSAPI_ERROR_OK) {
            socket.close();
        }

        return err;
    }

    int read(unsigned char* buffer, int len, int timeout) {
        socket.set_timeout(timeout);
        mbed::Timer timer;
        timer.start();

        int received = 0;
        while (received < len && elapsed_ms(timer) < timeout) {
            const int ret = socket.recv(buffer + received, len - received);
            if (ret > 0) {
                received += ret;
            } else if (ret == NSAPI_ERROR_WOULD_BLOCK || ret == NSAPI_ERROR_TIMEOUT) {
                break;
            } else if (ret == 0) {
                break;
            } else {
                return ret;
            }
        }

        return received;
    }

    int write(unsigned char* buffer, int len, int timeout) {
        socket.set_timeout(timeout);
        mbed::Timer timer;
        timer.start();

        int written = 0;
        while (written < len && elapsed_ms(timer) < timeout) {
            const int ret = socket.send(buffer + written, len - written);
            if (ret > 0) {
                written += ret;
            } else if (ret == NSAPI_ERROR_WOULD_BLOCK || ret == NSAPI_ERROR_TIMEOUT) {
                break;
            } else {
                return ret;
            }
        }

        return written;
    }

    int disconnect() {
        return socket.close();
    }

    private:
    static constexpr uint32_t SOCKET_TIMEOUT_MS = 5000;

    static int elapsed_ms(const mbed::Timer& timer) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
    }

    NetworkInterface* network;
    TCPSocket socket;
};
