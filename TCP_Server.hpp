/*include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/ip_addr.h"
#include "lwip/tcp.h"

#define WIFI_SSID "simp_wifi"
#define WIFI_PASSWORD "1den9akadar"
#define PICO_CYW43_ARCH_POLL 0


void tcp_server_connection_closed_callback(void* arg, err_t err);
err_t tcp_server_send(struct tcp_pcb* pcb, char* response);
err_t tcp_server_receive_callback(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err);
err_t tcp_server_accept_callback(void* arg, struct tcp_pcb* pcb, err_t err);
*/

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <stdio.h>
#include <string>
#include <functional>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/opt.h"
#include "lwip/tcp.h"

#include "BroadcastIP.hpp"

#define PICO_CYW43_ARCH_POLL 0


class TCP_Server {
public:
    TCP_Server(uint16_t port);
    ~TCP_Server();

    void start();
    void stop();
    bool is_running();

    struct tcp_pcb* listener;
    static tcp_pcb* pcb_send;
    static bool is_connected;
    static void add_cb(std::function<void(std::string)> new_cb);
    static bool is_cb_defined;
    static std::function<void(std::string)> cb;
    err_t send(const char* response);

private:
    static err_t accept_callback(void* arg, struct tcp_pcb* newpcb, err_t err);
    static err_t receive_callback(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err);
    static err_t connection_poll_callback(void* arg, struct tcp_pcb* pcb);
    static void connection_closed_callback(void* arg, err_t err);

    bool running;
};

#endif // TCP_SERVER_H