#ifndef BROADCAST_IP_H
#define BROADCAST_IP_H

#include "lwip/opt.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include <string.h>

void broadcast_ip(uint16_t port);

#endif // BROADCAST_IP_H