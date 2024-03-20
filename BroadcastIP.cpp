#include "BroadcastIP.hpp"

void broadcast_ip(uint16_t port) {
    struct udp_pcb* pcb = udp_new();

    // Bind the UDP socket to a specific port
    udp_bind(pcb, IP_ADDR_ANY, port);

    // Set the broadcast IP address and port
    ip_addr_t broadcast_addr;
    IP4_ADDR(&broadcast_addr, 255, 255, 255, 255);
    uint16_t broadcast_port = port;

    // Construct the message to be broadcasted (IP address as a string)
    char ip_str[16];
    ip4_addr_t ip_addr = *(ip_2_ip4(&netif_default->ip_addr));
    sprintf(ip_str, "%d.%d.%d.%d", ip4_addr1(&ip_addr), ip4_addr2(&ip_addr), ip4_addr3(&ip_addr), ip4_addr4(&ip_addr));
    const char* message = ip_str;

    // Create a new PBUF to hold the message data
    struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, strlen(message), PBUF_RAM);
    memcpy(p->payload, message, strlen(message));

    // Send the UDP packet to the broadcast address and port
    udp_sendto(pcb, p, &broadcast_addr, broadcast_port);

    // Free the PBUF
    pbuf_free(p);

    // Close the UDP socket
    udp_remove(pcb);
}