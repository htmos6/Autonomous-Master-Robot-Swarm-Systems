#include "TCP_Server.hpp"

/*void tcp_server_connection_closed_callback(void* arg, err_t err) {
    struct tcp_pcb* pcb = (struct tcp_pcb*)arg;
    if (pcb != NULL) {
        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_poll(pcb, NULL, 0);
        tcp_close(pcb);
    }

    // Start listening for new connections again
    struct tcp_pcb* listener = tcp_listen((struct tcp_pcb*)arg);
    tcp_accept(listener, tcp_server_accept_callback);
}

err_t tcp_server_send(struct tcp_pcb* pcb, char* response){
    //char* response = "Hello, client!";
    int response_len = strlen(response);
    struct pbuf* resp_buf = pbuf_alloc(PBUF_TRANSPORT, response_len, PBUF_RAM);
    memcpy(resp_buf->payload, response, response_len);
    tcp_write(pcb, resp_buf->payload, resp_buf->tot_len, TCP_WRITE_FLAG_COPY);
    pbuf_free(resp_buf);

    return ERR_OK;
}

err_t tcp_server_receive_callback(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err) {
    if (err == ERR_OK && p != NULL) {
        // Print received data to the console
        printf("Received data: %.*s\n", p->tot_len, (char*)p->payload);

        char out_str[40];
        sprintf(out_str, "test");
        tcp_server_send(pcb, out_str);

        tcp_recved(pcb, p->tot_len);
        pbuf_free(p);
    } else {
        tcp_close(pcb);
        if (p != NULL){
            pbuf_free(p);
        }
    }

    return ERR_OK;
}

err_t tcp_server_accept_callback(void* arg, struct tcp_pcb* pcb, err_t err) {
    if (err == ERR_OK && pcb != NULL) {
        // Set up a receive callback function for the new connection
        tcp_recv(pcb, tcp_server_receive_callback);

        // Set up a connection closed callback function for the new connection
        tcp_arg(pcb, pcb);
        tcp_err(pcb, tcp_server_connection_closed_callback);

        char out_str[40];
        sprintf(out_str, "Wellcome to Pico W!");
        tcp_server_send(pcb, out_str);
    } else {
        tcp_close(pcb);
    }
    return ERR_OK;
}*/

bool TCP_Server::is_connected = false;
bool TCP_Server::is_cb_defined = false;
tcp_pcb* TCP_Server::pcb_send = nullptr;
std::function<void(std::string)> TCP_Server::cb;


TCP_Server::TCP_Server(uint16_t port) {
    is_connected = false;
    listener = tcp_new();
    tcp_bind(listener, IP_ADDR_ANY, port);
    listener = tcp_listen(listener);
    tcp_accept(listener, accept_callback);
    running = false;
}

TCP_Server::~TCP_Server() {
    stop();
}

void TCP_Server::start() {
    running = true;

    while(!is_connected){
        broadcast_ip(12345);
        sleep_ms(1000);
    }

    printf("Client connected!\n");
}

void TCP_Server::stop() {
    if (listener != NULL) {
        //tcp_abort(listener);
        tcp_close(listener);
        listener = NULL;
    }
    running = false;
}

bool TCP_Server::is_running() {
    return running;
}

void TCP_Server::add_cb(std::function<void(std::string)> new_cb){
    cb = new_cb;
    is_cb_defined = true;
}

err_t TCP_Server::accept_callback(void* arg, struct tcp_pcb* newpcb, err_t err) {
    if (err == ERR_OK && newpcb != NULL) {
        tcp_arg(newpcb, NULL);
        tcp_recv(newpcb, receive_callback);
        tcp_err(newpcb, connection_closed_callback);
        tcp_poll(newpcb, connection_poll_callback, 4);

        const char* welcome_message = "Welcome to my TCP server!\r\n";
        tcp_write(newpcb, welcome_message, strlen(welcome_message), TCP_WRITE_FLAG_COPY);
        pcb_send = newpcb;
        is_connected = true;

    } else {
        tcp_close(newpcb);
    }

    return ERR_OK;
}

err_t TCP_Server::connection_poll_callback(void* arg, struct tcp_pcb* pcb) {
    // Check if the connection has been closed by the remote end
    //printf("%d\n", pcb->state);
    if (pcb->state == CLOSED || pcb->state == TIME_WAIT || pcb->state == CLOSE_WAIT) {
        printf("Connection closed\n");
        is_connected = false;
        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_poll(pcb, NULL, 0);
        tcp_close(pcb);
        return ERR_OK;
    }
    return ERR_OK;
}

err_t TCP_Server::receive_callback(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err) {
    if (err == ERR_OK && p != NULL) {
        char* data = new char[p->tot_len + 1];
        memcpy(data, p->payload, p->tot_len);
        data[p->tot_len] = '\0';
        printf("Received data: %s\n", data);

        if (is_cb_defined){
            cb(std::string(data));
        }

        delete[] data;

        // Echo the received data back to the client
        //tcp_write(pcb, p->payload, p->tot_len, TCP_WRITE_FLAG_COPY);

        // Free the receive buffer
        pbuf_free(p);

        return ERR_OK;
    } else {
        if (p != NULL) {
            pbuf_free(p);
        }
        return err;
    }
}

void TCP_Server::connection_closed_callback(void* arg, err_t err) {
    struct tcp_pcb* pcb = (struct tcp_pcb*)arg;
    if (pcb != NULL) {
        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_poll(pcb, NULL, 0);
        tcp_close(pcb);
        is_connected = false;

        // Start listening for new connections again
        struct tcp_pcb* listener = tcp_listen((struct tcp_pcb*)arg);
        tcp_accept(listener, accept_callback);
    }
}

err_t TCP_Server::send(const char* response){
    //int response_len = strlen(response);
    tcp_write(TCP_Server::pcb_send, response, strlen(response), TCP_WRITE_FLAG_COPY);
    /*struct pbuf* resp_buf = pbuf_alloc(PBUF_TRANSPORT, response_len, PBUF_RAM);
    memcpy(resp_buf->payload, response, response_len);
    tcp_write(listener, resp_buf->payload, resp_buf->tot_len, TCP_WRITE_FLAG_COPY);
    pbuf_free(resp_buf);*/

    return ERR_OK;
}