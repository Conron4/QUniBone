/* dhq11.cpp: DHQ11-compatible QBUS serial line controller with telnet forwarding */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include "dhq11.hpp"
#include "logger.hpp"
#include "qunibusadapter.hpp"
#include "qunibus.h"

#define DHQ11_LINE_COUNT 8
#define DHQ11_CSR_LINE_MASK 0000007
#define DHQ11_CSR_RXIE 0000100
#define DHQ11_CSR_TXIE 0000200
#define DHQ11_CSR_RXRDY 0000400
#define DHQ11_CSR_TXRDY 0001000

static void set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
        (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void close_fd(int &fd)
{
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

dhq11_c::dhq11_c() : qunibusdevice_c()
{
    set_workers_count(1);

    name.value = "DHQ11";
    type_name.value = "dhq11_c";
    log_label = "dhq11";

    set_default_bus_params(0776500, 1, 0300, 4);

    register_count = dhq11_idx_count;

    reg_csr = &(registers[dhq11_idx_csr]);
    strcpy(reg_csr->name, "CSR");
    reg_csr->active_on_dati = false;
    reg_csr->active_on_dato = true;
    reg_csr->reset_value = DHQ11_CSR_TXRDY;
    reg_csr->writable_bits = 0xffff;

    reg_rbuf = &(registers[dhq11_idx_rbuf]);
    strcpy(reg_rbuf->name, "RBUF");
    reg_rbuf->active_on_dati = true;
    reg_rbuf->active_on_dato = true;
    reg_rbuf->reset_value = 0;
    reg_rbuf->writable_bits = 0x0000;

    reg_xbuf = &(registers[dhq11_idx_xbuf]);
    strcpy(reg_xbuf->name, "XBUF");
    reg_xbuf->active_on_dati = false;
    reg_xbuf->active_on_dato = true;
    reg_xbuf->reset_value = 0;
    reg_xbuf->writable_bits = 0x00ff;

    telnet_port_base.value = 20000;

    reset_state();
}

dhq11_c::~dhq11_c()
{
}

bool dhq11_c::setup_line(unsigned line_index, unsigned port_base)
{
    line_state_t &line = lines[line_index];
    line.tcp_port = port_base + line_index;

    line.line_index = line_index;
    line.listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (line.listen_fd < 0) {
        ERROR("DHQ11 line %u: socket() failed: %s", line_index, strerror(errno));
        return false;
    }

    int reuse = 1;
    (void)setsockopt(line.listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    set_nonblocking(line.listen_fd);

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)line.tcp_port);

    if (bind(line.listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ERROR("DHQ11 line %u: bind(:%u) failed: %s", line_index, line.tcp_port, strerror(errno));
        close_fd(line.listen_fd);
        return false;
    }

    if (listen(line.listen_fd, 1) < 0) {
        ERROR("DHQ11 line %u: listen(:%u) failed: %s", line_index, line.tcp_port, strerror(errno));
        close_fd(line.listen_fd);
        return false;
    }

    INFO("DHQ11 line %u listening on TCP port %u", line_index, line.tcp_port);
    return true;
}

void dhq11_c::close_line(line_state_t &line)
{
    close_fd(line.client_fd);
    close_fd(line.listen_fd);
    line.connected = false;
    line.telnet_iac = false;
    line.telnet_skip_option = false;
    line.rx_queue.clear();
    line.tx_queue.clear();

    for (auto it = rx_queue.begin(); it != rx_queue.end(); ) {
        if (it->line == line.line_index) {
            it = rx_queue.erase(it);
        } else {
            ++it;
        }
    }
}

void dhq11_c::close_all_lines(void)
{
    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i)
        close_line(lines[i]);
}

bool dhq11_c::open_listeners(void)
{
    bool ok = true;
    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i)
        ok = setup_line(i, telnet_port_base.value) && ok;
    if (!ok)
        close_all_lines();
    return ok;
}

void dhq11_c::queue_rx_byte(unsigned line_index, uint8_t value)
{
    line_state_t &line = lines[line_index];
    rx_entry_t entry;
    entry.line = (uint8_t)line_index;
    entry.value = value;
    rx_queue.push_back(entry);
    line.rx_queue.push_back(value);
    if (!rx_done) {
        rx_line = entry.line;
        rx_buffer = entry.value;
        rx_done = true;
        update_rbuf();
    }
}

void dhq11_c::queue_tx_byte(unsigned line_index, uint8_t value)
{
    line_state_t &line = lines[line_index];
    line.tx_queue.push_back(value);
}

void dhq11_c::print_line_status(void)
{
    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i) {
        line_state_t &line = lines[i];
        printf("DHQ11 line %u: tcp=%u listen=%s client=%s rx=%u tx=%u\n",
               i,
               line.tcp_port,
               line.listen_fd >= 0 ? "up" : "down",
               line.client_fd >= 0 ? "up" : "down",
               (unsigned)line.rx_queue.size(),
               (unsigned)line.tx_queue.size());
    }
}

void dhq11_c::update_csr(void)
{
    uint16_t value = (uint16_t)(selected_line & DHQ11_CSR_LINE_MASK);
    tx_ready = lines[selected_line].tx_queue.empty();
    if (rx_intr_enable)
        value |= DHQ11_CSR_RXIE;
    if (tx_intr_enable)
        value |= DHQ11_CSR_TXIE;
    if (rx_done)
        value |= DHQ11_CSR_RXRDY;
    if (tx_ready)
        value |= DHQ11_CSR_TXRDY;

    bool interrupt_condition = get_intr_condition();
    switch (intr_request.edge_detect(interrupt_condition)) {
    case intr_request_c::INTERRUPT_EDGE_RAISING:
        qunibusadapter->INTR(intr_request, reg_csr, value);
        break;
    case intr_request_c::INTERRUPT_EDGE_FALLING:
        qunibusadapter->cancel_INTR(intr_request);
        set_register_dati_value(reg_csr, value, __func__);
        break;
    default:
        set_register_dati_value(reg_csr, value, __func__);
        break;
    }
}

void dhq11_c::update_rbuf(void)
{
    if (!rx_queue.empty()) {
        rx_line = rx_queue.front().line;
        rx_buffer = rx_queue.front().value;
        rx_done = true;
    }
    uint16_t value = rx_buffer | ((uint16_t)rx_line << 8);
    set_register_dati_value(reg_rbuf, value, __func__);
}

void dhq11_c::update_xbuf(void)
{
    uint16_t value = get_register_dato_value(reg_xbuf) & 0x00ff;
    set_register_dati_value(reg_xbuf, value, __func__);
}

bool dhq11_c::get_intr_condition(void)
{
    return (rx_done && rx_intr_enable) || (tx_ready && tx_intr_enable);
}

void dhq11_c::reset_state(void)
{
    reset_unibus_registers();
    qunibusadapter->cancel_INTR(intr_request);
    rx_queue.clear();
    selected_line = 0;
    rx_line = 0;
    rx_buffer = 0;
    rx_done = false;
    rx_intr_enable = false;
    tx_ready = true;
    tx_intr_enable = false;
    intr_request.edge_detect_reset();
    update_csr();
    update_rbuf();
    update_xbuf();
}

void dhq11_c::reset(void)
{
    pthread_mutex_lock(&on_after_register_access_mutex);
    reset_state();
    pthread_mutex_unlock(&on_after_register_access_mutex);
}

bool dhq11_c::on_before_install(void)
{
    close_all_lines();
    pthread_mutex_lock(&on_after_register_access_mutex);
    reset_state();
    bool ok = open_listeners();
    pthread_mutex_unlock(&on_after_register_access_mutex);
    return ok;
}

void dhq11_c::on_after_uninstall(void)
{
    reset();
    close_all_lines();
}

bool dhq11_c::on_param_changed(parameter_c *param)
{
    if (param == &priority_slot) {
        intr_request.set_priority_slot(priority_slot.new_value);
    } else if (param == &intr_vector) {
        intr_request.set_vector(intr_vector.new_value);
    } else if (param == &intr_level) {
        intr_request.set_level(intr_level.new_value);
    }
    return qunibusdevice_c::on_param_changed(param);
}

void dhq11_c::eval_csr_dato_value(void)
{
    uint16_t value = get_register_dato_value(reg_csr);
    selected_line = value & DHQ11_CSR_LINE_MASK;
    rx_intr_enable = !!(value & DHQ11_CSR_RXIE);
    tx_intr_enable = !!(value & DHQ11_CSR_TXIE);
    update_csr();
}

void dhq11_c::eval_xbuf_dato_value(void)
{
    queue_tx_byte(selected_line, get_register_dato_value(reg_xbuf) & 0x00ff);
    update_csr();
}

void dhq11_c::on_after_register_access(qunibusdevice_register_t *device_reg,
                                       uint8_t unibus_control,
                                       DATO_ACCESS access)
{
    UNUSED(access);

    if (qunibusadapter->line_INIT)
        return;

    pthread_mutex_lock(&on_after_register_access_mutex);

    switch (device_reg->index) {
    case dhq11_idx_csr:
        if (unibus_control == QUNIBUS_CYCLE_DATO)
            eval_csr_dato_value();
        else
            update_csr();
        break;
    case dhq11_idx_rbuf:
        if (unibus_control == QUNIBUS_CYCLE_DATI) {
            if (!rx_queue.empty()) {
                uint8_t line = rx_queue.front().line;
                rx_queue.pop_front();
                if (!lines[line].rx_queue.empty())
                    lines[line].rx_queue.pop_front();
            }
            if (!rx_queue.empty()) {
                rx_line = rx_queue.front().line;
                rx_buffer = rx_queue.front().value;
                rx_done = true;
            } else {
                rx_done = false;
            }
            update_csr();
            update_rbuf();
        }
        break;
    case dhq11_idx_xbuf:
        if (unibus_control == QUNIBUS_CYCLE_DATO)
            eval_xbuf_dato_value();
        break;
    default:
        break;
    }

    pthread_mutex_unlock(&on_after_register_access_mutex);
}

void dhq11_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge)
{
    UNUSED(aclo_edge);
    UNUSED(dclo_edge);
    reset();
}

void dhq11_c::on_init_changed(void)
{
    if (init_asserted)
        reset();
}

void dhq11_c::service_listener(line_state_t &line, unsigned line_index)
{
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd = accept(line.listen_fd, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
    if (client_fd < 0)
        return;

    set_nonblocking(client_fd);

    if (line.client_fd >= 0)
        close_fd(line.client_fd);

    line.client_fd = client_fd;
    line.connected = true;
    line.telnet_iac = false;
    line.telnet_skip_option = false;

    INFO("DHQ11 line %u connected on TCP port %u", line_index, line.tcp_port);
}

void dhq11_c::service_client_rx(line_state_t &line, unsigned line_index)
{
    if (line.client_fd < 0)
        return;

    uint8_t buffer[128];
    ssize_t count = recv(line.client_fd, buffer, sizeof(buffer), 0);
    if (count == 0) {
        INFO("DHQ11 line %u disconnected", line_index);
        close_fd(line.client_fd);
        line.connected = false;
        line.telnet_iac = false;
        line.telnet_skip_option = false;
        if (line.tx_queue.empty()) {
            tx_ready = true;
            update_csr();
        }
        return;
    }
    if (count < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            INFO("DHQ11 line %u recv() failed: %s", line_index, strerror(errno));
        return;
    }

    for (ssize_t i = 0; i < count; ++i) {
        uint8_t ch = buffer[i];
        if (line.telnet_skip_option) {
            line.telnet_skip_option = false;
            continue;
        }
        if (line.telnet_iac) {
            line.telnet_iac = false;
            if (ch == 255) {
                queue_rx_byte(line_index, 255);
            } else if (ch == 251 || ch == 252 || ch == 253 || ch == 254) {
                line.telnet_skip_option = true;
            }
            continue;
        }
        if (ch == 255) {
            line.telnet_iac = true;
            continue;
        }
        queue_rx_byte(line_index, ch);
    }
}

void dhq11_c::service_client_tx(line_state_t &line, unsigned line_index)
{
    if (line.client_fd < 0 || line.tx_queue.empty())
        return;

    uint8_t value = line.tx_queue.front();
    ssize_t count = send(line.client_fd, &value, 1, 0);
    if (count == 1) {
        line.tx_queue.pop_front();
        update_csr();
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        INFO("DHQ11 line %u send() failed: %s", line_index, strerror(errno));
        close_fd(line.client_fd);
        line.connected = false;
        update_csr();
    }
}

void dhq11_c::service_lines(void)
{
    fd_set read_fds;
    fd_set write_fds;
    int max_fd = -1;

    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);

    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i) {
        line_state_t &line = lines[i];
        if (line.listen_fd >= 0) {
            FD_SET(line.listen_fd, &read_fds);
            if (line.listen_fd > max_fd)
                max_fd = line.listen_fd;
        }
        if (line.client_fd >= 0) {
            FD_SET(line.client_fd, &read_fds);
            if (!line.tx_queue.empty())
                FD_SET(line.client_fd, &write_fds);
            if (line.client_fd > max_fd)
                max_fd = line.client_fd;
        }
    }

    if (max_fd < 0)
        return;

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 25000;

    int status = select(max_fd + 1, &read_fds, &write_fds, nullptr, &timeout);
    if (status < 0) {
        if (errno != EINTR)
            WARNING("DHQ11 select() failed: %s", strerror(errno));
        return;
    }

    pthread_mutex_lock(&on_after_register_access_mutex);

    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i) {
        line_state_t &line = lines[i];
        if (line.listen_fd >= 0 && FD_ISSET(line.listen_fd, &read_fds))
            service_listener(line, i);
        if (line.client_fd >= 0 && FD_ISSET(line.client_fd, &read_fds))
            service_client_rx(line, i);
        if (line.client_fd >= 0 && FD_ISSET(line.client_fd, &write_fds))
            service_client_tx(line, i);
    }

    if (rx_done || get_intr_condition())
        update_csr();

    pthread_mutex_unlock(&on_after_register_access_mutex);
}

void dhq11_c::worker(unsigned instance)
{
    UNUSED(instance);
    worker_init_realtime_priority(rt_device);

    while (!workers_terminate)
        service_lines();
}
