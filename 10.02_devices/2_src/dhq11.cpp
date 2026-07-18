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
#include <vector>

#include "dhq11.hpp"
#include "logger.hpp"
#include "qunibusadapter.hpp"
#include "qunibus.h"

#define DHQ11_LINE_COUNT 8

#define DHQ11_CSR_DHV11_TXA       0x8000
#define DHQ11_CSR_DHV11_TXIE      0x4000
#define DHQ11_CSR_DHV11_DF        0x2000
#define DHQ11_CSR_DHV11_TDE       0x1000
#define DHQ11_CSR_DHV11_TX_LINE   0x0400
#define DHQ11_CSR_DHV11_RDA       0x0200
#define DHQ11_CSR_DHV11_RXIE      0x0100
#define DHQ11_CSR_DHV11_MR        0x0080
#define DHQ11_CSR_DHV11_LINE_MASK 0x003f

#define DHQ11_CSR_DHU11_TXA       0x8000
#define DHQ11_CSR_DHU11_TXIE      0x4000
#define DHQ11_CSR_DHU11_DF        0x2000
#define DHQ11_CSR_DHU11_TDE       0x1000
#define DHQ11_CSR_DHU11_TX_LINE   0x0800
#define DHQ11_CSR_DHU11_RDA       0x0400
#define DHQ11_CSR_DHU11_RXIE      0x0200
#define DHQ11_CSR_DHU11_MR        0x0100
#define DHQ11_CSR_DHU11_SKIP      0x0080
#define DHQ11_CSR_DHU11_LINE_MASK 0x007f

#define DHQ11_STAT_DSR            0x8000
#define DHQ11_STAT_RI             0x2000
#define DHQ11_STAT_DCD            0x1000
#define DHQ11_STAT_CTS            0x0800
#define DHQ11_STAT_DHU            0x0080

#define DHQ11_CTRL_RTS            0x4000
#define DHQ11_CTRL_DTR            0x1000
#define DHQ11_CTRL_LTYP           0x0800
#define DHQ11_CTRL_MAINT          0x0400
#define DHQ11_CTRL_FXOFF          0x0200
#define DHQ11_CTRL_OAUTO          0x0100
#define DHQ11_CTRL_BREAK          0x0080
#define DHQ11_CTRL_RXE            0x0040
#define DHQ11_CTRL_IAF            0x0020
#define DHQ11_CTRL_TXA            0x0010

#define DHQ11_LPR_BAUD_MASK       0xf000

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

    set_default_bus_params(0776600, 18, 0300, 4);

    register_count = dhq11_idx_count;

    reg_csr = &(registers[dhq11_idx_csr]);
    strcpy(reg_csr->name, "CSR");
    reg_csr->active_on_dati = false;
    reg_csr->active_on_dato = true;
    reg_csr->reset_value = 0;
    reg_csr->writable_bits = 0xffff;

    reg_rbuf = &(registers[dhq11_idx_rbuf]);
    strcpy(reg_rbuf->name, "RBUF");
    reg_rbuf->active_on_dati = true;
    reg_rbuf->active_on_dato = true;
    reg_rbuf->reset_value = 0;
    reg_rbuf->writable_bits = 0x0000;

    reg_lpr = &(registers[dhq11_idx_lpr]);
    strcpy(reg_lpr->name, "LPR");
    reg_lpr->active_on_dati = false;
    reg_lpr->active_on_dato = true;
    reg_lpr->reset_value = 0;
    reg_lpr->writable_bits = 0xffff;

    reg_stat = &(registers[dhq11_idx_stat]);
    strcpy(reg_stat->name, "STAT");
    reg_stat->active_on_dati = true;
    reg_stat->active_on_dato = true;
    reg_stat->reset_value = 0;
    reg_stat->writable_bits = 0x0000;

    reg_ctrl = &(registers[dhq11_idx_ctrl]);
    strcpy(reg_ctrl->name, "CTRL");
    reg_ctrl->active_on_dati = false;
    reg_ctrl->active_on_dato = true;
    reg_ctrl->reset_value = 0;
    reg_ctrl->writable_bits = 0xffff;

    reg_tbadl = &(registers[dhq11_idx_tbadl]);
    strcpy(reg_tbadl->name, "TBADL");
    reg_tbadl->active_on_dati = true;
    reg_tbadl->active_on_dato = true;
    reg_tbadl->reset_value = 0;
    reg_tbadl->writable_bits = 0xffff;

    reg_tbadh = &(registers[dhq11_idx_tbadh]);
    strcpy(reg_tbadh->name, "TBADH");
    reg_tbadh->active_on_dati = true;
    reg_tbadh->active_on_dato = true;
    reg_tbadh->reset_value = 0;
    reg_tbadh->writable_bits = 0xffff;

    reg_tbct = &(registers[dhq11_idx_tbct]);
    strcpy(reg_tbct->name, "TBCT");
    reg_tbct->active_on_dati = true;
    reg_tbct->active_on_dato = true;
    reg_tbct->reset_value = 0;
    reg_tbct->writable_bits = 0xffff;

    compatibility.value = "DHV11";
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

bool dhq11_c::is_dhu11_mode(void) const
{
    return compatibility.value == "DHU11";
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
    rx_interrupt_pending = false;
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
    tx_interrupt_pending = false;
}

void dhq11_c::print_line_status(void)
{
    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i) {
        line_state_t &line = lines[i];
        printf("DHQ11 line %u: tcp=%u listen=%s client=%s rx=%u tx=%u dcd=%u dsr=%u cts=%u dtr=%u rts=%u\n",
               i,
               line.tcp_port,
               line.listen_fd >= 0 ? "up" : "down",
               line.client_fd >= 0 ? "up" : "down",
               (unsigned)line.rx_queue.size(),
               (unsigned)line.tx_queue.size(),
               line.connected ? 1 : 0,
               line.connected ? 1 : 0,
               line.connected ? 1 : 0,
               line.connected ? 1 : 0,
               line.connected ? 1 : 0);
    }
}

void dhq11_c::update_csr(void)
{
    uint16_t value = get_register_dato_value(reg_csr);
    const uint16_t line_mask = is_dhu11_mode() ? DHQ11_CSR_DHU11_LINE_MASK : DHQ11_CSR_DHV11_LINE_MASK;
    const uint16_t rda_mask = is_dhu11_mode() ? DHQ11_CSR_DHU11_RDA : DHQ11_CSR_DHV11_RDA;
    const uint16_t txa_mask = DHQ11_CSR_DHV11_TXA;
    const uint16_t tx_line_mask = is_dhu11_mode() ? DHQ11_CSR_DHU11_TX_LINE : DHQ11_CSR_DHV11_TX_LINE;
    line_state_t &line = lines[selected_line];

    tx_ready = !line.tx_dma_pending && !line.tx_dma_active && line.tx_queue.empty();
    value &= ~(rda_mask | txa_mask | tx_line_mask | DHQ11_CSR_DHV11_DF | DHQ11_CSR_DHV11_TDE);
    if (rx_done)
        value |= rda_mask;
    if (!tx_ready) {
        value |= txa_mask;
        value |= tx_line_mask;
    }
    if (line.tx_dma_error)
        value |= DHQ11_CSR_DHV11_TDE;
    value &= ~line_mask;
    value |= (uint16_t)(selected_line & 0x0007);

    bool interrupt_condition = get_intr_condition();
    switch (intr_request.edge_detect(interrupt_condition)) {
    case intr_request_c::INTERRUPT_EDGE_RAISING:
        if (rx_done && rx_intr_enable)
            rx_interrupt_pending = true;
        if (tx_ready && tx_intr_enable)
            tx_interrupt_pending = true;
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

void dhq11_c::update_lpr(void)
{
    line_state_t &line = lines[selected_line];
    uint16_t value = get_register_dato_value(reg_lpr);
    value &= ~DHQ11_LPR_BAUD_MASK;
    if (line.connected)
        value |= 0x1000;
    set_register_dati_value(reg_lpr, value, __func__);
}

void dhq11_c::update_stat(void)
{
    line_state_t &line = lines[selected_line];
    uint16_t value = get_register_dato_value(reg_stat);
    value &= ~(DHQ11_STAT_DSR | DHQ11_STAT_RI | DHQ11_STAT_DCD | DHQ11_STAT_CTS | DHQ11_STAT_DHU);
    if (line.connected) {
        value |= DHQ11_STAT_DSR | DHQ11_STAT_DCD | DHQ11_STAT_CTS;
        if (line.telnet_iac)
            value |= DHQ11_STAT_RI;
    }
    if (!is_dhu11_mode())
        value &= ~DHQ11_STAT_DHU;
    set_register_dati_value(reg_stat, value, __func__);
}

void dhq11_c::update_ctrl(void)
{
    line_state_t &line = lines[selected_line];
    uint16_t value = get_register_dato_value(reg_ctrl);
    value &= ~(DHQ11_CTRL_RTS | DHQ11_CTRL_DTR | DHQ11_CTRL_LTYP | DHQ11_CTRL_MAINT | DHQ11_CTRL_FXOFF |
               DHQ11_CTRL_OAUTO | DHQ11_CTRL_BREAK | DHQ11_CTRL_RXE | DHQ11_CTRL_IAF | DHQ11_CTRL_TXA);
    if (line.connected)
        value |= DHQ11_CTRL_RTS | DHQ11_CTRL_DTR | DHQ11_CTRL_RXE;
    if (line.telnet_iac)
        value |= DHQ11_CTRL_BREAK;
    set_register_dati_value(reg_ctrl, value, __func__);
}

void dhq11_c::update_tbadl(void)
{
    line_state_t &line = lines[selected_line];
    set_register_dati_value(reg_tbadl, line.tbadl, __func__);
}

void dhq11_c::update_tbadh(void)
{
    line_state_t &line = lines[selected_line];
    set_register_dati_value(reg_tbadh, line.tbadh, __func__);
}

void dhq11_c::update_tbct(void)
{
    line_state_t &line = lines[selected_line];
    set_register_dati_value(reg_tbct, line.tbct, __func__);
}

bool dhq11_c::get_intr_condition(void)
{
    bool rx_condition = rx_done && rx_intr_enable && !rx_interrupt_pending;
    bool tx_condition = tx_ready && tx_intr_enable && !tx_interrupt_pending;
    return rx_condition || tx_condition;
}

void dhq11_c::reset_state(void)
{
    rx_queue.clear();
    selected_line = 0;
    rx_line = 0;
    rx_buffer = 0;
    rx_done = false;
    rx_intr_enable = false;
    rx_interrupt_pending = false;
    tx_ready = true;
    tx_intr_enable = false;
    tx_interrupt_pending = false;
    intr_request.edge_detect_reset();

    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i) {
        lines[i].tbadl = 0;
        lines[i].tbadh = 0;
        lines[i].tbct = 0;
        lines[i].tx_dma_pending = false;
        lines[i].tx_dma_active = false;
        lines[i].tx_dma_error = false;
        lines[i].rx_queue.clear();
        lines[i].tx_queue.clear();
        lines[i].telnet_iac = false;
        lines[i].telnet_skip_option = false;
    }

    if (handle != 0) {
        reset_unibus_registers();
        update_csr();
        update_rbuf();
        update_lpr();
        update_stat();
        update_ctrl();
        update_tbadl();
        update_tbadh();
        update_tbct();
    }
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
    if (intr_request.get_priority_slot() < PRIORITY_SLOT_COUNT)
        qunibusadapter->cancel_INTR(intr_request);
    close_all_lines();
}

bool dhq11_c::on_param_changed(parameter_c *param)
{
    if (param == &compatibility) {
        if (compatibility.new_value != "DHV11" && compatibility.new_value != "DHU11") {
            WARNING("DHQ11 compatibility '%s' not supported, using DHV11", compatibility.new_value.c_str());
            compatibility.new_value = "DHV11";
        }
    }
    if (param == &priority_slot) {
        intr_request.set_priority_slot(priority_slot.new_value);
    } else if (param == &intr_vector) {
        intr_request.set_vector(intr_vector.new_value);
    } else if (param == &intr_level) {
        intr_request.set_level(intr_level.new_value);
    } else if (param == &compatibility) {
        reset_state();
    }
    return qunibusdevice_c::on_param_changed(param);
}

void dhq11_c::eval_csr_dato_value(void)
{
    uint16_t value = get_register_dato_value(reg_csr);
    const uint16_t rxie_mask = is_dhu11_mode() ? DHQ11_CSR_DHU11_RXIE : DHQ11_CSR_DHV11_RXIE;
    const uint16_t txie_mask = is_dhu11_mode() ? DHQ11_CSR_DHU11_TXIE : DHQ11_CSR_DHV11_TXIE;
    const uint16_t mr_mask = is_dhu11_mode() ? DHQ11_CSR_DHU11_MR : DHQ11_CSR_DHV11_MR;

    selected_line = value & 0x0007;
    rx_intr_enable = !!(value & rxie_mask);
    tx_intr_enable = !!(value & txie_mask);
    if (value & mr_mask) {
        reset_state();
        value &= ~mr_mask;
        set_register_dati_value(reg_csr, value, __func__);
        return;
    }
    update_csr();
}

void dhq11_c::eval_lpr_dato_value(void)
{
    update_lpr();
}

void dhq11_c::eval_ctrl_dato_value(void)
{
    line_state_t &line = lines[selected_line];
    uint16_t value = get_register_dato_value(reg_ctrl);
    if (value & DHQ11_CTRL_TXA) {
        line.tx_dma_pending = false;
        line.tx_dma_active = false;
        line.tx_dma_error = false;
        line.tbct = 0;
        line.tx_queue.clear();
    }
    line.connected = !!(value & DHQ11_CTRL_DTR);
    if (!line.connected && line.client_fd >= 0) {
        close_fd(line.client_fd);
        line.connected = false;
    }
    update_ctrl();
    update_stat();
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
        if (unibus_control == QUNIBUS_CYCLE_DATO) {
            tx_interrupt_pending = false;
            queue_tx_byte(selected_line, get_register_dato_value(reg_rbuf) & 0x00ff);
            update_csr();
        } else if (unibus_control == QUNIBUS_CYCLE_DATI) {
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
                rx_interrupt_pending = false;
            } else {
                rx_done = false;
                rx_interrupt_pending = false;
            }
            update_csr();
            update_rbuf();
        }
        break;
    case dhq11_idx_lpr:
        if (unibus_control == QUNIBUS_CYCLE_DATO)
            eval_lpr_dato_value();
        else
            update_lpr();
        break;
    case dhq11_idx_stat:
        update_stat();
        break;
    case dhq11_idx_ctrl:
        if (unibus_control == QUNIBUS_CYCLE_DATO)
            eval_ctrl_dato_value();
        else
            update_ctrl();
        break;
    case dhq11_idx_tbadl:
        if (unibus_control == QUNIBUS_CYCLE_DATO) {
            line_state_t &line = lines[selected_line];
            line.tbadl = get_register_dato_value(reg_tbadl);
            update_tbadl();
        } else {
            update_tbadl();
        }
        break;
    case dhq11_idx_tbadh:
        if (unibus_control == QUNIBUS_CYCLE_DATO) {
            line_state_t &line = lines[selected_line];
            line.tbadh = get_register_dato_value(reg_tbadh);
            update_tbadh();
        } else {
            update_tbadh();
        }
        break;
    case dhq11_idx_tbct:
        if (unibus_control == QUNIBUS_CYCLE_DATO) {
            line_state_t &line = lines[selected_line];
            line.tbct = get_register_dato_value(reg_tbct);
            line.tx_dma_pending = line.tbct != 0;
            line.tx_dma_active = false;
            line.tx_dma_error = false;
            update_tbct();
            update_csr();
        } else {
            update_tbct();
        }
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
    update_stat();
    update_ctrl();
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
        update_stat();
        update_ctrl();
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
        if (line.tx_queue.empty())
            tx_interrupt_pending = false;
        update_csr();
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        INFO("DHQ11 line %u send() failed: %s", line_index, strerror(errno));
        close_fd(line.client_fd);
        line.connected = false;
        update_stat();
        update_ctrl();
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

    update_stat();
    update_ctrl();

    pthread_mutex_unlock(&on_after_register_access_mutex);
}

void dhq11_c::worker(unsigned instance)
{
    UNUSED(instance);
    worker_init_realtime_priority(rt_device);

    while (!workers_terminate) {
        service_lines();
        service_pending_tx_dma();
    }
}

bool dhq11_c::service_pending_tx_dma(void)
{
    unsigned line_index = DHQ11_LINE_COUNT;
    uint32_t start_address = 0;
    uint16_t word_count = 0;

    pthread_mutex_lock(&on_after_register_access_mutex);
    for (unsigned i = 0; i < DHQ11_LINE_COUNT; ++i) {
        line_state_t &line = lines[i];
        if (line.tx_dma_pending && !line.tx_dma_active) {
            line.tx_dma_pending = false;
            line.tx_dma_active = true;
            line.tx_dma_error = false;
            line_index = i;
            start_address = ((uint32_t)line.tbadh << 16) | (uint32_t)line.tbadl;
            word_count = line.tbct;
            break;
        }
    }
    pthread_mutex_unlock(&on_after_register_access_mutex);

    if (line_index >= DHQ11_LINE_COUNT || word_count == 0)
        return false;

    std::vector<uint16_t> buffer(word_count);
    dma_request.success = false;
    qunibusadapter->DMA(dma_request, true, QUNIBUS_CYCLE_DATI, start_address, buffer.data(), word_count);

    pthread_mutex_lock(&on_after_register_access_mutex);
    line_state_t &line = lines[line_index];
    line.tx_dma_active = false;
    if (dma_request.success) {
        for (uint16_t word : buffer)
            line.tx_queue.push_back((uint8_t)(word & 0x00ff));
        line.tbct = 0;
        if (line_index == selected_line)
            tx_interrupt_pending = false;
    } else {
        line.tx_dma_error = true;
    }
    if (line_index == selected_line)
        update_csr();
    pthread_mutex_unlock(&on_after_register_access_mutex);

    return dma_request.success;
}
