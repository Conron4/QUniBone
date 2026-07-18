/* dhq11.hpp: DHQ11-compatible QBUS serial line controller with telnet forwarding

 Copyright (c) 2026

 This is an initial implementation of a DHQ11-style multi-line serial
 controller for QBone/QUniBone.  It focuses on the host-side transport and
 exposes eight independent TCP listeners for the eight serial lines.
 */

#ifndef _DHQ11_HPP_
#define _DHQ11_HPP_

#include <deque>
#include <string>

#include <pthread.h>

#include "parameter.hpp"
#include "qunibusdevice.hpp"

class dhq11_c : public qunibusdevice_c {
private:

    enum dhq11_personality_e {
        personality_dhv11 = 0,
        personality_dhu11 = 1,
    };

    enum dhq11_reg_index {
        dhq11_idx_csr = 0,
        dhq11_idx_rbuf,
        dhq11_idx_lpr,
        dhq11_idx_stat,
        dhq11_idx_ctrl,
        dhq11_idx_xbuf,
        dhq11_idx_count
    };

    struct line_state_t {
        unsigned line_index;
        int listen_fd;
        int client_fd;
        unsigned tcp_port;
        bool connected;
        bool telnet_iac;
        bool telnet_skip_option;
        std::deque<uint8_t> rx_queue;
        std::deque<uint8_t> tx_queue;

        line_state_t()
            : line_index(0), listen_fd(-1), client_fd(-1), tcp_port(0), connected(false),
              telnet_iac(false), telnet_skip_option(false) {}
    };

    struct rx_entry_t {
        uint8_t line;
        uint8_t value;
    };

    qunibusdevice_register_t *reg_csr;
    qunibusdevice_register_t *reg_rbuf;
    qunibusdevice_register_t *reg_lpr;
    qunibusdevice_register_t *reg_stat;
    qunibusdevice_register_t *reg_ctrl;
    qunibusdevice_register_t *reg_xbuf;

    intr_request_c intr_request = intr_request_c(this);

    dhq11_personality_e personality;

    line_state_t lines[8];
    std::deque<rx_entry_t> rx_queue;

    uint8_t selected_line;
    uint8_t rx_line;
    uint8_t rx_buffer;
    bool rx_done;
    bool rx_intr_enable;
    bool rx_interrupt_pending;
    bool tx_ready;
    bool tx_intr_enable;
    bool tx_interrupt_pending;

    bool is_dhu11_mode(void) const;
    bool setup_line(unsigned line_index, unsigned port_base);
    void close_line(line_state_t &line);
    void close_all_lines(void);
    bool open_listeners(void);
    void service_lines(void);
    void service_listener(line_state_t &line, unsigned line_index);
    void service_client_rx(line_state_t &line, unsigned line_index);
    void service_client_tx(line_state_t &line, unsigned line_index);
    void queue_rx_byte(unsigned line_index, uint8_t value);
    void queue_tx_byte(unsigned line_index, uint8_t value);
    void update_csr(void);
    void update_rbuf(void);
    void update_lpr(void);
    void update_stat(void);
    void update_ctrl(void);
    void update_xbuf(void);
    bool get_intr_condition(void);
    void reset_state(void);
    void eval_csr_dato_value(void);
    void eval_lpr_dato_value(void);
    void eval_ctrl_dato_value(void);
    void eval_xbuf_dato_value(void);

public:
    void print_line_status(void);

public:
    dhq11_c();
    ~dhq11_c();

    parameter_unsigned_c telnet_port_base = parameter_unsigned_c(
        this,
        "telnet_port_base",
        "tp",
        false,
        "Base TCP port for line listeners",
        "%u",
        "TCP port base for line 0; the remaining lines use consecutive ports",
        16,
        10);

    parameter_string_c compatibility = parameter_string_c(
        this,
        "compatibility",
        "cmp",
        false,
        "DHV11/DHU11 personality; default DHV11");

    void reset(void);

    bool on_before_install(void) override;
    void on_after_uninstall(void) override;
    bool on_param_changed(parameter_c *param) override;

    void worker(unsigned instance) override;
    void on_after_register_access(qunibusdevice_register_t *device_reg,
                                  uint8_t unibus_control,
                                  DATO_ACCESS access) override;
    void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
};

#endif