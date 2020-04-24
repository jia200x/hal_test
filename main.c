/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for AT86RF2xx network device driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sys/uio.h"
#include "luid.h"

#include "net/netdev.h"
#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "mutex.h"
#if !NETDEV
#include "net/ieee802154/radio.h"
#endif

#ifdef MODULE_AT86RF2XX
#include "at86rf2xx.h"
#include "at86rf2xx_params.h"
#endif

#ifdef MODULE_NRF802154
#include "nrf802154.h"
#endif

#include "od.h"
#include "event/thread.h"
#include "event/callback.h"
#include "xtimer.h"
#define MAX_LINE    (80)

uint8_t buffer[127];
eui64_t ext_addr;
network_uint16_t short_addr;
uint8_t seq;

#ifdef MODULE_AT86RF2XX
static at86rf2xx_t at86rf2xx_dev;
static void *_dev = &at86rf2xx_dev;
#endif
#ifdef MODULE_NRF802154
#if NETDEV
extern netdev_ieee802154_t nrf802154_dev;
#else
extern ieee802154_dev_t nrf802154_dev;
#endif
static void *_dev = &nrf802154_dev;
#endif

event_t _irq_event;
            
static void _print_packet(size_t size, uint8_t lqi, int16_t rssi)
{
    puts("Packet received:");
    for (unsigned i=0;i<size;i++) {
        printf("%02x ", buffer[i]);
    }
    printf("LQI: %i, RSSI: %i\n", (int) lqi, (int) rssi);
    puts("");
}

static void _finish_tx(void)
{
    puts("TX_DONE!");
}

static void _isr(void *arg)
{
    (void) arg;
    event_post(EVENT_PRIO_HIGHEST, &_irq_event);
}

static int print_addr(int argc, char **argv)
{
    (void) argc;
    (void) argv;
    uint8_t *_p = (uint8_t*) &ext_addr;
    for(int i=0;i<8;i++) {
        printf("%02x", *_p++);
    }
    printf("\n");
    return 0;
}

#if NETDEV
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    if (event == NETDEV_EVENT_ISR) {
        _isr(NULL);
    }
    else {
        switch (event) {
            case NETDEV_EVENT_RX_COMPLETE:
            {

                netdev_ieee802154_rx_info_t rx_info;
                size_t data_len = dev->driver->recv(dev, buffer, sizeof(buffer), &rx_info);
                _print_packet(data_len, rx_info.lqi, rx_info.rssi);
                break;
            }
            case NETDEV_EVENT_TX_COMPLETE:
            case NETDEV_EVENT_TX_NOACK:
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                _finish_tx();
                break;
            default:
                puts("Unexpected event received");
                break;
        }
    }
}

static void _send(iolist_t *pkt)
{
    netdev_t *netdev = (netdev_t*) _dev;
    netdev->driver->send(netdev, pkt);
}

static void _isr_handler(void)
{
    netdev_t *netdev = (netdev_t*) _dev;
    netdev->driver->isr(netdev);
}

static int _init(void)
{
#ifdef MODULE_AT86RF2XX
    const at86rf2xx_params_t *p = &at86rf2xx_params[0];
    at86rf2xx_setup(&at86rf2xx_dev, p);
#endif
    netopt_enable_t en = NETOPT_ENABLE;
    netdev_t *netdev = (netdev_t*) _dev;
    netdev->event_callback = _event_cb;
    netdev->driver->init(netdev);
    netdev->driver->set(netdev, NETOPT_RX_END_IRQ, &en, sizeof(en));
    netdev->driver->set(netdev, NETOPT_TX_END_IRQ, &en, sizeof(en));
    uint16_t channel = 21;
    int16_t power = 1000;
    netdev->driver->set(netdev, NETOPT_CHANNEL, &channel, sizeof(channel));
    netdev->driver->set(netdev, NETOPT_TX_POWER, &power, sizeof(power));

    netdev->driver->get(netdev, NETOPT_ADDRESS, &short_addr, sizeof(short_addr));
    netdev->driver->get(netdev, NETOPT_ADDRESS_LONG, &ext_addr, sizeof(ext_addr));

    return 0;
}
#else
static void _hal_rx_done(ieee802154_dev_t *dev)
{
    ieee802154_rx_info_t info;

    /* Read packet from internal framebuffer
     *
     * NOTE: It's possible to call `ieee802154_radio_len` to retrieve the packet
     * length. Since the buffer is fixed in this demo, we don't use it
     */
    size_t size = ieee802154_radio_read(dev, buffer, 127, &info);

    _print_packet(size, info.lqi, info.rssi);
}

/* Event Notification callback */
static void _hal_radio_cb(ieee802154_dev_t *dev, ieee802154_tx_status_t status)
{
    switch(status) {
        case IEEE802154_RADIO_TX_DONE:
            /* The Radio HAL won't switch back to RX_ON automatically. So, it is
             * necessary to add an explicit call to this function */
            ieee802154_radio_set_trx_state(dev, IEEE802154_TRX_STATE_RX_ON);

            /* Note:
             * If the device supports frame retransmissions, it is possible to
             * call `ieee802154_radio_get_tx_status` to retrieve tx info
             * (pending bit, retransmission status, etc). However, this is not
             * included in this demo.
             */
            _finish_tx();
            break;
        case IEEE802154_RADIO_RX_DONE:
            _hal_rx_done(dev);
        default:
           break;
    }
}

static void _send(iolist_t *pkt)
{
    /* Set the transceiver state to TX_ON and load the packet into the
     * framebuffer.
     *
     * Note: It's not required to set TX_ON state before calling
     * prepare (only before calling transmit). However, this is done in order
     * to stop receiving packets while loading the framebuffer. Consider that
     * the TX_ON state consumes > 10 times more power than TRX_OFF, so loading
     * with `trx_state=TRX_OFF` is preferred.  NB the transition between
     * TRX_OFF and TX_ON is not immediate and this should be considered for
     * time-critical applications.
     */
    ieee802154_radio_set_trx_state(_dev, IEEE802154_TRX_STATE_TX_ON);
    ieee802154_radio_prepare(_dev, pkt);

    /* If the radio supports frame retransmissions (thus, CSMA-CA), send with
     * CSMA-CA. Otherwise, send without any kind of collision avoidance */
    ieee802154_tx_mode_t mode = ieee802154_radio_has_frame_retries(_dev) ?
        IEEE802154_TX_MODE_CSMA_CA : IEEE802154_TX_MODE_DIRECT;
    ieee802154_radio_transmit(_dev, mode);
}

static void _isr_handler(void)
{
    /* Process the radio IRQ handler */
    ieee802154_radio_irq_handler(_dev);
}

static int _init(void)
{
    /* Call the init function of the device (this will be handled by
     * `auto_init`) */
#ifdef MODULE_AT86RF2XX
    const at86rf2xx_params_t *p = &at86rf2xx_params[0];
    at86rf2xx_init(&at86rf2xx_dev, p, _isr);
#endif
#ifdef MODULE_NRF802154
    nrf802154_init(_isr);
#endif

    /* Set the Event Notification */
    ((ieee802154_dev_t*) _dev)->cb = _hal_radio_cb;

    /* Note that addresses are not kept in the radio. This assumes MAC layers
     * already have a copy of the address */
    luid_get_eui64(&ext_addr);
    luid_get_short(&short_addr);

    /* Start the radio (enable interrupts and put it in a state that can be
     * operated by the HAL. The transceiver state will be "TRX_OFF" */
    ieee802154_radio_start(_dev);

    /* If the radio supports address filtering, set all IEEE addresses */
    if (ieee802154_radio_has_addr_filter(_dev)) {
        ieee802154_radio_set_hw_addr_filter(_dev, (uint8_t*) &short_addr, (uint8_t*) &ext_addr, 0x23);
    }

    /* Set PHY configuration */
    ieee802154_phy_conf_t conf = {.channel=21, .page=0, .pow=1000};
    ieee802154_radio_config_phy(_dev, &conf);


    /* Set the transceiver state to RX_ON in order to receive packets */
    ieee802154_radio_set_trx_state(_dev, IEEE802154_TRX_STATE_RX_ON);

    return 0;
}
#endif

static int send(uint8_t *dst, size_t dst_len,
                char *data)
{
    uint8_t flags;
    uint8_t mhr[IEEE802154_MAX_HDR_LEN];
    int mhr_len;

    le_uint16_t src_pan, dst_pan;
    iolist_t iol_data = {
        .iol_base = data,
        .iol_len = strlen(data),
        .iol_next = NULL,
    };

    flags = IEEE802154_FCF_TYPE_DATA | 0x20;
    src_pan = byteorder_btols(byteorder_htons(0x23));
    dst_pan = byteorder_btols(byteorder_htons(0x23));
    uint8_t src_len = 8;
    void *src = &ext_addr;

    /* fill MAC header, seq should be set by device */
    if ((mhr_len = ieee802154_set_frame_hdr(mhr, src, src_len,
                                        dst, dst_len,
                                        src_pan, dst_pan,
                                        flags, seq++)) < 0) {
        puts("txtsnd: Error preperaring frame");
        return 1;
    }

    iolist_t iol_hdr = {
        .iol_next = &iol_data,
        .iol_base = mhr,
        .iol_len = mhr_len,
    };

    _send(&iol_hdr);
    return 0;
}

static inline int _dehex(char c, int default_)
{
    if ('0' <= c && c <= '9') {
        return c - '0';
    }
    else if ('A' <= c && c <= 'F') {
        return c - 'A' + 10;
    }
    else if ('a' <= c && c <= 'f') {
        return c - 'a' + 10;
    }
    else {
        return default_;
    }
}

static size_t _parse_addr(uint8_t *out, size_t out_len, const char *in)
{
    const char *end_str = in;
    uint8_t *out_end = out;
    size_t count = 0;
    int assert_cell = 1;

    if (!in || !*in) {
        return 0;
    }
    while (end_str[1]) {
        ++end_str;
    }

    while (end_str >= in) {
        int a = 0, b = _dehex(*end_str--, -1);
        if (b < 0) {
            if (assert_cell) {
                return 0;
            }
            else {
                assert_cell = 1;
                continue;
            }
        }
        assert_cell = 0;

        if (end_str >= in) {
            a = _dehex(*end_str--, 0);
        }

        if (++count > out_len) {
            return 0;
        }
        *out_end++ = (a << 4) | b;
    }
    if (assert_cell) {
        return 0;
    }
    /* out is reversed */

    while (out < --out_end) {
        uint8_t tmp = *out_end;
        *out_end = *out;
        *out++ = tmp;
    }

    return count;
}

int txtsnd(int argc, char **argv)
{
    char *text;
    uint8_t addr[8];
    size_t res;

    if (argc != 3) {
        puts("Usage: txtsnd <long_addr> <data>");
        return 1;
    }

    res = _parse_addr(addr, sizeof(addr), argv[1]);
    if (res == 0) {
        puts("Usage: txtsnd <long_addr> <data>");
        return 1;
    }
    text = argv[2];
    return send(addr, res, text);
}

int config_phy(int argc, char **argv)
{
    if (argc < 2) {
        puts("Usage: config_phy <channel> <tx_pow>");
        return 1;
    }

    uint8_t channel = atoi(argv[1]);
    int16_t tx_pow = atoi(argv[2]);

    if (channel < 11 || channel > 26) {
        puts("Wrong channel configuration (11 <= channel <= 26).");
        return 1;
    }
#if NETDEV
    netdev_t *netdev = (netdev_t*) _dev;
    netdev->driver->set(netdev, NETOPT_CHANNEL, &channel, sizeof(channel));
    netdev->driver->set(netdev, NETOPT_TX_POWER, &tx_pow, sizeof(tx_pow));
#else
    ieee802154_dev_t *dev = _dev;
    ieee802154_phy_conf_t conf = {.channel=channel, .page=0, .pow=tx_pow};
    dev->driver->config_phy(dev, &conf);
#endif
    puts("Success!");
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "config_phy", "Set channel and TX power", config_phy},
    { "print_addr", "Print IEEE802.15.4 addresses", print_addr},
    { "txtsnd", "Send IEEE 802.15.4 packet", txtsnd },
    { NULL, NULL, NULL }
};

/* This is our Bottom-Half processor */
void _irq_event_handler(event_t *event)
{
    (void) event;
    _isr_handler();
}

event_t _irq_event = {
    .handler = _irq_event_handler,
};

int main(void)
{
    _init();

    /* start the shell */
    puts("Initialization successful - starting the shell now");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
