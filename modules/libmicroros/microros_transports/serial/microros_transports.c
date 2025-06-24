#include <uxr/client/transport.h>

#include <microros_transports.h>
#include <version.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3,1,0)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/posix/unistd.h>
#else
#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <posix/unistd.h>
#endif

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define RING_BUF_SIZE 2048
#define UART_NODE DT_NODELABEL(usart1)

char uart_in_buffer[RING_BUF_SIZE];
char uart_out_buffer[RING_BUF_SIZE];

struct ring_buf out_ringbuf, in_ringbuf;

// --- micro-ROS Serial Transport for Zephyr ---

static void uart_fifo_callback(const struct device *dev, void *args)
{
    ARG_UNUSED(args);

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {

        uint8_t *dst;
        uint32_t space = ring_buf_put_claim(&in_ringbuf, &dst,
                                            ring_buf_space_get(&in_ringbuf));

        if (space == 0) {            /* ring buffer full – drop one byte */
            uint8_t drop;
            uart_fifo_read(dev, &drop, 1);
            continue;
        }

        int rx = uart_fifo_read(dev, dst, space);
        ring_buf_put_finish(&in_ringbuf, rx);
    }
}



bool zephyr_transport_open(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    params->uart_dev = DEVICE_DT_GET(UART_NODE);
    if (!params->uart_dev) {
        printk("Serial device not found\n");
        return false;
    }

    ring_buf_init(&in_ringbuf, sizeof(uart_in_buffer), uart_in_buffer);

    uart_irq_callback_set(params->uart_dev, uart_fifo_callback);

    /* Enable rx interrupts */
    uart_irq_rx_enable(params->uart_dev);

    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport *transport)
{
    zephyr_transport_params_t *p = (zephyr_transport_params_t *)transport->args;
    const struct device *uart = p->uart_dev;

    if (!uart)
        return false;

    uart_irq_rx_disable(uart);
    uart_irq_tx_disable(uart);
    uart_irq_callback_set(uart, NULL);

    ring_buf_reset(&in_ringbuf);
    p->uart_dev = NULL;
    return true;
}


size_t zephyr_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf,
                              size_t len,
                              uint8_t *err)
{
    zephyr_transport_params_t *params =
        (zephyr_transport_params_t *)transport->args;
    const struct device *uart = params->uart_dev;

    size_t sent = 0;
    while (sent < len) {
        size_t n = uart_fifo_fill(uart, buf + sent, len - sent);
        sent += n;
        if (n == 0) {
            k_yield();
        }
    }
    return sent;
}


size_t zephyr_transport_read(struct uxrCustomTransport *transport,
                             uint8_t *buf,
                             size_t len,
                             int timeout,          /* ms, <0 = wait forever */
                             uint8_t *err)
{
    zephyr_transport_params_t *p = (zephyr_transport_params_t *)transport->args;

    const uint64_t deadline = (timeout < 0)
                                ? UINT64_MAX
                                : k_uptime_get() + (uint64_t)timeout;

    size_t got = 0;
    do {
        uart_irq_rx_disable(p->uart_dev);
        got = ring_buf_get(&in_ringbuf, buf, len);
        uart_irq_rx_enable(p->uart_dev);

        if (got || k_uptime_get() >= deadline) {
            break;                  /* read something or timed out */
        }
        k_msleep(1);                /* co-operative wait, no busy loop */
    } while (true);

    return got;
}
