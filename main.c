/**
 * Bouffalo Lab BL602 - Pure Local UART to TCP Bridge
 * Dedicated 100% Local Firmware for Solar Inverter Telemetry
 *
 * WiFi: STA mode -> SSID "lab4", Password "qqwe7799rm5974"
 * Serial: UART0 (TX GPIO16, RX GPIO7) @ 2400 Baud, 8N1
 * TCP Server: Port 8888 (0.0.0.0)
 */

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <aos/kernel.h>
#include <aos/yloop.h>
#include <event_device.h>
#include <cli.h>

#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/tcp.h>
#include <lwip/err.h>

#include <wifi_mgmr_ext.h>
#include <bl_wifi.h>
#include <hal_uart.h>
#include <hosal_uart.h>
#include <bl_uart.h>
#include <bl_gpio.h>

#define WIFI_TARGET_SSID     "lab4"
#define WIFI_TARGET_PASSWORD "qqwe7799rm5974"
#define TCP_BRIDGE_PORT      8888
#define UART_BAUD_RATE       2400

/* Define UART0 device: ID 0, TX GPIO16, RX GPIO7, 2400 Baud */
HOSAL_UART_DEV_DECL(uart_bridge_dev, 0, 16, 7, UART_BAUD_RATE);

static wifi_conf_t g_wifi_conf = {
    .country_code = "IT",
};

static volatile int g_bridge_running = 0;

/* TCP Server Task: Bridges raw TCP packets <-> UART0 */
static void tcp_server_bridge_task(void *pvParameters)
{
    int server_sock = -1;
    int client_sock = -1;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    uint8_t buf[256];
    
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        g_bridge_running = 0;
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_BRIDGE_PORT);

    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(server_sock);
        g_bridge_running = 0;
        vTaskDelete(NULL);
        return;
    }

    listen(server_sock, 2);

    while (1) {
        client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Lowest latency */
        int nodelay = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        /* Non-blocking mode */
        int flags = fcntl(client_sock, F_GETFL, 0);
        fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);

        while (1) {
            /* 1. TCP -> UART (Inverter Request) */
            int rx_len = recv(client_sock, buf, sizeof(buf), 0);
            if (rx_len > 0) {
                hosal_uart_send(&uart_bridge_dev, buf, rx_len);
            } else if (rx_len == 0) {
                /* Client closed connection */
                break;
            } else {
                if (errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR) {
                    break;
                }
            }

            /* 2. UART -> TCP (Inverter Response) */
            int ur_len = hosal_uart_receive(&uart_bridge_dev, buf, sizeof(buf));
            if (ur_len > 0) {
                int sent = send(client_sock, buf, ur_len, 0);
                if (sent < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
                    break;
                }
            }

            /* Yield CPU for 2ms */
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        close(client_sock);
        client_sock = -1;
    }
}

/* Connect to WiFi station */
static void wifi_connect_sta(void)
{
    wifi_interface_t wifi_interface = wifi_mgmr_sta_enable();
    wifi_mgmr_sta_connect(wifi_interface, WIFI_TARGET_SSID, WIFI_TARGET_PASSWORD, NULL, NULL, 0, 0);
}

/* Wi-Fi Event Handler */
static void wifi_event_handler(input_event_t *event, void *private_data)
{
    switch (event->code) {
        case CODE_WIFI_ON_INIT_DONE:
            wifi_mgmr_start_background(&g_wifi_conf);
            wifi_connect_sta();
            break;

        case CODE_WIFI_ON_MGMR_DONE:
            /* Connected and obtained DHCP IP */
            if (!g_bridge_running) {
                g_bridge_running = 1;
                xTaskCreate(tcp_server_bridge_task, "tcp_bridge", 1024, NULL, 15, NULL);
            }
            break;

        case CODE_WIFI_ON_DISCONNECT:
            /* Auto-reconnect after 3 seconds */
            vTaskDelay(pdMS_TO_TICKS(3000));
            wifi_connect_sta();
            break;

        default:
            break;
    }
}

int main(void)
{
    /* 1. Initialize Hardware UART0 at 2400 Baud */
    hosal_uart_init(&uart_bridge_dev);

    /* 2. Register WiFi subsystem event filter */
    aos_register_event_filter(EV_WIFI, wifi_event_handler, NULL);

    /* 3. Run AOS loop */
    aos_loop_run();

    return 0;
}
