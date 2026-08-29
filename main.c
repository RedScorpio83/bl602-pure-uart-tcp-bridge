/**
 * Bouffalo Lab BL602 - Pure Local UART to TCP Bridge
 * Dedicated 100% Local Firmware for Solar Inverter Telemetry
 *
 * WiFi: STA mode -> Default SSID "lab4", Password "qqwe7799rm5974"
 * Fallback: SoftAP "BL602-Setup", Password "config1234" (Port 8899)
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
#include <errno.h>

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
#include <hosal_uart.h>
#include <bl_gpio.h>
#include <bl_sys.h>
#include <easyflash.h>

#define WIFI_TARGET_SSID        "lab4"
#define WIFI_TARGET_PASSWORD    "qqwe7799rm5974"
#define TCP_BRIDGE_PORT         8888
#define UART_BAUD_RATE          2400

#define AP_FALLBACK_SSID        "BL602-Setup"
#define AP_FALLBACK_PASSWORD    "config1234"
#define AP_FALLBACK_CHANNEL     6
#define CONFIG_TCP_PORT         8899
#define WIFI_KEY_SSID           "inv_wifi_ssid"
#define WIFI_KEY_PASS           "inv_wifi_pass"
#define STA_CONNECT_TIMEOUT_MS  20000

/* Define UART0 device: ID 0, TX GPIO16, RX GPIO7, 2400 Baud */
HOSAL_UART_DEV_DECL(uart_bridge_dev, 0, 16, 7, UART_BAUD_RATE);

static wifi_conf_t g_wifi_conf = {
    .country_code = "IT",
};

static char g_sta_ssid[33];
static char g_sta_pass[65];
static volatile int g_sta_connected = 0;
static volatile int g_ap_fallback_active = 0;
static volatile int g_bridge_running = 0;
static TimerHandle_t g_sta_timeout_timer = NULL;
static wifi_interface_t g_ap_interface;

/* 1. Carica credenziali salvate (o quelle di default al primo avvio) */
static void load_wifi_credentials(void)
{
    size_t saved_len = 0;

    easyflash_init();

    saved_len = ef_get_env_blob(WIFI_KEY_SSID, g_sta_ssid, sizeof(g_sta_ssid) - 1, NULL);
    if (saved_len == 0) {
        strncpy(g_sta_ssid, WIFI_TARGET_SSID, sizeof(g_sta_ssid) - 1);
        g_sta_ssid[sizeof(g_sta_ssid) - 1] = 0;
        strncpy(g_sta_pass, WIFI_TARGET_PASSWORD, sizeof(g_sta_pass) - 1);
        g_sta_pass[sizeof(g_sta_pass) - 1] = 0;
    } else {
        g_sta_ssid[saved_len] = 0;
        saved_len = ef_get_env_blob(WIFI_KEY_PASS, g_sta_pass, sizeof(g_sta_pass) - 1, NULL);
        g_sta_pass[saved_len] = 0;
    }
}

/* 2. Salva nuove credenziali e riavvia */
static void save_wifi_credentials_and_reboot(const char *ssid, const char *pass)
{
    ef_set_env_blob(WIFI_KEY_SSID, ssid, strlen(ssid));
    ef_set_env_blob(WIFI_KEY_PASS, pass, strlen(pass));
    ef_save_env();

    vTaskDelay(pdMS_TO_TICKS(500));
    bl_sys_reset_por();
}

/* 3. Config Server Task in modalita' AP (porta 8899) */
static void config_server_task(void *arg)
{
    int server_sock, client_sock;
    struct sockaddr_in addr;
    char buf[128];

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(CONFIG_TCP_PORT);

    bind(server_sock, (struct sockaddr *)&addr, sizeof(addr));
    listen(server_sock, 1);

    while (1) {
        client_sock = accept(server_sock, NULL, NULL);
        if (client_sock < 0) continue;

        int len = recv(client_sock, buf, sizeof(buf) - 1, 0);
        if (len > 0) {
            buf[len] = 0;
            char *comma = strchr(buf, ',');
            if (comma) {
                *comma = 0;
                char *new_ssid = buf;
                char *new_pass = comma + 1;
                char *nl = strpbrk(new_pass, "\r\n");
                if (nl) *nl = 0;

                send(client_sock, "OK, riavvio in corso...\n", 25, 0);
                close(client_sock);
                close(server_sock);

                save_wifi_credentials_and_reboot(new_ssid, new_pass);
            }
        }
        close(client_sock);
    }
}

/* 4. Avvia la modalita' AP di recovery */
static void start_ap_fallback(void)
{
    if (g_ap_fallback_active) return;
    g_ap_fallback_active = 1;

    g_ap_interface = wifi_mgmr_ap_enable();
    wifi_mgmr_ap_start(&g_ap_interface, AP_FALLBACK_SSID, 0, AP_FALLBACK_PASSWORD, AP_FALLBACK_CHANNEL);

    xTaskCreate(config_server_task, "cfg_srv", 1024, NULL, 15, NULL);
}

/* 5. Callback del timer di timeout connessione STA */
static void sta_timeout_callback(TimerHandle_t xTimer)
{
    if (!g_sta_connected) {
        start_ap_fallback();
    }
}

/* 6. TCP Server Bridge Task (porta 8888) */
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

        int nodelay = 1;
        setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        int flags = fcntl(client_sock, F_GETFL, 0);
        fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);

        while (1) {
            /* 1. TCP -> UART (Inverter Request) */
            int rx_len = recv(client_sock, buf, sizeof(buf), 0);
            if (rx_len > 0) {
                hosal_uart_send(&uart_bridge_dev, buf, rx_len);
            } else if (rx_len == 0) {
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

            vTaskDelay(pdMS_TO_TICKS(2));
        }

        close(client_sock);
        client_sock = -1;
    }
}

/* Connessione alla rete WiFi STA usando credenziali caricate */
static void wifi_connect_sta(void)
{
    wifi_interface_t wifi_interface = wifi_mgmr_sta_enable();
    wifi_mgmr_sta_connect(wifi_interface, g_sta_ssid, g_sta_pass, NULL, NULL, 0, 0);
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
            g_sta_connected = 1;
            if (g_sta_timeout_timer) {
                xTimerStop(g_sta_timeout_timer, 0);
            }
            if (!g_bridge_running) {
                g_bridge_running = 1;
                xTaskCreate(tcp_server_bridge_task, "tcp_bridge", 1024, NULL, 15, NULL);
            }
            break;

        case CODE_WIFI_ON_DISCONNECT:
            g_sta_connected = 0;
            vTaskDelay(pdMS_TO_TICKS(3000));
            wifi_connect_sta();
            break;

        default:
            break;
    }
}

int main(void)
{
    /* 1. Inizializza UART0 a 2400 Baud */
    hosal_uart_init(&uart_bridge_dev);

    /* 2. Carica credenziali da Flash EasyFlash */
    load_wifi_credentials();

    /* 3. Timer di fallback AP (20s) */
    g_sta_timeout_timer = xTimerCreate("sta_to", pdMS_TO_TICKS(STA_CONNECT_TIMEOUT_MS),
                                      pdFALSE, NULL, sta_timeout_callback);
    if (g_sta_timeout_timer) {
        xTimerStart(g_sta_timeout_timer, 0);
    }

    /* 4. Registra eventi WiFi e avvia loop */
    aos_register_event_filter(EV_WIFI, wifi_event_handler, NULL);
    aos_loop_run();

    return 0;
}
