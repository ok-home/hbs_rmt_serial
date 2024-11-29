/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/queue.h"
#include "driver/timer.h"
#include "esp_clk_tree.h"
#include <rom/ets_sys.h>

#include "soft_serial_rmt.h"

#define RMT_TX_GPIO (23)
#define RMT_RX_GPIO (22)

static const char *TAG = "RMT TEST";

mdb_item16_t send_data[] =
    {
        {.cmd = 1, .data = 0xaa},
        {.cmd = 0, .data = 0x1},
        {.cmd = 0, .data = 0x10},
        {.cmd = 0, .data = 0xff},
        {.cmd = 1, .data = 0xff},
        {.cmd = 0, .data = 0x00},
        {.cmd = 0, .data = 0x0f},
        {.cmd = 0, .data = 0xf0},
        {.cmd = 0, .data = 0x30},
        {.cmd = 0, .data = 0x03},
        {.cmd = 0, .data = 0xe0},
        {.cmd = 0, .data = 0x0e},
        {.cmd = 1, .data = 0xc0},
        {.cmd = 0, .data = 0x0c},
        {.cmd = 1, .data = 0xcc},
        {.cmd = 0, .data = 0xbb},
        {.cmd = 0, .data = 0x33},
        {.cmd = 1, .data = 0x77},
        {.cmd = 0, .data = 0x88},
        {.cmd = 0, .data = 0x11},
        {.cmd = 1, .data = 0x0}};

mdb_packet_t send_packet =
    {
        .packet_hdr.packet_size = 21,
        .packet_data =
            {
                {.cmd = 1, .data = 0xaa},
                {.cmd = 0, .data = 0x01},
                {.cmd = 0, .data = 0x10},
                {.cmd = 0, .data = 0xff},
                {.cmd = 1, .data = 0xff},
                {.cmd = 0, .data = 0x00},
                {.cmd = 0, .data = 0x0f},
                {.cmd = 0, .data = 0xf0},
                {.cmd = 0, .data = 0x30},
                {.cmd = 0, .data = 0x03},
                {.cmd = 0, .data = 0xe0},
                {.cmd = 0, .data = 0x0e},
                {.cmd = 1, .data = 0xc0},
                {.cmd = 0, .data = 0x0c},
                {.cmd = 1, .data = 0xcc},
                {.cmd = 0, .data = 0xbb},
                {.cmd = 0, .data = 0x33},
                {.cmd = 1, .data = 0x77},
                {.cmd = 0, .data = 0x88},
                {.cmd = 0, .data = 0x11},
                {.cmd = 1, .data = 0x00}}};

void app_main(void)

{

#if DBG
// dbg logic analyzer
#include "logic_analyzer_ws_server.h"

    logic_analyzer_ws_server();
    //
    //  dbg GPIO PIN
    gpio_reset_pin(TX_TEST_GPIO);
    gpio_set_direction(TX_TEST_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RX_TEST_GPIO);
    gpio_set_direction(RX_TEST_GPIO, GPIO_MODE_OUTPUT);
    //
#endif
    mdb_init(RMT_RX_GPIO, RMT_TX_GPIO);

    mdb_packet_t rx_packet = {0};
    int cnt = 0;
    int icnt = 0;
    int err_cnt = 0;
    while (1)
    {
        //vTaskDelay(10);
#if DBG
        gpio_set_level(TX_TEST_GPIO, 1);
#endif
        mdb_tx_packet(&send_packet);
        mdb_rx_packet(&rx_packet, portMAX_DELAY);
#if DBG
        gpio_set_level(TX_TEST_GPIO, 0);
#endif
        if (send_packet.packet_hdr.packet_size != rx_packet.packet_hdr.packet_size)
            ESP_LOGE(TAG, "ERROR Size tx=%d rx=%d", send_packet.packet_hdr.packet_size, rx_packet.packet_hdr.packet_size);
        for (int i = 0; i < send_packet.packet_hdr.packet_size; i++)
        {
            if (send_packet.packet_data[i].data != rx_packet.packet_data[i].data)
            {
                ESP_LOGE(TAG, "ERROR Packet idx= %d tx %x rx %x err_cnt=%d", i, send_packet.packet_data[i].data, rx_packet.packet_data[i].data, err_cnt++);
            }
        }
        memset((void *)&rx_packet, 0, sizeof(rx_packet));
        cnt++;
        if (cnt >= 10)
        {
            ESP_LOGI(TAG, "Send/Receive %d packet icnt=%d", cnt, icnt++);
            cnt = 0;
        }
    }

}