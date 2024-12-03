/* hbs rmt serial

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
#include "freertos/event_groups.h"
#include "driver/rmt.h"

#include "hbs_rmt_serial.h"

static const char *TAG = "HBS RMT";

/*
 * Divider set
 * 9600-1%+2% about 9500-9800 - hbs baud standart
 * tx baud calculate 80 000 000 / RMT_TX_DIV / TX_BIT_DIVIDER
 * rx bit divider should be a little more then max tx rate(9800)
 * RX_BIT_DIVIDER = ( 80 000 000/19200/RMT_RX_DIV ) = 100-102
 * RMT_RX_IDLE_THRES may be  greater then (80 000 000/RX_BIT_DIVIDER/9500*11) = 1158(1200-2400)
 */

#define RX_CHANNEL RMT_CHANNEL_1
#define RMT_RX_DIV (40)          // 8
#define RMT_RX_IDLE_THRES (5000) // 12000
#define RX_BIT_DIVIDER (102)     // 1040
// min duration on log  ( dur = 59 ) compensation = 104-59 = 45
#define RX_PULSE_HI_LVL_DELAY_COMPENSATION (0)
#define RX_PULSE_LOW_LVL_DELAY_COMPENSATION (0)
#define RX_INVERT_LVL 0

#define TX_CHANNEL RMT_CHANNEL_0
// tx baud=80000000/40/104 = 19230 baud
#define RMT_TX_DIV (40)      // 8 // esp32_hbs = 82
#define TX_BIT_DIVIDER (102) // 1042 // esp32_hbs=100

#define BIT_IN_WORD (22) // -> for hsb start(2)+18 bit+stop(2)
                         // -> For hsb_rmt start(1 bit) + 20 bit (msb already 1, 16(8*2) bit data, 1(2) bit parity, lsb already 1 ) + stop(1 bit)
                         // -> for data start(1 bit) + 9 bit + stop(1 bit)

static QueueHandle_t hbs_rx_packet_queue;
static QueueHandle_t hbs_tx_packet_queue;
static TaskHandle_t hbs_rx_packet_task_handle;
static TaskHandle_t hbs_tx_packet_task_handle;

#define hbs_TX_DONE_BIT BIT0
static EventGroupHandle_t hbs_tx_event_group;

// single rmt item
typedef struct
{
    union
    {
        struct
        {
            uint16_t duration : 15; /*!< Duration of level */
            uint16_t level : 1;     /*!< Level  */
        };
        uint16_t val; /*!< Equivalent unsigned value for the RMT item */
    };
} rmt_item16_t;

static uint16_t decode_hbs_bit_data(uint32_t hbs_data_bit) // decode from hbs bit stream to byte+parity
{
    uint16_t data_parity = 0;
    hbs_data_bit = hbs_data_bit >> 1; // remove part of start bit
    hbs_data_bit &= 0x3ffff;          // remove part of stop bit
    for (int i = 0; i < 9; i++)       // 8 bit + parity
    {
        if ((hbs_data_bit & 1) == 1)
        {
            data_parity |= 1 << 9;
        }
        hbs_data_bit = hbs_data_bit >> 2;
        data_parity = data_parity >> 1;
    }
    return data_parity;
}

static void hbs_rx_packet_task(void *p)
{
    size_t length = 0;
    RingbufHandle_t rb = NULL;
    rmt_item16_t *items = NULL;
    uint32_t hbs_data_bit = 0; // 22 bit from hbs
    // hbs_item16_t data = {0};
    hbs_packet_t packet = {0};
    int cnt_bit = 0;  // wait start bit, bit count
    int cnt_byte = 0; // byte in packet
    rmt_get_ringbuf_handle(RX_CHANNEL, &rb);
    rmt_rx_start(RX_CHANNEL, true);
    while (1)
    {
        items = (rmt_item16_t *)xRingbufferReceive(rb, &length, portMAX_DELAY);
        if (items)
        {
#if DBG
            gpio_set_level(RX_TEST_GPIO, 1);
#endif
            length /= 2; // one RMT = 2 Bytes
            for (int i = 0; i < length; i++)
            {
#if RX_INVERT_LVL
                int lvl = (!items[i].level) & 1; // invert lvl
#else
                int lvl = (items[i].level) & 1;
#endif
//                int duration = (lvl == 1) ? (items[i].duration + RX_PULSE_HI_LVL_DELAY_COMPENSATION) / RX_BIT_DIVIDER : (items[i].duration - RX_PULSE_LOW_LVL_DELAY_COMPENSATION) / RX_BIT_DIVIDER;
                int duration = (items[i].duration + RX_BIT_DIVIDER/2) / RX_BIT_DIVIDER ;
                 ESP_LOGI(TAG, "%d lvl=%d, bit_in=%d,dur=%d", i, lvl, duration, items[i].duration);
                if (cnt_bit == 0) // start bit
                {
                    if (lvl == 0 && duration > 0 && duration < BIT_IN_WORD) // start bit
                    {
                        hbs_data_bit = 0;   // first  bits in byte
                        cnt_bit = duration; // start bit + some bits=0
                    }
                    else
                    {
                        ESP_LOGE(TAG, "receive frame err START bit %d lvl=%d, bit_in=%d,dur=%d", i, lvl, duration, items[i].duration);
                    }
                }
                else if (duration == 0 || (cnt_bit + duration) > (BIT_IN_WORD - 1)) // last item && stop bit
                {
                    for (; cnt_bit < BIT_IN_WORD - 1; cnt_bit++)
                    {
                        hbs_data_bit >>= 1;
                        hbs_data_bit |= lvl << (BIT_IN_WORD - 3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                    packet.packet_data[cnt_byte].val = decode_hbs_bit_data(hbs_data_bit);
                    cnt_byte++;
                    cnt_bit = 0; // wait next start bit
                }
                else
                {
                    for (int j = 0; j < duration; cnt_bit++, j++)
                    {
                        hbs_data_bit >>= 1;
                        hbs_data_bit |= lvl << (BIT_IN_WORD - 3); // 8 with cmd or parity check //BIT_IN_WORD-3
                    }
                }
            }
        }
        packet.packet_hdr.packet_size = cnt_byte;
        // ESP_LOGI(TAG, "all item converted %d byte ",cnt_byte);
        xQueueSend(hbs_rx_packet_queue, &packet, portMAX_DELAY);
#if DBG
        gpio_set_level(RX_TEST_GPIO, 0);
#endif
        cnt_byte = 0;
        cnt_bit = 0; // wait next start bit
        memset(&packet, 0, sizeof(packet));
        // after parsing the data, return spaces to ringbuffer.
        vRingbufferReturnItem(rb, (void *)items);
    }
}
static void hbs_item_to_rmt_item_cvt(rmt_item16_t *rmt_data, hbs_item16_t data)
{
    int cnt = 0;
    int parity = 0;          // parity bit calculate on cvt -> hbs_item16_t parity ignored
    rmt_data[cnt].level = 0; // start bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 1; // start bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    for (; cnt < BIT_IN_WORD - 4; cnt += 2) // 22 - 2 (start) - 2 (parity)  -> lsb first
    {
        rmt_data[cnt].level = data.val & 1;
        rmt_data[cnt].duration = TX_BIT_DIVIDER;
        rmt_data[cnt + 1].level = 1; // 0 -> encoded 01, 1 -> encoded 11
        rmt_data[cnt + 1].duration = TX_BIT_DIVIDER;
        data.val >>= 1;
        parity += (data.val & 1);
    }
    // parity bit
    rmt_data[cnt].level = parity & 1;
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 1;
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 1; // stop bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 1; // stop bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 1; // end transfer
    rmt_data[cnt].duration = 0;
    cnt++;
    rmt_data[cnt].level = 1; // end transfer
    rmt_data[cnt].duration = 0;
}
static void hbs_tx_packet_task(void *p)
{
    rmt_item32_t rmt_item[14]; // 14*2 -> 28 bit ( with 00 end transfer )
    rmt_item16_t *rmt_data = (rmt_item16_t *)rmt_item;
    int cnt = 0;
    hbs_packet_t packet = {0};
    while (1)
    {
        xQueueReceive(hbs_tx_packet_queue, &packet, portMAX_DELAY);
        for (cnt = 0; cnt < packet.packet_hdr.packet_size; cnt++)
        {
            hbs_item_to_rmt_item_cvt(rmt_data, packet.packet_data[cnt]);
            rmt_write_items(TX_CHANNEL, rmt_item, 14, 1); // start & wait done
        }
        xEventGroupSetBits(hbs_tx_event_group, hbs_TX_DONE_BIT);
    }
}
#if 0
static void hbs_tx_packet_tx(hbs_packet_t *packet)
{
    rmt_item32_t rmt_item[14];  // 14*2 -> 28 bit ( with 00 end transfer )
    rmt_item16_t *rmt_data = (rmt_item16_t *)rmt_item;
    int cnt = 0;
        for (cnt = 0; cnt < packet->packet_hdr.packet_size; cnt++)
        {
            hbs_item_to_rmt_item_cvt(rmt_data, packet->packet_data[cnt]);
            rmt_write_items(TX_CHANNEL, rmt_item, 14, 1); // start & wait done
        }
}
#endif

esp_err_t hbs_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{

    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(tx_pin, TX_CHANNEL);
    rmt_tx_config.clk_div = RMT_TX_DIV;
    rmt_tx_config.mem_block_num = 1;
    rmt_tx_config.tx_config.loop_count = 1;
    rmt_tx_config.tx_config.carrier_en = 0;
    rmt_tx_config.tx_config.loop_en = 0;
    rmt_tx_config.tx_config.idle_level = 1;
    rmt_tx_config.tx_config.idle_output_en = 1;
    //
    // rmt_tx_config.flags=RMT_CHANNEL_FLAGS_INVERT_SIG;
    //
    hbs_tx_event_group = xEventGroupCreate();
    hbs_tx_packet_queue = xQueueCreate(1, sizeof(hbs_packet_t));
    rmt_config(&rmt_tx_config);
    rmt_driver_install(TX_CHANNEL, 0, 0);
    xTaskCreate(hbs_tx_packet_task, "rmt tx", 4096, NULL, 5, &hbs_tx_packet_task_handle);

    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(rx_pin, RX_CHANNEL);
    rmt_rx_config.clk_div = RMT_RX_DIV;
    rmt_rx_config.mem_block_num = 7;
    rmt_rx_config.rx_config.idle_threshold = RMT_RX_IDLE_THRES;
    rmt_rx_config.rx_config.filter_en = true;
    rmt_rx_config.rx_config.filter_ticks_thresh = 40; // increase value if log dur = 1. thresold tick = 12.5nS ( APB clock = 80 mHz )
    //
    // rmt_rx_config.flags=RMT_CHANNEL_FLAGS_INVERT_SIG;
    //
    hbs_rx_packet_queue = xQueueCreate(4, sizeof(hbs_packet_t));
    rmt_config(&rmt_rx_config);
    rmt_driver_install(RX_CHANNEL, 4096 * 4, 0);
    xTaskCreate(hbs_rx_packet_task, "rmt rx", 4096, NULL, 5, &hbs_rx_packet_task_handle);

    return ESP_OK;
}
esp_err_t hbs_deinit(void)
{
    vTaskDelete(hbs_rx_packet_task_handle);
    rmt_driver_uninstall(RX_CHANNEL);
    vQueueDelete(hbs_rx_packet_queue);

    vTaskDelete(hbs_tx_packet_task_handle);
    rmt_driver_uninstall(TX_CHANNEL);
    vQueueDelete(hbs_tx_packet_queue);
    vEventGroupDelete(hbs_tx_event_group);

    return ESP_OK;
}
void hbs_tx_packet(hbs_packet_t *packet)
{
    xQueueSend(hbs_tx_packet_queue, packet, portMAX_DELAY);                                   // data send to tx queue, start transmit
    xEventGroupWaitBits(hbs_tx_event_group, hbs_TX_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY); // all data transmitted
#if 0
    hbs_tx_packet_tx(packet);
#endif
}
esp_err_t hbs_rx_packet(hbs_packet_t *packet, TickType_t wait_time)
{
    int ret = xQueueReceive(hbs_rx_packet_queue, packet, wait_time);
    if (ret != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
void hbs_clear_rx_queue(void)
{
    xQueueReset(hbs_rx_packet_queue);
}
