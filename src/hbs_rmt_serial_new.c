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

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include "hbs_rmt_serial.h"

static const char *TAG = "HBS RMT";

/*
 * Divider set
 * 9600-1%+2% about 9500-9800 - hbs baud standart
 * tx baud calculate 80 000 000 / RMT_TX_DIV / TX_BIT_DIVIDER
 * rx bit divider should be a little more then max tx rate(9800)
 * RX_BIT_DIVIDER = ( 80 000 000/19200/RMT_RX_DIV ) = 100-104
 * RMT_RX_IDLE_THRES may be  greater then (80 000 000/RX_BIT_DIVIDER/9500*11) = 1158(1200-2400)
 */

#define RMT_RX_DIV (40) // 8

#define RMT_RX_IDLE_THRES (5*1000*1000) // nanosek
#define RMT_RX_GLITCH_FILTER (1*1000)   //nanosek
#define RX_BIT_DIVIDER (104)     // 1040
#define RMT_RX_CLK_OUT (80 * 1000 * 1000 / RMT_RX_DIV)

#define RX_INVERT_LVL 0


#ifdef CONFIG_IDF_TARGET_ESP32
#define RX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*7)      // 7 rmt channel memory
#define TX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*1)      // 1 rmt channel memory always ping/pong
#else // ESP32-S3 ESP32-C3 -> PINGPONG RX
#define RX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*2)      // 2  rmt channel memory ping/pong
#define TX_BLOCK_SYMBOL (SOC_RMT_MEM_WORDS_PER_CHANNEL*2)      // 2 rmt channel memory always ping/pong
#endif


// tx baud=80000000/40/104 = 9600 baud
#define RMT_TX_DIV (40) // 8 // esp32_hbs = 82
#define RMT_TX_CLK_OUT (80 * 1000 * 1000 / RMT_TX_DIV)
#define TX_BIT_DIVIDER (104) // 1042 // esp32_hbs=100

#define BIT_IN_WORD (22) // -> for hsb start(2)+18 bit+stop(2)
                         // -> For hsb_rmt start(1 bit) + 20 bit (msb already 1, 16(8*2) bit data, 1(2) bit parity, lsb already 1 ) + stop(1 bit)
                         // -> for data start(1 bit) + 9 bit + stop(1 bit)

static QueueHandle_t hbs_rx_packet_queue;
static QueueHandle_t hbs_tx_packet_queue;
static TaskHandle_t hbs_rx_packet_task_handle;
static TaskHandle_t hbs_tx_packet_task_handle;

static rmt_channel_handle_t tx_chan_handle = NULL;
static rmt_encoder_handle_t tx_encoder = NULL;
static rmt_channel_handle_t rx_chan = NULL;
static QueueHandle_t receive_queue;

#define HBS_TX_DONE_BIT BIT0
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

static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}

static rmt_item16_t rmt_rx_items[MAX_HBS_PACKET_SIZE * BIT_IN_WORD + 2] = {0}; // for software glitch detect must be * 2
static void hbs_rx_packet_task(void *p)
{
    rmt_rx_done_event_data_t rx_data;
    size_t length = 0;
    uint32_t hbs_data_bit = 0; // 22 bit from hbs
    // hbs_item16_t data = {0};
    hbs_packet_t packet = {0};
    int cnt_bit = 0;  // wait start bit, bit count
    int cnt_byte = 0; // byte in packet

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = RMT_RX_GLITCH_FILTER,        // the shortest duration
        .signal_range_max_ns = RMT_RX_IDLE_THRES,            // the longest duration
#if SOC_RMT_SUPPORT_RX_PINGPONG        
        .flags.en_partial_rx = true,
#endif        
    };
    while (1)
    {
        cnt_byte = 0;
        cnt_bit = 0; // wait next start bit
        memset(&packet, 0, sizeof(packet));
        memset(rmt_rx_items,0,sizeof(rmt_rx_items));
        ESP_ERROR_CHECK(rmt_receive(rx_chan, rmt_rx_items, sizeof(rmt_rx_items), &receive_config));
        if (xQueueReceive(receive_queue, &rx_data, portMAX_DELAY) == pdTRUE)
        {
            // ESP_LOGI(TAG, "receive %d symbols iptr = %p dptr = %p", rx_data.num_symbols, rx_data.received_symbols, rmt_rx_items);

            length = rx_data.num_symbols;

            length *= 2; // one RMT = 2 Bytes
            for (int i = 0; i < length; i++)
            {
#if RX_INVERT_LVL
                int lvl = (!rmt_rx_items[i].level) & 1; // invert lvl
#else
                int lvl = (rmt_rx_items[i].level) & 1;
#endif
                int duration = (rmt_rx_items[i].duration + RX_BIT_DIVIDER / 2) / RX_BIT_DIVIDER;
                // ESP_LOGI(TAG, "%d lvl=%d, bit_in=%d,dur=%d", i, lvl, duration, rmt_rx_items[i].duration);
                if (cnt_bit == 0) // start bit
                {
                    if (lvl == 0 && duration > 0 && duration < BIT_IN_WORD) // start bit
                    {
                        hbs_data_bit = 0;   // first  bits in byte
                        cnt_bit = duration; // start bit + some bits=0
                    }
                    else
                    {
                        ESP_LOGE(TAG, "receive frame err START bit %d lvl=%d, bit_in=%d,dur=%d", i, lvl, duration, rmt_rx_items[i].duration);
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
    }
}
static void hbs_item_to_rmt_item_cvt(rmt_item16_t *rmt_data, hbs_item16_t data)
{
    int cnt = 0;
    int parity = 0;          // parity bit calculate on cvt -> hbs_item16_t parity ignored
    rmt_data[cnt].level = 1; // start bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 0; // start bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    for (; cnt < BIT_IN_WORD - 4; cnt += 2) // 22 - 2 (start) - 2 (parity)  -> lsb first
    {
        rmt_data[cnt].level = (~data.val) & 1;
        rmt_data[cnt].duration = TX_BIT_DIVIDER;
        rmt_data[cnt + 1].level = 0; // 0 -> encoded 01, 1 -> encoded 11
        rmt_data[cnt + 1].duration = TX_BIT_DIVIDER;
        data.val >>= 1;
        parity += (data.val & 1);
    }
    // parity bit
    rmt_data[cnt].level = (~parity) & 1;
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 0;
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 0; // stop bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 0; // stop bit
    rmt_data[cnt].duration = TX_BIT_DIVIDER;
    cnt++;
    rmt_data[cnt].level = 0; // end transfer
    rmt_data[cnt].duration = 0;
    cnt++;
    rmt_data[cnt].level = 0; // end transfer
    rmt_data[cnt].duration = 0;
}
static rmt_item16_t rmt_tx_items[MAX_HBS_PACKET_SIZE * BIT_IN_WORD + 2] = {0}; // for software glitch detect must be * 2

static void hbs_tx_packet_task(void *p)
{
    int cnt = 0;
    hbs_packet_t packet = {0};
    rmt_transmit_config_t rmt_tx_config = {
        .loop_count = 0,
    };
    while (1)
    {
        xQueueReceive(hbs_tx_packet_queue, &packet, portMAX_DELAY);
        for (cnt = 0; cnt < packet.packet_hdr.packet_size; cnt++)
        {
            hbs_item_to_rmt_item_cvt(rmt_tx_items+(cnt*BIT_IN_WORD), packet.packet_data[cnt]);
        }
        ESP_ERROR_CHECK(rmt_transmit(tx_chan_handle, tx_encoder, rmt_tx_items, sizeof(rmt_tx_items), &rmt_tx_config));
        rmt_tx_wait_all_done(tx_chan_handle, portMAX_DELAY);
        xEventGroupSetBits(hbs_tx_event_group, HBS_TX_DONE_BIT);
    }
}

esp_err_t hbs_init(gpio_num_t rx_pin, gpio_num_t tx_pin)
{

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = tx_pin,
        .mem_block_symbols = TX_BLOCK_SYMBOL, //for esp32c3 !!!!!!! esp32s -> 64 !!!!!!! // 16*2 bit = 32 bit
        .flags.invert_out = true,
        .resolution_hz = RMT_TX_CLK_OUT,
        .trans_queue_depth = 5, // set the maximum number of transactions that can pend in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan_handle));

    rmt_copy_encoder_config_t tx_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&tx_encoder_config, &tx_encoder));
    ESP_ERROR_CHECK(rmt_enable(tx_chan_handle));
    //
    hbs_tx_event_group = xEventGroupCreate();
    hbs_tx_packet_queue = xQueueCreate(1, sizeof(hbs_packet_t));
    xTaskCreate(hbs_tx_packet_task, "rmt tx", 4096, NULL, 5, &hbs_tx_packet_task_handle);

    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,  // select source clock
        .resolution_hz = RMT_RX_CLK_OUT, // tick resolution,
        .mem_block_symbols = RX_BLOCK_SYMBOL,// for esp32c3 !!!!!!! esp32s -> 338 !!!!!!!        // memory block size, 338*4 = 676 bytes -> 26 bit in msg * 26 msg in packet =
        .gpio_num = rx_pin,              // GPIO number
        .flags.invert_in = false,        // do not invert input signal
        .flags.with_dma = false,         // do not need DMA backend
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));
    receive_queue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_chan));
    //
    hbs_rx_packet_queue = xQueueCreate(4, sizeof(hbs_packet_t));
    xTaskCreate(hbs_rx_packet_task, "rmt rx", 4096, NULL, 5, &hbs_rx_packet_task_handle);

    return ESP_OK;
}
esp_err_t hbs_deinit(void)
{
    // deinit RX
    vTaskDelete(hbs_rx_packet_task_handle);

    rmt_disable(rx_chan);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = NULL,
    };
    rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue);
    rmt_del_channel(rx_chan);
    vQueueDelete(receive_queue);

    // deinit TX
    vQueueDelete(hbs_rx_packet_queue);

    vTaskDelete(hbs_tx_packet_task_handle);

    rmt_disable(tx_chan_handle);
    rmt_del_encoder(tx_encoder);
    rmt_del_channel(tx_chan_handle);

    vQueueDelete(hbs_tx_packet_queue);
    vEventGroupDelete(hbs_tx_event_group);

    return ESP_OK;
}
void hbs_tx_packet(hbs_packet_t *packet)
{
    xQueueSend(hbs_tx_packet_queue, packet, portMAX_DELAY);                                   // data send to tx queue, start transmit
    xEventGroupWaitBits(hbs_tx_event_group, HBS_TX_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY); // all data transmitted
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
