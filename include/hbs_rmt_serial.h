#include "driver/gpio.h"
// hbs item
#define MAX_HBS_PACKET_SIZE 26
typedef struct
{
    union
    {
        struct
        {
            uint16_t data : 8;  // hbs data 
            uint16_t parity : 1;   // hbs cmd  
            uint16_t spare : 7; // spare for future
        };
        uint16_t val; // Equivalent unsigned value for the hbs item 
    };
} hbs_item16_t;
// hbs packet header
typedef struct 
{
    union
    {
        struct
        {
            uint16_t packet_size : 8; // transmit/receive packet size
            uint16_t spare : 8;       // spare for future  
        };
        uint16_t value; // Equivalent unsigned value for the hbs packet header
    };
} hbs_packet_hdr_t;
// hbs packet struct (header+items)
typedef struct
{
    hbs_packet_hdr_t packet_hdr; // hbs packet header
    hbs_item16_t packet_data[MAX_HBS_PACKET_SIZE];// hbs packet data
} hbs_packet_t;

esp_err_t hbs_init(gpio_num_t rx_pin, gpio_num_t tx_pin);   // init hbs/rmt task and queue
esp_err_t hbs_deinit(void);                                 // deinit hbs/rmt task and queue
void      hbs_tx_packet(hbs_packet_t *packet);              // send hbs packet ( return after all data transmitted )
esp_err_t hbs_rx_packet(hbs_packet_t *packet, TickType_t wait_time); // receive hbs packet
void      hbs_clear_rx_queue(void);                         // clear rx buffer

#define DBG 1

#if DBG

#define TX_TEST_GPIO (5)//for c3 !! esp32->(25)
#define RX_TEST_GPIO (6)//for c3 !! esp32->(26)

#endif