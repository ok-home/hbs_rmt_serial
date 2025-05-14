# Simple exchange according to the HBS standard (ESP32, ESP32C3)
  - HBS - 22 BIT in message (01 - start, 01/11 - bit value, 01/11 - parity bit, 11 - stop bit)
  - 8-bit+parity/9600 baud only, using RMT 
  - maximum data packet size 32 hbs_item16_t records
## interface functions
```
esp_err_t hbs_init(gpio_num_t rx_pin, gpio_num_t tx_pin);   // init hbs/rmt task and queue
esp_err_t hbs_deinit(void);                                 // deinit hbs/rmt task and queue
void      hbs_tx_packet(hbs_packet_t *packet);              // send hbs packet ( return after all data transmitted )
esp_err_t hbs_rx_packet(hbs_packet_t *packet, TickType_t wait_time); // receive hbs packet
void      hbs_clear_rx_queue(void);                         // clear rx buffer
```
## transmit/receive data format
```
// hbs item
typedef struct
{
    union
    {
        struct
        {
            uint16_t data : 8;  // hbs data 
            uint16_t parity : 1;   // hbs parity  
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
```
## connects as a standard ESP-IDF component
## Simple example of use
  - hbs_rmt_serial_example/main/hbs_rmt_serial_example.c
