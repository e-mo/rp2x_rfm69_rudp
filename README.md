# rp2x_rfm69_rudp v0.1.0
This is a Reliable UDP interface library for reliable data transmission between RFM69 transceivers controlled by an RP2XXX series MCU.   

## Wisdom Transmission Protocol
The underlying communication protocol (WTP) is one that I developed for my Wisdom communcation board. I did it because I had never designed and implemented a protocol, and I wanted to. Good reason to do anything, right?  

[WTP_1.0 Specification](docs/WTP_specification-1_0.txt)

## Interface
### rudp_init
**description:** Initializes the Rfm69 hardware and puts it in the correct state for usage with the RUDP interface.
**return:** `true` if init was successful.  
**error:** Returns `false` if init fails.  
```
bool rudp_init(rudp_context_t *rudp, const struct rudp_config_s *config);
```
**usage:** This MUST BE CALLED to initialize rudp context object.
```c
struct rudp_config_s {
    rfm69_context_t *rfm;
    struct rfm69_config_s *rfm_config;
    // Number of retries of a packet before failure in tx
    uint32_t tx_resend_max;
    // How long the TX side waits for a response before resending
    uint32_t tx_resend_timeout;
    // How long RX side will wait for first packet
    uint32_t rx_wait_timeout;
    // How long RX will wait for next packet before declaring
    // transmission dropped
    uint32_t rx_drop_timeout;
};
```
```c
// rudp_init example

int main() {
    // SPI init
    spi_init(RFM69_SPI, 1000*1000);
    gpio_set_function(RFM69_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(RFM69_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(RFM69_PIN_MOSI, GPIO_FUNC_SPI);

    // Drive CS pin high
    gpio_init(RFM69_PIN_CS);
    gpio_set_dir(RFM69_PIN_CS, GPIO_OUT);
    gpio_put(RFM69_PIN_CS, 1);

	// DIO 0-2 must be defined to use interrupts instead of polling
    struct rfm69_config_s rfm_config = {
        .spi      = RFM69_SPI,
        .pin_cs   = RFM69_PIN_CS,
        .pin_rst  = RFM69_PIN_RST,
        .pin_dio0 = RFM69_PIN_DIO0,
        .pin_dio1 = RFM69_PIN_DIO1,
        .pin_dio2 = RFM69_PIN_DIO2
    };

    // Can rely on default RUDP values in config but must provide rfm69 instance
	// and config
    rfm69_context_t rfm;
    struct rudp_config_s rudp_config = {
        .rfm = &rfm,
        .rfm_config = &rfm_config
    };

    rudp_context_t rudp;
    if (rudp_init(&rudp, &rudp_config) == false) {
        // Handle init error here
    }
}
```
