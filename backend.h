#ifndef BACKEND_H
#define BACKEND_H

#include <stdint.h>
#include <avr/io.h>

#define RST PB1
#define SS PB2
#define MOSI PB3
#define MISO PB4
#define SCK PB5

#define STEPS_MIN 69
#define STEPS_MAX 119
#define STEPS_AVG 94
#define COUNTER_MAX 188
#define DATA_MAX 40

#define LED_COUNT 4
#define RED PC0
#define YLW PC1
#define GRN PC2
#define BLU PC3

/************************* SPI ************************/
void spi_setup();

void spi_write_reg(uint8_t address, uint8_t value);

void spi_write_n(uint8_t address, uint8_t* data, uint8_t len);

uint8_t spi_read_reg(uint8_t address);

void spi_read_n(uint8_t address, uint8_t* data, uint8_t len);


/************************* PWM ************************/
void pwm_setup();

void set_speed(uint8_t s);


/************************* LED ************************/
void led_setup();

void led_set(uint8_t bitmask);

void led_toggle(uint8_t color);

#endif // BACKEND_H
