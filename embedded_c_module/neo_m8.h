#ifndef NEO_M8_H
#define NEO_M8_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/mphal.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "esp_timer.h"

// Constant definitions
#define CHAR_PTR_SIZE sizeof(char*)
#define FLOAT_SIZE sizeof(float)
#define INTERNAL_BUFFER_LENGTH 512

// Struct to store the NMEA sentences read from the module
typedef struct {
	char *gll;
	char *gsa;
	char *gga;
	char *rmc;
} nmea_sentences_data;

// Object definition
typedef struct {
	mp_obj_base_t base;
	uart_port_t uart_number;
	nmea_sentences_data data;

	uint8_t buffer[INTERNAL_BUFFER_LENGTH];
	uint16_t buffer_length;
} neo_m8_obj_t;

// Function declarations
static int16_t find_in_char_array(char *array, uint16_t length, char character_to_look_for, int16_t starting_point);
static uint8_t nmea_checksum(char *nmea_sentence, uint8_t length);
static int8_t ubx_ack_nack(neo_m8_obj_t *self);
static void update_buffer_internal(uart_port_t uart_num, uint16_t* length, uint8_t* buffer);
static void extract_timestamp(char* nmea_section, char* timestamp_out);
static void extract_lat_long(char* nmea_section, float* output);
static void update_data(neo_m8_obj_t *self);

extern const mp_obj_type_t neo_m8_type;

#endif