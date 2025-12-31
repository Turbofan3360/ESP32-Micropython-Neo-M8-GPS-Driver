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

// Struct to return NMEA sentence data
typedef struct {
    uint8_t* sentence_start;
    uint8_t length;
} nmea_sentence_data_t;

// Struct to hold parsed data
typedef struct {
    float latitude;
    float longitude;
    float position_error;

    float altitude;
    float geosep;
    float vertical_error;

    float sog;
    float cog;

    char timestamp[8];
    uint32_t date;
} gps_data_t;

// Object definition
typedef struct {
	mp_obj_base_t base;
	uart_port_t uart_number;

	uint8_t buffer[INTERNAL_BUFFER_LENGTH];
	uint16_t buffer_length;

    gps_data_t data;
} neo_m8_obj_t;

// Function declarations
static int16_t find_in_char_array(char *array, uint16_t length, char character_to_look_for, int16_t starting_point);
static uint8_t nmea_checksum(char *nmea_sentence, uint8_t length);
static int8_t ubx_ack_nack(neo_m8_obj_t *self);
static void update_buffer_internal(neo_m8_obj_t* self);
static void extract_timestamp(char* nmea_section, char* timestamp_out);
static void extract_lat_long(char* nmea_section, float* output);
static void get_sentence(neo_m8_obj_t *self, nmea_sentence_data* output, char* desired_sentence);

static int8_t parse_gga(neo_m8_obj_t* self);
static int8_t parse_rmc(neo_m8_obj_t* self);

extern const mp_obj_type_t neo_m8_type;

#endif
