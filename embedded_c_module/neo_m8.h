#ifndef NEO_M8_H
#define NEO_M8_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/mphal.h"
#include "py/nlr.h"

// Constant definitions
#define CHAR_SIZE sizeof(char)

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
	mp_obj_t uart_bus;
	
	uint16_t buffer_len;
	char buffer[513];

	nmea_sentences_data data;
} neo_m8_obj_t;

// Function declarations
static int16_t find_in_char_array(char *array, uint16_t length, char character_to_look_for, int16_t starting_point);
static uint8_t nmea_checksum(char *nmea_sentence, uint8_t length);
static char* extract_timestamp(char *nmea_section);
static float extract_lat_long(char *nmea_section);
static void update_data(neo_m8_obj_t *self);

extern const mp_obj_type_t neo_m8_type;

#endif