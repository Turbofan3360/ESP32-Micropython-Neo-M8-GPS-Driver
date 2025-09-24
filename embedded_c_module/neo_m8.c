#include "neo_m8.h"

mp_obj_t neo_m8_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
	// Checking arguments
	mp_arg_check_num(n_args, n_kw, 1, 1, false);

	// Testing to see if the UART object is valid (i.e. the required methods can be loaded from it)
	nlr_buf_t cpu_state;
	mp_obj_t test_method[2];

	if (nlr_push(&cpu_state) == 0){
		mp_load_method(args[0], MP_QSTR_any, test_method);
		mp_load_method(args[0], MP_QSTR_read, test_method);
		mp_load_method(args[0], MP_QSTR_write, test_method);
	}
	else {
		mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("UART bus object not valid"));
	}

	// Creating and allocating memory to the "self" instance of this module
	neo_m8_obj_t *self = m_new_obj(neo_m8_obj_t);

	// Initialising required data in the "self" object
	self->base.type = &neo_m8_type;
	self->uart_bus = args[0];
	self->buffer_len = 0;

	return MP_OBJ_FROM_PTR(self);
}

static int16_t find_in_char_array(char *array, uint16_t length, char character_to_look_for, int16_t starting_point){
	uint16_t i;

	// Making sure the starting point is valid - some cases mean that starting_point might be passed in as -1
	if (starting_point < 0){
		starting_point = 0;
	}

	// Utility to search through a string (char array) for a specific character and return the index
	for (i = starting_point; i < length; i++){
		if (array[i] == character_to_look_for){
			return i;
		}
	}

	return -1;
}

static uint8_t nmea_checksum(char *nmea_sentence, uint8_t length){
	uint8_t i, checksum_pos, checksum_calc, checksum_sentence;

	// Finding where the checksum starts
	checksum_pos = find_in_char_array(nmea_sentence, length, '*', 0);

	// Getting the checksum from the NMEA sentence (strtol converts from hex string to int)
	char hex_checksum[3] = {nmea_sentence[checksum_pos+1], nmea_sentence[checksum_pos+2], '\0'};
	checksum_sentence = strtol(hex_checksum, NULL, 16);

	for (i = 1; i < checksum_pos; i++){
		// Calculating the checksum
		checksum_calc ^= nmea_sentence[i];
	}

	if (checksum_calc == checksum_sentence){
		return 1;
	}
	else{
		return 0;
	}
}

static void update_data(neo_m8_obj_t *self){
	mp_obj_t self_mp = MP_OBJ_FROM_PTR(self);

	uint8_t sentences_read = 0;
	int16_t start_pos = -1, end_pos = -1;
	char *nmea_sentence_type[3]

	while (sentences_read < 5){
		update_buffer(self_mp);

		while (sentences_read < 5){
			start_pos = find_in_char_array(self->buffer, self->buffer_len, '$', 0);
			end_pos = find_in_char_array(self->buffer, self->buffer_len, '\n', start_pos);

			if ((start_pos == -1) || (end_pos == -1)){
				break;
			}

			// Allocating memory to and copying the NMEA sentence it's found into a temporary variable
			char *data_section = (char *) malloc((end_pos - start_pos)*CHAR_SIZE);
			strncpy(data_section, self->buffer + start_pos, end_pos - start_pos);

			// Removing the section of the buffer used by the data_section NMEA sentence
			memmove(self->buffer, self->buffer + end_pos, self->buffer_len - end_pos);
			self->buffer_len -= end_pos

			// Checking the NMEA checksum
			if (nmea_checksum(data_section, end_pos-start_pos) == 0){
				break;
			}

			// Finding the type of NMEA sentence it is, then saving it into memory
			strncpy(nmea_sentence_type, data_section[1], 3);

			if (nmea_sentence_type == "GLL"){
				self->data->gll = *data_section;
			}
			else if(nmea_sentence_type = "GSA"){
				self->data->gsa = *data_section;
			}
			else if(nmea_sentence_type = "GGA"){
				self->data->gga = *data_section;
			}
			else if(nmea_sentence_type = "RMC"){
				self->data->rmc = *data_section;
			}

			sentences_read++;
			free(data_section);
		}
	}
}

mp_obj_t update_buffer(mp_obj_t self_in){
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

	mp_buffer_info_t buf_info;
	mp_obj_t any_method[2], read_method[2];
	uint16_t copy_start, new_bytes_copy_start = 0;

	// Loading required methods
	mp_load_method(self->uart_bus, MP_QSTR_any, any_method);
	mp_load_method(self->uart_bus, MP_QSTR_read, read_method);

	// Checking if there's any data available
	if (mp_obj_get_int(mp_call_method_n_kw(0, 0, any_method)) == 0){
		return mp_const_none;
	}

	// Getting new UART data and then buffer info from it
	mp_obj_t bytes = mp_call_method_n_kw(0, 0, read_method);
	mp_get_buffer_raise(bytes, &buf_info, MP_BUFFER_READ);

	// Finding total number of bytes held
	uint16_t buffer_total = self->buffer_len + buf_info.len;

	// If there's too many bytes to fill the buffer, then slide the window
	if (buffer_total > 512){
		// Finding the point where the most recent 512 bytes starts
		copy_start = buffer_total - 512;

		// If the most recent 512 bytes starts before the current buffer ends, then copy from old buffer into new one
		if (copy_start < self->buffer_len){
			uint16_t old_bytes_copy = self->buffer_len - copy_start;

			// Copying the required bytes from the current buffer into the new buffer
			memmove(self->buffer, self->buffer + copy_start, old_bytes_copy);

			// Updating the buffer length
			self->buffer_len = old_bytes_copy;
		}
		// If the most recent 512 bytes is after the current buffer window, then discard all buffer data
		else {
			new_bytes_copy_start = buf_info.len - 512;
			self->buffer_len = 0;
		}
	}

	// Working out the number of new bytes to copy
	uint16_t new_bytes_copy = buf_info.len - new_bytes_copy_start;

	// Copying bytes from the new data into the buffer
	memcpy(self->buffer + self->buffer_len, buf_info.buf + new_bytes_copy_start, new_bytes_copy);

	// Updating buffer length
	self->buffer_len += new_bytes_copy;

	// Ensuring termination character added
	self->buffer[self->buffer_len] = '\0';

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJECT_1(neo_m8_update_buffer_obj, update_buffer)





/**
 * Code here exposes the module functions above to micropython as an object
*/

// Defining the functions that are exposed to micropython
static const mp_rom_map_elem_t neo_m8_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_update_buffer), MP_ROM_PTR(&neo_m8_update_buffer_obj)},
};
static MP_DEFINE_CONST_DICT(neo_m8_locals_dict, neo_m8_locals_dict_table);

// Overall module definition
MP_DEFINE_CONST_OBJ_TYPE(
    neo_m8_type,
    MP_QSTR_neo_m8,
    MP_TYPE_FLAG_NONE,
    make_new, neo_m8_make_new,
    locals_dict, &neo_m8_locals_dict
);

// Defining global constants
static const mp_rom_map_elem_t neo_m8_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__) , MP_ROM_QSTR(MP_QSTR_neo_m8) },
    { MP_ROM_QSTR(MP_QSTR_neo_m8), MP_ROM_PTR(&neo_m8_type) },
};
static MP_DEFINE_CONST_DICT(neo_m8_globals_table, neo_m8_module_globals_table);

// Creating module object
const mp_obj_module_t neo_m8_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&neo_m8_globals_table,
};

MP_REGISTER_MODULE(MP_QSTR_neo_m8, neo_m8_module);