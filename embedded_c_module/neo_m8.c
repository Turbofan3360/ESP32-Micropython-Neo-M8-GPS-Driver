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

		nlr_pop();
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

	self->data.gll = NULL;
	self->data.gsa = NULL;
	self->data.gga = NULL;
	self->data.rmc = NULL;

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
	int16_t checksum_pos;
	uint8_t i, checksum_calc = 0, checksum_sentence;

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
	uint8_t sentences_read = 0;
	int16_t start_pos = -1, end_pos = -1;
	uint32_t start_time = mp_hal_ticks_ms();
	char nmea_sentence_type[4];

	while (sentences_read < 5){
		// Timeout for the function. If it's running for more than 1 second, then the function quits and raises an error
		if (mp_hal_ticks_ms() - start_time > 1500){
			mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Function timed out - no valid NMEA data found in buffer."));
		}

		start_pos = find_in_char_array(self->buffer, self->buffer_len, '$', 0);
		end_pos = find_in_char_array(self->buffer, self->buffer_len, '\n', start_pos);

		if ((start_pos == -1) || (end_pos == -1)){
			update_buffer_internal(self);
			continue;
		}

		// Allocating memory to and copying the NMEA sentence it's found into a temporary variable
		size_t sentence_length = end_pos - start_pos;
		char *data_section = (char *) malloc(sentence_length*CHAR_SIZE + 1);

		if (data_section == NULL){
			mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
		}

		strncpy(data_section, self->buffer + start_pos, sentence_length);
		data_section[sentence_length] = '\0';

		// Removing the section of the buffer used by the data_section NMEA sentence
		memmove(self->buffer, self->buffer + end_pos, self->buffer_len - end_pos);
		self->buffer_len -= end_pos;

		// Checking the NMEA checksum
		if (nmea_checksum(data_section, end_pos-start_pos) == 0){
			continue;
		}

		// Finding the type of NMEA sentence it is, then saving it into memory
		strncpy(nmea_sentence_type, data_section + 3, 3);
		nmea_sentence_type[3] = '\0';
		size_t str_length = strlen(data_section);

		if (strcmp(nmea_sentence_type, "GLL") == 0){
			self->data.gll = (char *) realloc(self->data.gll, str_length*CHAR_SIZE + 1);

			if (self->data.gll == NULL){
				// Error: out of memory
				free(data_section);
				mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
			}

			strncpy(self->data.gll, data_section, str_length);
			self->data.gll[str_length] = '\0';
		}
		else if(strcmp(nmea_sentence_type, "GSA") == 0){
			self->data.gsa = (char *) realloc(self->data.gsa, str_length*CHAR_SIZE + 1);

			if (self->data.gsa == NULL){
				// Error: out of memory
				free(data_section);
				mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
			}

			strncpy(self->data.gsa, data_section, str_length);
			self->data.gsa[str_length] = '\0';
		}
		else if(strcmp(nmea_sentence_type, "GGA") == 0){
			self->data.gga = (char *) realloc(self->data.gga, str_length*CHAR_SIZE + 1);

			if (self->data.gga == NULL){
				// Error: out of memory
				free(data_section);
				mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
			}

			strncpy(self->data.gga, data_section, str_length);
			self->data.gga[str_length] = '\0';
		}
		else if(strcmp(nmea_sentence_type, "RMC") == 0){
			self->data.rmc = (char *) realloc(self->data.rmc, str_length*CHAR_SIZE + 1);

			if (self->data.rmc == NULL){
				// Error: out of memory
				free(data_section);
				mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
			}

			strncpy(self->data.rmc, data_section, str_length);
			self->data.rmc[str_length] = '\0';
		}

		sentences_read++;
		free(data_section);
	}
}

static char* extract_timestamp(char *nmea_section){
	char *timestamp = (char *) malloc(9*CHAR_SIZE);

	if (timestamp == NULL){
		mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
	}

	timestamp[0] = nmea_section[0];
	timestamp[1] = nmea_section[1];
	timestamp[2] = ':';
	timestamp[3] = nmea_section[2];
	timestamp[4] = nmea_section[3];
	timestamp[5] = ':';
	timestamp[6] = nmea_section[4];
	timestamp[7] = nmea_section[5];
	timestamp[8] = '\0';

	return timestamp;
}

static float* extract_lat_long(char *nmea_section){
	uint8_t i;
	int8_t pos_degrees_end, degrees;
	float minutes;
	float *total = (float *) malloc(FLOAT_SIZE);

	if (total == NULL){
		mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
	}

	size_t length = strlen(nmea_section);

	// Finding the end of the degrees part of the lat/long string
	pos_degrees_end = find_in_char_array(nmea_section, length, '.', 0);

	if (pos_degrees_end <= 1){
		mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid NMEA sentence input"));
	}

	// Extracting the degrees value
	pos_degrees_end -= 2;
	char degrees_char[pos_degrees_end+1];
	degrees_char[pos_degrees_end] = '\0';

	for (i = 0; i < pos_degrees_end; i++){
		degrees_char[i] = nmea_section[i];
	}

	degrees = atoi(degrees_char);

	// Extracting the minutes value
	char minutes_char[length - pos_degrees_end + 1];
	minutes_char[length - pos_degrees_end] = '\0';

	for (i = pos_degrees_end; i < length; i++){
		minutes_char[i - pos_degrees_end] = nmea_section[i];
	}

	minutes = atof(minutes_char);

	// Combining them
	*total = (degrees + minutes/60);

	return total;
}

static void update_buffer_internal(neo_m8_obj_t *self){
	mp_buffer_info_t buf_info;
	mp_obj_t any_method[2], read_method[2];
	uint16_t copy_start, new_bytes_copy_start = 0;
	nlr_buf_t cpu_state;

	// Loading required methods
	mp_load_method(self->uart_bus, MP_QSTR_any, any_method);
	mp_load_method(self->uart_bus, MP_QSTR_read, read_method);

	if (nlr_push(&cpu_state) == 0){
		// Checking if there's any data available
		if (mp_obj_get_int(mp_call_method_n_kw(0, 0, any_method)) == 0){
			nlr_pop();
			return;
		}

		// Getting new UART data and then buffer info from it
		mp_obj_t bytes = mp_call_method_n_kw(0, 0, read_method);
		mp_get_buffer_raise(bytes, &buf_info, MP_BUFFER_READ);

		nlr_pop();
	}
	else {
		mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("UART method failed"));
	}

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

	return;
}

mp_obj_t update_buffer(mp_obj_t self_in){
	/**
	 * Micropython-exposed method to call the internal C buffer update method
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

	update_buffer_internal(self);

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_update_buffer_obj, update_buffer);

mp_obj_t position(mp_obj_t self_in){
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

	mp_obj_t retvals;
	uint8_t i;
	char *gga_split[9], *timestamp;
	float *latitude, *longitude, pos_error;

	update_data(self);

	// Splitting the GLL sentence up into sections, which can then be processed
	char *token = strtok(self->data.gga, ",");
	for (i = 0; token != NULL; i++){
		gga_split[i] = token;

		token = strtok(NULL, ",");
	}

	// If there aren't enough fields (i.e. incomplete sentence, bad data) OR status flag indicates bad fix, then return zeros
	if ((i < 8) || (strcmp(gga_split[6], "1") != 0)){
		return mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

	// Extracting GMT timestamp in hh:mm:ss format
	timestamp = extract_timestamp(gga_split[1]);

	// Extracting latitude in degrees decimal minutes
	latitude = extract_lat_long(gga_split[2]);

	if (strcmp(gga_split[3], "S") == 0){
		*latitude *= -1;
	}

	// Extracting longitude in degrees decimal minutes
	longitude = extract_lat_long(gga_split[4]);

	if (strcmp(gga_split[5], "W") == 0){
		*longitude *= -1;
	}

	// Extracting HDOP value, converting it to horizontal position error
	pos_error = atof(gga_split[8])*2.5;

	retvals = mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(*latitude), mp_obj_new_float(*longitude), mp_obj_new_float(pos_error), mp_obj_new_str(timestamp, 8)});

	free(timestamp);
	free(latitude);
	free(longitude);

	return retvals;
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_position_obj, position);

mp_obj_t velocity(mp_obj_t self_in){
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

	mp_obj_t retvals;
	uint8_t i;
	char *rmc_split[13], *timestamp;
	float sog, cog, mag_var;

	update_data(self);

	// Splitting the RMC sentence up into sections, which can then be processed
	char *token = strtok(self->data.rmc, ",");
	for (i = 0; token != NULL; i++){
		rmc_split[i] = token;

		token = strtok(NULL, ",");
	}

	if ((i < 8) || (strcmp(rmc_split[2], "A") != 0)){
		return mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

	// Extracting timestamp
	timestamp = extract_timestamp(rmc_split[1]);

	// Extracting SOG (knots)
	sog = atof(rmc_split[7]);

	// Extracting COG (degrees)
	cog = atof(rmc_split[8]);

	// Extracting magnetic variation (degrees)
	mag_var = atof(rmc_split[10]);

	if (strcmp(rmc_split[11], "W") == 0){
		mag_var *= -1;
	}

	retvals = mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(sog), mp_obj_new_float(cog), mp_obj_new_float(mag_var), mp_obj_new_str(timestamp, 8)});

	free(timestamp);

	return retvals;
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_velocity_obj, velocity);

mp_obj_t altitude(mp_obj_t self_in){
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

	mp_obj_t retvals;
	uint8_t i;
	char *gga_split[15], *timestamp, **gsa_split = NULL;
	float altitude, geosep, verterror;

	update_data(self);

	// Splitting the GGA sentence up into sections, which can then be processed
	char *token = strtok(self->data.gga, ",");
	for (i = 0; token != NULL; i++){
		gga_split[i] = token;

		token = strtok(NULL, ",");
	}

	if ((i < 13) || (strcmp(gga_split[6], "1") != 0)){
		return mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

	// Extracting timestamp
	timestamp = extract_timestamp(gga_split[1]);

	// Extracting altitude
	altitude = atof(gga_split[9]);

	// Extracting geoid separation
	geosep = atof(gga_split[11]);

	// Extracting vertical error
	token = strtok(self->data.gsa, ",");
	for (i = 0; token != NULL; i++){
		// Re-allocating extended memory - the length of the GSA sentence is unknown
		gsa_split = (char **) realloc(gsa_split, (i+1)*CHAR_PTR_SIZE);

		if (gsa_split == NULL){
			free(timestamp);
			mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
		}

		gsa_split[i] = token;

		token = strtok(NULL, ",");
	}

	verterror = atof(gsa_split[i-1])*5;

	retvals = mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(altitude), mp_obj_new_float(geosep), mp_obj_new_float(verterror), mp_obj_new_str(timestamp, 8)});

	free(timestamp);
	free(gsa_split);

	return retvals;
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_altitude_obj, altitude);

mp_obj_t getdata(mp_obj_t self_in){
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

	mp_obj_t retvals;
	uint8_t i;
	char *gga_split[15], *rmc_split[13], *timestamp, **gsa_split = NULL;
	float *latitude, *longitude, pos_error, altitude, geo_sep, verterror, sog, cog, mag_var;

	update_data(self);

	// Splitting the GGA sentence up into sections, which can then be processed
	char *token = strtok(self->data.gga, ",");
	for (i = 0; token != NULL; i++){
		gga_split[i] = token;

		token = strtok(NULL, ",");
	}

	if ((i < 13) || (strcmp(gga_split[6], "1") != 0)){
		return mp_obj_new_list(10, (mp_obj_t[10]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

	// Splitting the RMC sentence up into sections, which can then be processed
	token = strtok(self->data.rmc, ",");
	for (i = 0; token != NULL; i++){
		rmc_split[i] = token;

		token = strtok(NULL, ",");
	}

	if ((i < 8) || (strcmp(rmc_split[2], "A") != 0)){
		return mp_obj_new_list(10, (mp_obj_t[10]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_float(0.0f),
													mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

	// Extracting GMT timestamp in hh:mm:ss format
	timestamp = extract_timestamp(gga_split[1]);

	// Extracting latitude in degrees decimal minutes
	latitude = extract_lat_long(gga_split[2]);

	if (strcmp(gga_split[3], "S") == 0){
		*latitude *= -1;
	}

	// Extracting longitude in degrees decimal minutes
	longitude = extract_lat_long(gga_split[4]);

	if (strcmp(gga_split[5], "W") == 0){
		*longitude *= -1;
	}

	// Extracting HDOP value, converting it to horizontal position error
	pos_error = atof(gga_split[8])*2.5;

	// Extracting altitude
	altitude = atof(gga_split[9]);

	// Extracting geoid separation
	geo_sep = atof(gga_split[11]);

	// Extracting vertical error
	token = strtok(self->data.gsa, ",");
	for (i = 0; token != NULL; i++){
		// Re-allocating extended memory - the length of the GSA sentence is unknown
		gsa_split = (char **) realloc(gsa_split, (i+1)*CHAR_PTR_SIZE);

		if (gsa_split == NULL){
			free(timestamp);
			free(latitude);
			free(longitude);
			mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
		}

		gsa_split[i] = token;

		token = strtok(NULL, ",");
	}

	verterror = atof(gsa_split[i-1])*5;

	// Extracting SOG (knots)
	sog = atof(rmc_split[7]);

	// Extracting COG (degrees)
	cog = atof(rmc_split[8]);

	// Extracting magnetic variation (degrees)
	mag_var = atof(rmc_split[10]);

	if (strcmp(rmc_split[11], "W") == 0){
		mag_var *= -1;
	}

	retvals = mp_obj_new_list(10, (mp_obj_t[10]){mp_obj_new_float(*latitude), mp_obj_new_float(*longitude),
													mp_obj_new_float(pos_error), mp_obj_new_float(altitude),
													mp_obj_new_float(verterror), mp_obj_new_float(sog),
													mp_obj_new_float(cog), mp_obj_new_float(mag_var),
													mp_obj_new_float(geo_sep), mp_obj_new_str(timestamp, 8)});

	free(timestamp);
	free(latitude);
	free(longitude);
	free(gsa_split);

	return retvals;
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_getdata_obj, getdata);




/**
 * Code here exposes the module functions above to micropython as an object
*/

// Defining the functions that are exposed to micropython
static const mp_rom_map_elem_t neo_m8_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_update_buffer), MP_ROM_PTR(&neo_m8_update_buffer_obj)},
	{MP_ROM_QSTR(MP_QSTR_position), MP_ROM_PTR(&neo_m8_position_obj)},
	{MP_ROM_QSTR(MP_QSTR_velocity), MP_ROM_PTR(&neo_m8_velocity_obj)},
	{MP_ROM_QSTR(MP_QSTR_altitude), MP_ROM_PTR(&neo_m8_altitude_obj)},
	{MP_ROM_QSTR(MP_QSTR_getdata), MP_ROM_PTR(&neo_m8_getdata_obj)},
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
    { MP_ROM_QSTR(MP_QSTR_NEO_M8), MP_ROM_PTR(&neo_m8_type) },
};
static MP_DEFINE_CONST_DICT(neo_m8_globals_table, neo_m8_module_globals_table);

// Creating module object
const mp_obj_module_t neo_m8_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&neo_m8_globals_table,
};

MP_REGISTER_MODULE(MP_QSTR_neo_m8, neo_m8_module);