#include "neo_m8.h"

mp_obj_t neo_m8_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
	/**
	 * Checks all the given arguments, tests the micropython UART object, and handles initialization of the driver
	 * Also initializes the micropython object which is passed back
	*/
	uart_port_t uart_num;
	gpio_num_t uart_tx_pin, uart_rx_pin;
	uint8_t uart_id;
	esp_err_t err;

	// Checking arguments
	mp_arg_check_num(n_args, n_kw, 3, 3, false);

	// Getting arguments data
	uart_tx_pin = mp_obj_get_uint(args[0]);
	uart_rx_pin = mp_obj_get_int(args[1]);
	uart_id = mp_obj_get_uint(args[2]);

	// Ensuring UART ID and Pin number are valid - configured for ESP32-S3
	if ((uart_id != 1) && (uart_id != 2)){
		mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("UART ID can only be 1 or 2"));
	}
	if (!GPIO_IS_VALID_GPIO(uart_rx_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(uart_tx_pin)){
		mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid UART pin numbers"));
	}

	if (uart_id == 1){
		uart_num = UART_NUM_1;
	}
	else {
		uart_num = UART_NUM_2;
	}

	if (uart_is_driver_installed(uart_num)){
    	err = uart_driver_delete(uart_num);

		if (err != ESP_OK) {
    		mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART driver cleanup failed: %s"), esp_err_to_name(err));
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}

	// Configuring UART parameters
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 0,
		.source_clk = UART_SCLK_DEFAULT,
	};

	// Configuring UART
	err = uart_param_config(uart_num, &uart_config);
	if (err != ESP_OK) {
    	mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART driver config failed: %s"), esp_err_to_name(err));
	}

	err = uart_set_pin(uart_num, uart_tx_pin, uart_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
   		mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART driver pin config failed: %s"), esp_err_to_name(err));
	}

	// Creating the ESP-IDF UART - 512 byte RXbuf, 0 byte TXbuf (as I want writing UART info to be blocking, so that
	// the code doesn't go looking for ACKs/NACKs before a command has been sent)
	err = uart_driver_install(uart_num, 512, 0, 0, NULL, 0);
	if (err != ESP_OK){
   		mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART driver install failed: %s"), esp_err_to_name(err));
	}

	// Creating and allocating memory to the "self" instance of this module
	neo_m8_obj_t *self = m_new_obj(neo_m8_obj_t);

	// Initialising required data in the "self" object
	self->base.type = &neo_m8_type;
	self->uart_number = uart_num;
	self->buffer_length = 0;

	self->data.gll = NULL;
	self->data.gsa = NULL;
	self->data.gga = NULL;
	self->data.rmc = NULL;

	vTaskDelay(pdMS_TO_TICKS(100));

	return MP_OBJ_FROM_PTR(self);
}

static int16_t find_in_char_array(char *array, uint16_t length, char character_to_look_for, int16_t starting_point){
	/**
	 * Utility to find the index of a specific character in a string
	 * Basically, a C implementation of .find() in python
	 * Returns the index of the character, or -1 if it's not found
	*/
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
	/**
	 * Calculates and checks NMEA sentence checksums
	 * Returns 1 for a correct checksum, and 0 for incorrect checksums
	*/
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

static void update_buffer_internal(neo_m8_obj_t* self){
	/**
	 * Function to handle writing data into a 512-byte sliding window buffer
	*/
	int16_t length_read;
	size_t data_bytes_available;

	// Checking for potential buffer overflow
    uart_get_buffered_data_len(self->uart_number, &data_bytes_available);
	if (data_bytes_available > 500){
        uart_flush_input(self->uart_number);
	}

    if (self->buffer_length == 512){
        // Sliding the window a fixed amount 448 bytes
        memmove(self->buffer, self->buffer + 448, 64);
		self->buffer_length = 64;
	}

    length_read = uart_read_bytes(self->uart_number, self->buffer + self->buffer_length, 512 - self->buffer_length, 1);

	if (length_read < 0){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART reading error"));
	}

    self->buffer_length += length_read;
}

static void get_sentence(neo_m8_obj_t *self, nmea_sentence_data* output, char* desired_sentence){
	/**
     * Looks for a certain NMEA sentence, returns a pointer in the buffer to that sentence
	 * Times out after 1 second of running
	*/
    uint8_t sentence_length;
	int16_t start_pos = -1, end_pos = -1;
	uint64_t start_time = esp_timer_get_time();
	char nmea_sentence_type[4];

    // Function times out if it's running for more than 1 second
    while (esp_timer_get_time() - start_time < 1e6){
        update_buffer_internal(self);

		start_pos = find_in_char_array((char *) self->buffer, self->buffer_length, '$', 0);
		end_pos = find_in_char_array((char *) self->buffer, self->buffer_length, '\n', start_pos);

		if ((start_pos == -1) || (end_pos == -1)){
			continue;
		}

        sentence_length = end_pos - start_pos;

		// Checking the NMEA checksum
        if (nmea_checksum(self->buffer+start_pos, sentence_length) == 0){
			continue;
		}

        // Extracting sentence type
        strncpy(nmea_sentence_type, self->buffer+start_pos+3, 3);
		nmea_sentence_type[3] = '\0';

        // Checking if it's the sentence type we want
        if (strcmp(nmea_sentence_type, desired_sentence) == 0){
            output->sentence_start = self->buffer + start_pos;
            output->length = sentence_length;

            return;
		}
    }

    output->sentence_start = NULL;
    output->length = 0;

    return;
}

static void extract_timestamp(char* nmea_section, char* timestamp_out){
	/**
	 * Utility to take a segment of an NMEA sentence containing the timestamp and format it into a nice, human-readable form.
	*/

	timestamp_out[0] = nmea_section[0];
	timestamp_out[1] = nmea_section[1];
	timestamp_out[2] = ':';
	timestamp_out[3] = nmea_section[2];
	timestamp_out[4] = nmea_section[3];
	timestamp_out[5] = ':';
	timestamp_out[6] = nmea_section[4];
	timestamp_out[7] = nmea_section[5];
	timestamp_out[8] = '\0';
}

static void extract_lat_long(char* nmea_section, float* output){
	/**
	 * Utility to take the latitude/longitude section of an NMEA sentence and convert it into degrees and decimal minutes
	*/
	uint8_t i;
	int8_t pos_degrees_end, degrees;
	float minutes;

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

	// Combining and saving them
	*output = (degrees + minutes/60);
}

static int8_t ubx_ack_nack(neo_m8_obj_t *self){
	uint64_t start_time = esp_timer_get_time();
	uint16_t i;

	// This function times out after 1s of looking for an ACK/NACK
	while (esp_timer_get_time() - start_time < 1000000){
		vTaskDelay(pdMS_TO_TICKS(10));

		update_buffer_internal(self);

		// Making sure no buffer underflow is possible
		if (self->buffer_length < 4){
			continue;
		}

		// Searching for ACKs/NACKs
		for (i = 0; i < self->buffer_length-3; i++){
			if ((self->buffer[i] == 0xB5) && (self->buffer[i+1] == 0x62) && (self->buffer[i+2] == 0x05)){

				// NACK
				if (self->buffer[i+3] == 0x00){
					return 0;
				}
				// ACK
				else if (self->buffer[i+3] == 0x01){
					return 1;
				}
			}
		}
	}

	// Nothing found
	return -1;
}

static int8_t parse_gga(neo_m8_obj_t* self){
    /**
     * Parses the GGA NMEA sentence
    */
    nmea_sentence_data_t gga_sentence;
    char gga_copy[83], *gga_split[9];
    uint8_t i;

    // Collecting sentence position in buffer
    get_sentence(self, &gga_sentence, "GGA\0");

    // Checking for null pointer
    if (gga_sentence.sentence_start == NULL){
        return -1;
    }

    // Creating a copy of the GGA sentence as strtok is destructive
    strncpy(gga_copy, gga_sentence.sentence_start, gga_sentence.length);
    gga_copy[gga_sentence.length] = '\0';

    // Splitting the GGA sentence up into sections, which can then be processed
	char* token = strtok(gga_copy, ",");
	for (i = 0; token != NULL; i++){
		gga_split[i] = token;

		token = strtok(NULL, ",");
	}

    // If not enough fields OR status flag indicates bad fix, then return zero
	if ((i < 8) || (strcmp(gga_split[6], "1") != 0)){
        return 0;
	}

    // Extracting latitude in degrees decimal minutes
    extract_lat_long(gga_split[2], &(self->data.latitude));

    if (strcmp(gga_split[3], "S") == 0){
        self->data.latitude *= -1;
	}

    // Extracting longitude in degrees decimal minutes
    extract_lat_long(gga_split[4], &(self->data.longitude)));

	if (strcmp(gga_split[5], "W") == 0){
        self->data.longitude *= -1;
	}

    // Extracting HDOP value, converting it to horizontal position error
    self->data.position_error = atof(gga_split[8])*2.5;

    // Extracting altitude
    self->data.altitude = atof(gga_split[9]);

	// Extracting geoid separation
    self->data.geosep = atof(gga_split[11]);

    // Extracting GMT timestamp in hh:mm:ss format
    extract_timestamp(gga_split[1], self->data.timestamp);

    // Removing this NMEA sentence from the buffer
    memmove(gga_sentence.sentence_start, gga_sentence.sentence_start + gga_sentence.length, gga_sentence.length);

    return 1;
}

static int8_t parse_rmc(neo_m8_obj_t* self){
    /**
     * Parses the RMC NMEA sentence
    */
    nmea_sentence_data_t rmc_sentence;
    char rmc_copy[83], *rmc_split[13];
    float cog
    uint8_t i;

    // Collecting RMC sentence position in buffer
    get_sentence(self, &rmc_sentence, "RMC\0");

	// Checking for null pointers
    if (rmc_sentence.sentence_start == NULL){
        return -1;
	}

	// Creating a copy of the RMC sentence as strtok is destructive
	// Uses fixed length of 83 bytes, the maximum sentence length in NMEA 0183 Version 4.10
    strncpy(rmc_copy, rmc_sentence.sentence_start, rmc_sentence.length);
    rmc_copy[rmc_sentence.length] = '\0';

    // Splitting the RMC sentence up into sections, which can then be processed
	char* token = strtok(rmc_copy, ",");
	for (i = 0; token != NULL; i++){
		rmc_split[i] = token;

		token = strtok(NULL, ",");
	}

    // If not enough fields OR status flag indicates bad fix, then return zero
	if ((i < 8) || (strcmp(rmc_split[2], "A") != 0)){
        return 0;
	}

    // Extracting timestamp
    extract_timestamp(rmc_split[1], self->data.timestamp);

	// Extracting SOG (knots)
    self->data.sog = atof(rmc_split[7]);

	// Extracting COG (degrees)
    cog = atof(rmc_split[8]);

	// If the COG is > 360 degrees, it means it's picked out the "date" field instead
    // Which happens if the SOG isn't high enough for an accurate COG to be calculate. So -1 is saved instead
    if (cog > 360.0f){
        self->data.cog = -1;
	}
    else {
        self->data.cog = cog;
    }

    // Extracting date
    self->data.date = atoi(rmc_split[i-2]);

    // Removing this NMEA sentence from the buffer
    memmove(rmc_sentence.sentence_start, rmc_sentence.sentence_start + rmc_sentence.length, rmc_sentence.length);

    return 1;
}

static int8_t parse_gsa(neo_m8_obj_t* self){
    /**
     * Parses the GSA NMEA sentence
    */
    nmea_sentence_data_t gsa_sentence;
    char gsa_copy[83], **gsa_split = NULL;
    uint8_t i;

    // Getting pointer to the start of the GSA sentence in the buffer
    get_sentence(self, &gsa_sentence, "GSA\0");

    // Checking for null pointer
    if (gsa_sentence.sentence_start == NULL){
        return -1;
	}

    // Copying sentence as strtok is destructive
    strncpy(gsa_copy, gsa_sentence.sentence_start, gsa_sentence.length);
    gsa_copy[gsa_sentence.length] = '\0';

    // Splitting up the GSA sentence
    char* token = strtok(gsa_copy, ",");
	for (i = 0; token != NULL; i++){
		// Re-allocating extended memory - the length of the GSA sentence is unknown
		gsa_split = (char **) realloc(gsa_split, (i+1)*CHAR_PTR_SIZE);

		if (gsa_split == NULL){
			mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Could not allocate memory"));
		}

		gsa_split[i] = token;

		token = strtok(NULL, ",");
	}

    // Extracting vertical error
    self->data.vertical_error = atof(gsa_split[i-1])*5;

	free(gsa_split);

    // Removing this NMEA sentence from the buffer
    memmove(gsa_sentence.sentence_start, gsa_sentence.sentence_start + gsa_sentence.length, gsa_sentence.length);

    return 1;
}

mp_obj_t update_buffer(mp_obj_t self_in){
	/**
	 * Exposing the update_buffer_internal function to micropython
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

	update_buffer_internal(self);

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_update_buffer_obj, update_buffer);

mp_obj_t position(mp_obj_t self_in){
	/**
	 * Micropython-exposed function
	 * Returns location data: latitude, longitude, position error, timestamp
	 *                    |degrees/decimal minutes|    meters    | GMT hh:mm:ss
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int8_t err;

    err = parse_gga(self);

    // Checking for errors
    if (err != 1){
		return mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

    return mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(self->data.latitude),
                                            mp_obj_new_float(self->data.longitude),
                                            mp_obj_new_float(self->data.position_error)
                                            mp_obj_new_str(self->data.timestamp, 8)});
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_position_obj, position);

mp_obj_t velocity(mp_obj_t self_in){
	/**
	 * Micropython-exposed function
	 * Returns velocity and course data - speed over ground (knots), course over ground (degrees), timestamp (GMT hh:mm:ss)
	 * Course over ground is returned as Python Nonetype if not availible due to speed being too low
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int8_t err;

    err = parse_rmc(self);

    // Checking for errors
    if (err != 1){
		return mp_obj_new_list(3, (mp_obj_t[3]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

    if (self->data.cog == -1){
        return mp_obj_new_list(3, (mp_obj_t[3]){mp_obj_new_float(self->data.sog),
                                                mp_obj_new_float(mp_const_none),
                                                mp_obj_new_str(self->data.timestamp, 8)});
    }

    return mp_obj_new_list(3, (mp_obj_t[3]){mp_obj_new_float(self->data.sog),
                                            mp_obj_new_float(self->data.cog),
                                            mp_obj_new_str(self->data.timestamp, 8)});
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_velocity_obj, velocity);

mp_obj_t altitude(mp_obj_t self_in){
	/**
	 * Micropython-exposed function
	 * Returns altitude data - altitude AMSL (meters), geoid separation (meters), vertical error (meters), timestamp (GMT hh:mm:ss)
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int8_t err1, err2;

    err1 = parse_gga(self);
    err2 = parse_gsa(self);

    // Checking for errors
    if ((err1 != 1) || (err2 != 1)){
		return mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_float(0.0f), mp_obj_new_str("0", 1)});
	}

    return mp_obj_new_list(4, (mp_obj_t[4]){mp_obj_new_float(self->data.altitude),
                                            mp_obj_new_float(self->data.geosep),
                                            mp_obj_new_float(self->data.vertical_error),
                                            mp_obj_new_str(self->data.timestamp, 8)});
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_altitude_obj, altitude);

mp_obj_t getdata(mp_obj_t self_in){
	/**
	 * Micropython-exposed function
	 * Returns all availible GPS data - latitude, longitude, position error, altitude, vertical error, speed over ground, course over ground, geoid separation, timestamp
	 * Latitude/longitude: degrees and decimal minutes
	 * Position error/altitude/vertical error/geoid separation: meters
	 * Speed over ground: Knots
	 * Couse over ground: degrees (or Python Nonetype if speed too low to calculate course)
	 * Timestamp: GMT hh:mm:ss
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t err1, err2, err3;

    err1 = parse_gga(self);
    err2 = parse_rmc(self);
    err3 = parse_gsa(self);

    // Checking for errors
    if ((err1 != 1) || (err2 != 2) || (err3 != 1)){
		return mp_obj_new_list(9, (mp_obj_t[9]){mp_obj_new_float(0.0f),
                                                mp_obj_new_float(0.0f),
												mp_obj_new_float(0.0f),
                                                mp_obj_new_float(0.0f),
												mp_obj_new_float(0.0f),
                                                mp_obj_new_float(0.0f),
												mp_const_none,
                                                mp_obj_new_float(0.0f),
												mp_obj_new_str("0", 1)});
    }

    // If the COG is invalid, return none instead
    if (self->data.cog == -1){
        return mp_obj_new_list(9, (mp_obj_t[9]){mp_obj_new_float(self->data.latitude),
                                                mp_obj_new_float(self->data.longitude),
                                                mp_obj_new_float(self->data.position_error),
                                                mp_obj_new_float(self->data.altitude),
                                                mp_obj_new_float(self->data.vertical_error),
                                                mp_obj_new_float(self->data.sog),
                                                mp_const_none,
                                                mp_obj_new_float(self->data.geosep),
                                                mp_obj_new_str(self->data.timestamp, 8)});
	}

    return mp_obj_new_list(9, (mp_obj_t[9]){mp_obj_new_float(self->data.latitude),
                                            mp_obj_new_float(self->data.longitude),
                                            mp_obj_new_float(self->data.position_error),
                                            mp_obj_new_float(self->data.altitude),
                                            mp_obj_new_float(self->data.vertical_error),
                                            mp_obj_new_float(self->data.sog),
                                            mp_obj_new_float(self->data.cog),
                                            mp_obj_new_float(self->data.geosep),
                                            mp_obj_new_str(self->data.timestamp, 8)});
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_getdata_obj, getdata);

mp_obj_t timestamp(mp_obj_t self_in){
	/**
	 * Function to return GPS time/date stamp
	 * Formatted as "{YYYY-MM-DD}T{hh:mm:ss}Z"
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int8_t err;
	char timestamp[20] = "2000-01-01T00:00:00Z";

    err = parse_rmc(self)

    // Checking for errors
    if (err != 1){
        return mp_obj_new_str(timestamp, 20);
    }

	// Formatting the date data
    timestamp[8] = self->data.date[0];
	timestamp[9] = self->data.date[1];
	timestamp[5] = self->data.date[2];
	timestamp[6] = self->data.date[3];
	timestamp[2] = self->data.date[4];
	timestamp[3] = self->data.date[5];

	// Formatting the time data
    timestamp[11] = self->data.timestamp[0];
    timestamp[12] = self->data.timestamp[1];
    timestamp[14] = self->data.timestamp[3];
    timestamp[15] = self->data.timestamp[4];
    timestamp[17] = self->data.timestamp[6];
    timestamp[18] = self->data.timestamp[7];

	return mp_obj_new_str(timestamp, 20);
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_timestamp_obj, timestamp);

mp_obj_t gnss_stop(mp_obj_t self_in){
	/**
	 * Function to softly shut down the NEO-M8's GNSS systems
	 * Can be used for power saving as well as just turning it off
	 * Returns 1 if an ACK was received, 0 if a NACK was received, and -1 if nothing received
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);
	int8_t flag, bytes_written;

	// Defining the UBX-CFG-RST packet to send
	// Doesn't need to be in error catching, but needs write_method to be defined
	uint8_t packet[12] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74};

	// Sending the UBX packet
	bytes_written = uart_write_bytes(self->uart_number, packet, 12);
	flag = ubx_ack_nack(self);

	if (bytes_written != 12){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	return mp_obj_new_int(flag);
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_gnss_stop_obj, gnss_stop);

mp_obj_t gnss_start(mp_obj_t self_in){
	/**
	 * Function to start up the NEO-M8's GNSS systems
	 * To be used to start the module up again after calling gnss_stop()
	 * Returns 1 if an ACK was received, 0 if a NACK was received, and -1 if nothing received
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);
	int8_t flag, bytes_written;

	// Defining the UBX-CFG-RST packet to send
	// Doesn't need to be in error catching, but needs write_method to be defined
	uint8_t packet[12] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76};

	// Sending the UBX packet
	bytes_written = uart_write_bytes(self->uart_number, packet, 12);
	flag = ubx_ack_nack(self);

	if (bytes_written != 12){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	return mp_obj_new_int(flag);
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_gnss_start_obj, gnss_start);

mp_obj_t setrate(mp_obj_t self_in, mp_obj_t rate, mp_obj_t measurements_per_nav_sol){
	/**
	 * Function to change rate of new navigation solutions output for the NEO-M8
	 * Returns 1 if an ACK was received, 0 if a NACK was received, and -1 if nothing received
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);
	int8_t flag;
	uint8_t bytes_written, i, ck_a = 0, ck_b = 0;

	float mp_rate = mp_obj_get_float(rate);
	if ((mp_rate < 0) || (mp_rate > 10)){
		mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid GPS data output rate. Rate must be between 0 and 10 Hz."));
	}

	uint8_t ms_rate = (uint8_t)(1000/mp_rate), measurements_nav_sol = mp_obj_get_uint(measurements_per_nav_sol);

	uint8_t bytes_packet[8] = {0x06, 0x08, 0x06, 0x00, ms_rate, measurements_nav_sol, 0x00, 0x00};

	// Calculating the UBX checksum
	for (i = 0; i < 8; i++){
		ck_a += bytes_packet[i];
		ck_b += ck_a;
	}
	ck_a &= 0xFF;
	ck_b &= 0xFF;

	// Putting together the final data packet
	uint8_t packet[12] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, ms_rate, measurements_nav_sol, 0x00, 0x00, ck_a, ck_b};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet, 12);
	flag = ubx_ack_nack(self);

	if (bytes_written != 12){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	return mp_obj_new_int(flag);
}
static MP_DEFINE_CONST_FUN_OBJ_3(neo_m8_setrate_obj, setrate);

mp_obj_t modulesetup(mp_obj_t self_in){
	/**
	 * Configures the module to required settings
	 * Returns 1 if an ACK was received, 0 if a NACK was received, and -1 if nothing received
	*/
	neo_m8_obj_t *self = MP_OBJ_TO_PTR(self_in);
	int8_t flag, bytes_written;

	// UBX-CFG-MSG: Disabling VTG NMEA sentence as it is redundant
	// Putting together the data packet
	uint8_t packet[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet, 11);

	if (bytes_written != 11){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	// Checking for ACK/NACK, returning if no ACK found
	flag = ubx_ack_nack(self);

	if (flag != 1){
		return mp_obj_new_int(flag);
	}

	// UBX-CFG-NAV5: Configures module to airborne with <4g acceleration, 3D fix only,
	// satellites 15 degrees above horizon to be used for a fix, static hold at <20cm/s and 1m, automatic UTC standard
	// Putting together data packet
	uint8_t packet2[44] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x47, 0x08, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x2B};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet2, 44);

	if (bytes_written != 44){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	// Checking for ACK/NACK, returning if no ACK found
	flag = ubx_ack_nack(self);

	if (flag != 1){
		return mp_obj_new_int(flag);
	}

	// UBX-CFG-NAVX5: Configures module to min. satellites for navigation=4, max. satellites for navigation=50,
	// initial fix must be 3D, AssistNow Autonomous turned on, maximum AssistNow Autonomous orbit error=20m
	// Putting together data packet
	uint8_t packet3[48] = {0xB5, 0x62, 0x06, 0x23, 0x28, 0x00, 0x00, 0x00, 0x44, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x3C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2B, 0x19};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet3, 48);

	if (bytes_written != 48){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}
	// Checking for ACK/NACK, returning if no ACK found
	flag = ubx_ack_nack(self);

	if (flag != 1){
		return mp_obj_new_int(flag);
	}

	// UBX-CFG-GNSS: Configures module to enable Galileo, GPS, GLONASS, BeiDou, SBAS
	// Putting together data packet
	uint8_t packet4[52] = {0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 0x00, 0x00, 0xFF, 0x05, 0x00, 0x08, 0x10, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x01, 0x00, 0x01, 0x02, 0x02, 0x08, 0x00, 0x00, 0x01, 0x00, 0x01, 0x03, 0x08, 0x0E, 0x00, 0x00, 0x01, 0x00, 0x01, 0x06, 0x06, 0x0e, 0x00, 0x00, 0x01, 0x00, 0x01, 0xDA, 0x1A};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet4, 52);

	if (bytes_written != 52){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	// Checking for ACK/NACK, returning if no ACK found
	flag = ubx_ack_nack(self);

	if (flag != 1){
		return mp_obj_new_int(flag);
	}

	// UBX-CFG-ITFM: Configures module to enable interference detection, broadband threshold=7dB, continuous wave threshold=20dB, active antenna
	// Putting together data packet
	uint8_t packet5[16] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xAD, 0x62, 0xAD, 0x47, 0x00, 0x00, 0x23, 0x1E, 0x8B, 0xF6};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet5, 16);

	if (bytes_written != 16){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}
	// Checking for ACK/NACK, returning if no ACK found
	flag = ubx_ack_nack(self);

	if (flag != 1){
		return mp_obj_new_int(flag);
	}

	// UBX-CFG-CFG: Configures module to save all the above configured settings into the module's programmable flash
	// This should be changed to saving into battery-backed RAM for NEO-M8Q and NEO-M8M which don't have programmable flash
    // Do this by changing the byte b'\x02' below for the byte b'\x01' (assuming you have BBR, unless you want to save it into the SPI Flash)
	// Putting together data packet
	uint8_t packet6[21] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38, 0x57};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet6, 21);

	if (bytes_written != 21){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	// Checking for ACK/NACK, returning if no ACK found
	flag = ubx_ack_nack(self);

	if (flag != 1){
		return mp_obj_new_int(flag);
	}

	// UBX-CFG-RST: Completely hardware resets the module
	// Putting together data packet
	uint8_t packet7[12] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x0C, 0x5D};

	// Writing the packet to the UART
	bytes_written = uart_write_bytes(self->uart_number, packet7, 12);

	if (bytes_written != 12){
		mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("UART write failed"));
	}

	// Checking for ACK/NACK, returning if no ACK found
	flag = ubx_ack_nack(self);

	if (flag != 1){
		return mp_obj_new_int(flag);
	}

	return mp_obj_new_int(1);
}
static MP_DEFINE_CONST_FUN_OBJ_1(neo_m8_modulesetup_obj, modulesetup);



/**
 * Code here exposes the module functions above to micropython as an object
*/

// Defining the functions that are exposed to micropython
static const mp_rom_map_elem_t neo_m8_locals_dict_table[] = {
	{MP_ROM_QSTR(MP_QSTR_position), MP_ROM_PTR(&neo_m8_position_obj)},
	{MP_ROM_QSTR(MP_QSTR_velocity), MP_ROM_PTR(&neo_m8_velocity_obj)},
	{MP_ROM_QSTR(MP_QSTR_altitude), MP_ROM_PTR(&neo_m8_altitude_obj)},
	{MP_ROM_QSTR(MP_QSTR_timestamp), MP_ROM_PTR(&neo_m8_timestamp_obj)},
	{MP_ROM_QSTR(MP_QSTR_getdata), MP_ROM_PTR(&neo_m8_getdata_obj)},
	{MP_ROM_QSTR(MP_QSTR_update_buffer), MP_ROM_PTR(&neo_m8_update_buffer_obj)},
	{MP_ROM_QSTR(MP_QSTR_gnss_start), MP_ROM_PTR(&neo_m8_gnss_start_obj)},
	{MP_ROM_QSTR(MP_QSTR_gnss_stop), MP_ROM_PTR(&neo_m8_gnss_stop_obj)},
	{MP_ROM_QSTR(MP_QSTR_setrate), MP_ROM_PTR(&neo_m8_setrate_obj)},
	{MP_ROM_QSTR(MP_QSTR_modulesetup), MP_ROM_PTR(&neo_m8_modulesetup_obj)},
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
