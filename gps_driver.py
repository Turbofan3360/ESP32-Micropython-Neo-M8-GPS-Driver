from machine import UART
import time, struct

class GPSReceive:
    def __init__(self, rx_pin, tx_pin):
        """
        Driver for the NEO-M8 series of GPS modules.
        
        Required parameters upon initialization: The ESP32 pin that the module's TX pin is connected to, followed by the ESP32 pin that the module's RX pin is connected to.
        
        Potential methods:
            position() - Gets position information
            velocity() - Gets velocity information
            altitude() - Gets altitude information
            getdata() - Aggregator of the previous methods
            gnss_start() - Starts the module's GNSS system
            gnss_stop() - Softly stops the module's GNSS system
            update_buffer() - reads from the  ESP32 UART RX buffer into the driver's buffer (fixed to the most recent 512 bytes). This enables a speed-up of the reading process if called periodically.
            
            setrate(rate, measurements per nav solution) - Enables you to set the rate of data output from the module
            modulesetup() - Sets up the module to the settings required for my usage. Can be adapted to suit other uses. For the NEO-M8N and NEO-M8Q, this will need to be adjusted slightly. See the method for details.
        
        modulesetup() and setrate() return True, False or None depending on whether it received an ACK, NACK or nothing from the module.
        """

        self.gps = UART(2, baudrate=9600, tx=tx_pin, rx=rx_pin)
        
        self.data = {}
        self.background_buffer = bytearray()
    
    def _checksum(self, nmea_sentence):
        checksum = 0
        checksum_pos = nmea_sentence.find(b'*')
        nmea_sentence_stripped = nmea_sentence[1:checksum_pos]
            
        for i in nmea_sentence_stripped:
            checksum ^= i
        checksum = ("%02X"%checksum).encode('utf-8')

        if nmea_sentence[checksum_pos+1:checksum_pos+3] == checksum:
            return True
        return False
    
    def _ubx_checksum(self, ubx_packet):
        ck_a = ck_b = 0
        
        for byte in ubx_packet:
            ck_a += byte
            ck_b += ck_a
            
        ck_a &= 0xFF
        ck_b &= 0xFF
        
        return struct.pack("<B", ck_a), struct.pack("<B", ck_b)
    
    def update_buffer(self):
        """
        Method to update the 512-byte sliding window buffer of GPS data that is maintained by the driver.
        
        Calling this regularly takes little time and speeds up the executing of the main processing methods (e.g. position()) later on.
        """
        
        if self.gps.any():
            self.background_buffer += self.gps.read()
            
            length = len(self.background_buffer)
            if length > 512:
                self.background_buffer = self.background_buffer[length-512:]
    
    def _update_data(self):
        end_pos = -1
        start_pos = -1
        sentences_read = 0
        
        while sentences_read < 5:
            self.update_buffer()
            
            while sentences_read < 5:
                start_pos = self.background_buffer.find(b'$')
                end_pos = self.background_buffer.find(b'\n', start_pos)
                
                if start_pos == -1 or end_pos == -1:
                    break
            
                data_section = self.background_buffer[start_pos:end_pos+1]
                self.background_buffer = self.background_buffer[end_pos+1:]
                
                if self._checksum(data_section):
                    try:
                        data_section = data_section.decode('utf-8')
                    except UnicodeError:
                        break
                
                    sentence_header = data_section[3:6]
                    self.data[sentence_header] = data_section
                    sentences_read += 1
    
    def position(self):
        """
        Returns a list of [latitude, longitude, horizontal error, timestamp] all formatted as strings.
        
        Latitude/longitude are as degrees. Latitude - N considered positive, S negative. Longitude - E considered positive, W negative.
        Position error is in meters - fix error in 2D. Returned as float
        Timestamp (UTC) is formatted as hh:mm:ss - returned as string
        """
        
        try:
            self._update_data()
        except UnicodeError:
            self._update_data()
        
        try:
            gll_sentence = self.data["GLL"].split(",")
        except KeyError:
            return 0, 0, 0, 0
        
        time_utc = gll_sentence[5]
        if time_utc:
            time_stamp = time_utc[:2] + ":" + time_utc[2:4] + ":" + time_utc[4:]
        else:
            time_stamp = 0
        
        try:
            gsa_sentence = self.data["GSA"].split(",")
        except:
            return 0, 0, 0, time_stamp

        # checking status flag before extracting lat/long/timestamp
        if gll_sentence[6] == "A":
            pos_minutes = gll_sentence[1].find(".")-2
            minutes = float(gll_sentence[1][pos_minutes:])
            degrees = int(gll_sentence[1][:pos_minutes])
            
            if gll_sentence[2] == "E":
                lat = degrees+minutes/60
            else:
                lat = -1*(degrees+minutes/60)
            
            pos_minutes = gll_sentence[3].find(".")-2
            minutes = float(gll_sentence[3][pos_minutes:])
            degrees = int(gll_sentence[3][:pos_minutes])
            
            if gll_sentence[4] == "N":
                long = degrees+minutes/60
            else:
                long = -1*(degrees+minutes/60)
            
            # This is the 2D horizontal position error
            hdop = float(gsa_sentence[-3])
            position_error = hdop * 2.5 # 68% confidence level, 1 sigma - estimated accuracy of GPS module is ~2.5m from datasheet
            
            return lat, long, position_error, time_stamp
        return 0, 0, 0, time_stamp
        
    def velocity(self, data_needs_updating=True):
        """
        Returns a list of [speed over ground, course over ground, magnetic variation, timestamp]
        
        SOG is in Knots (Kn) - returned as string with "Kn"
        COG is in degrees - returned as string with "°"
        Timestamp (UTC) formatted as hh:mm:ss - returned as string
        """
        
        if data_needs_updating:
            try:
                self._update_data()
            except UnicodeError:
                self._update_data()
            
        try:
            rmc_sentence = self.data["RMC"].split(",")
        except KeyError:
            return 0, 0, 0, 0
        
        time_utc = rmc_sentence[1]
        if time_utc:
            time_stamp = time_utc[:2] + ":" + time_utc[2:4] + ":" + time_utc[4:]
        else:
            time_stamp = 0
        
        if rmc_sentence[2] == "A":
            sog = rmc_sentence[7] + "Kn"
            
            if rmc_sentence[8]:
                cog = rmc_sentence[8] + "°"
            else:
                cog = "N/A"
            
            return sog, cog, time_stamp
        return 0, 0, time_stamp

    def altitude(self, data_needs_updating=True):
        """
        Returns a list of [altitude, geoid separation, vertical error, timestamp]
        
        Altitude is in meters - returned as float
        Geoid separation is in meters - returned as float
        Vertical error is in meters - returned as float
        Timestamp (UTC) is formatted as hh:mm:ss - returned as string
        """
        
        if data_needs_updating:
            try:
                self._update_data()
            except UnicodeError:
                self._update_data()
            
        try:
            gga_sentence = self.data["GGA"].split(",")
        except KeyError:
            return 0, 0, 0, 0
        
        time_utc = gga_sentence[1]
        if time_utc:
            time_stamp = time_utc[:2] + ":" + time_utc[2:4] + ":" + time_utc[4:]
        else:
            time_stamp = 0 
        
        try:
            gsa_sentence = self.data["GSA"].split(",")
        except KeyError:
            return 0, 0, 0, time_stamp
        
        if gga_sentence[6] != "0":
            alt = gga_sentence[9]
            geo_sep = gga_sentence[11]
            
            vdop = float(gsa_sentence[-2])
            
            vertical_error = vdop * 5 # 68% confidence level, 1 sigma - estimated HORIZONTAL accuracy of GPS module is ~2.5m from datasheet, so vertical accuracy ~4.5-5m
            
            return float(alt), float(geo_sep), vertical_error, time_stamp
        
        return 0, 0, 0, time_stamp
    
    def getdata(self):
        """
        Returns list of [latitude, longitude, altitude, position error, sog, cog, magnetic variation, geoid separation, timestamp]
        
        Latitude/longitude are as degrees. Latitude - N considered positive, S negative. Longitude - E considered positive, W negative.
        Altitude is in meters - float
        Position error is in meters - float
        Vertical error is in meters - float
        SOG is in Knots (Kn) - string with "Kn"
        COG is in degrees - string with "°"
        Geoid separation is in meters - float
        Timestamp (UTC) is formatted as hh:mm:ss - string
        """
        
        lat, long, position_error, timestamp_0 = self.position()
        sog, cog, timestamp_1 = self.velocity(data_needs_updating=False)
        alt, geo_sep, vertical_error, timestamp_2 = self.altitude(data_needs_updating=False)
        
        # Trying to get timestamp from at least 1 method
        if timestamp_0:
            timestamp = timestamp_0
        elif timestamp_1:
            timestamp = timestamp_1
        elif timestamp_2:
            timestamp = timestamp_2
        else:
            timestamp = 0
                
        return lat, long, position_error, alt, vertical_error, sog, cog, geo_sep, timestamp
    
    def _ubx_ack_nack(self):
        start = time.time_ns()
        self.ackdata = b''
        
        while time.time_ns() < start+1000000000:
            if self.gps.any():
                self.ackdata += self.gps.read()
                
                index_ack = self.ackdata.find(b'\xb5\x62\x05\x01')
                index_nack = self.ackdata.find(b'\xb5\x62\x05\x00')
                
                if index_ack != -1:
                    return True
                if index_nack != -1:
                    return False
        return None
    
    def gnss_stop(self):
        """
        Softly shuts down the module's GNSS systems.
        Can be called to reduce power usage on the module.
        
        Should be called before pulling the power.
        """
        # UBX-CFG-RST message
        packet = b'\x06\x04' + b'\x04\x00' + b'\x00\x00' + b'\x08' + b'\x00'
            
        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)
        
        return
    
    def gnss_start(self):
        """
        Softly starts up the module's GNSS systems.
        """
        # UBX-CFG-RST message
        packet = b'\x06\x04' + b'\x04\x00' + b'\x00\x00' + b'\x09' + b'\x00'
            
        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)
        
        return
        
    def setrate(self, rate, measurements_per_nav_solution):
        """
        Enables you to set the data output rate from the module.
        
        Takes two arguments - the data output rate (Hz), followed by the number of measurements taken per data output.
        
        Returns True, False or None depending on whether an ACK, NACK, or nothing was received from the module.
        """
        
        measurement_time_delta_ms = int(1000/rate)
        # Packing up the key settings that need changing
        measurement_time_delta_ms = struct.pack("<H", measurement_time_delta_ms)
        measurements_per_nav_solution = struct.pack("<H", measurements_per_nav_solution)
        
        packet = b'\x06\x08\x06\x00' + measurement_time_delta_ms + measurements_per_nav_solution + b'\x00\x00'
        # Getting checksums
        ck_a, ck_b = self._ubx_checksum(packet)
        
        packet += ck_a+ck_b
        # Adding header
        packet = b'\xb5\x62' + packet
        
        self.gps.write(packet)
        
        # Checking for the ACK or NACK
        return self._ubx_ack_nack()
        
    def modulesetup(self):
        """
        Sets up the module to the settings required for my usage.
        
        Only needs to be called once per module, as settings are then saved into battery-backed RAM or the programmable flash
        Currently, the function works for the NEO-M8N and M8J, will need to be adjusted for the M8M and M8Q to save parameters into the battery-backed RAM instead of programmable flash.
        
        Settings configured:
            - VTG NMEA sentence disabled (contains redundant data)
            - Module set to airborne with <4g acceleration
            - 3D fixes only
                - Initial fix must be 3D
            - Satellites need to be 15 degrees above horizon to be used for a fix
            - Minimum satellites for navigation fix is 4
            - Maximum satellites for navigation fix is 50 (more than the module will realistically be able to see at any time)
            - Static hold at <20cm/s velocity and within 1m
            - AssistNow Autonomous enabled
                - Maximum AssistNow Autonomous orbit error is 20m
            - GNSS constellations enabled - Galileo, GPS, GLONASS, BeiDou, SBAS
            - Enabled interference detection
                - Broadband detection threshold is 7dB
                - Continuous wave detection threshold is 20dB
                - Active antenna
        """
        count = 0
        # Uses UBX-CFG-MSG sentence to disable the VTG NMEA sentence
        self.gps.write(b'\xb5\x62\x06\x01\x03\x00\xF0\x05\x00\xff\x19')
        flag = self._ubx_ack_nack()
        while not flag and count < 5:
            time.sleep(0.5)
            count += 1
            
            self.gps.write(b'\xb5\x62\x06\x01\x03\x00\xF0\x05\x00\xff\x19')
            flag = self._ubx_ack_nack()
        if count == 5 and not flag:
            return
        
        count = 0
        # Using UBX-CFG-NAV5 to set module to: airborne with <4g acceleration, 3D fix only
        # satellites 15 degrees above horizon to be used for a fix, static hold at <20cm/s and 1m,
        # automatic UTC standard
        packet = b'\x06\x24' + b'$\x00' + b'G\x08' + b'\x08' + b'\x02' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x00' + b'\x14' + b'\x00' + b'\x00\x00' + b'\x00\x00' + b'\x00\x00' + b'\x00\x00' + b'\x14' + b'\x00' + b'\x00' + b'\x00' + b'\x00' + b'\x01' + b'\x00' + b'\x00\x00\x00\x00\x00\x00\x00'

        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)
        flag = self._ubx_ack_nack()
        
        while not flag and count < 5:
            time.sleep(0.5)
            count += 1
            
            self.gps.write(packet)
            flag = self._ubx_ack_nack()
        if count == 5 and not flag:
            return
        
        count = 0
        # Using UBX-CFG-NAVX5 to set module to: min. satellites for navigation=4,
        # max. satellites for navigation=50, initial fix must be 3D, AssistNow Autonomous turned on,
        # maximum AssistNow Autonomous orbit error=20m
        packet = b'\x06\x23' + b'(\x00' + b'\x00\x00' + b'D@' + b'\x00\x00\x00\x00' + b'\x00\x00' + b'\x04' + b'<' + b'\x00' + b'\x00' + b'\x01' + b'\x00' + b'\x00' + b'\x00\x00' + b'\x00\x00\x00\x00\x00\x00' + b'\x00' + b'\x01' + b'\x00\x00' + b'\x14\x00' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x00' + b'\x00'
        
        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)
        flag = self._ubx_ack_nack()
        
        while not flag and count < 5:
            time.sleep(0.5)
            count += 1
            
            self.gps.write(packet)
            flag = self._ubx_ack_nack()
        if count == 5 and not flag:
            return
        
        count = 0
        # Constructing UBX-CFG-GNSS message to enable Galileo, GPS, GLONASS, BeiDou, SBAS
        packet = b'\x06\x3e' + b'\x2c\x00' + b'\x00\x00\xff\x05' + b'\x00\x08\x10\x00\x00\x01\x00\x01' + b'\x01\x01\x03\x00\x00\x01\x00\x01' + b'\x02\x02\x08\x00\x00\x01\x00\x01' + b'\x03\x08\x0e\x00\x00\x01\x00\x01' + b'\x06\x06\x0e\x00\x00\x01\x00\x01'
                               # Lenth       #payload              # GPS                                 # SBAS                                # Galileo                             # BeiDou                              # GLONASS
        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)
        flag = self._ubx_ack_nack()
        time.sleep(0.5)
        
        while not flag and count < 5:
            count += 1
            
            self.gps.write(packet)
            flag = self._ubx_ack_nack()
            time.sleep(0.5)
        if count == 5 and not flag:
            return
        
        self.gnss_start()
        
        count = 0
        # Constructing UBX-CFG-ITFM message to configure interference/jamming monitoring on the module
        # Enabling interference detection, broadband threshold=7dB, continuous wave threshold=20dB, active antenna
        packet = b'\x06\x39' + b'\x08\x00' + b'\xadb\xadG' + b'\x00\x00#\x1e'
            
        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)
        flag = self._ubx_ack_nack()
        
        while not flag and count < 5:
            time.sleep(0.5)
            count += 1
            
            self.gps.write(packet)
            flag = self._ubx_ack_nack()
        if count == 5 and not flag:
            return
        
        count = 0
        # Constructing UBX-CFG-CFG message: saving all the above configured settings into the module's programmable flash
        # This should be changed to saving into battery-backed RAM for NEO-M8Q and NEO-M8M which don't have programmable flash
        # Do this by changing the byte b'\x02' below for the byte b'\x01' (assuming you have BBR, unless you want to save it into the SPI Flash)
        packet = b'\x06\x09' + b'\x0d\x00' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x1a' + b'\x00\x00\x00\x00' + b'\x02'
            
        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)
        flag = self._ubx_ack_nack()
        
        while not flag and count < 5:
            time.sleep(0.5)
            count += 1
            
            self.gps.write(packet)
            flag = self._ubx_ack_nack()
        if count == 5 and not flag:
            return
        
        count = 0
        
        # UBX-CFG-RST message to do a complete hardware reset to the module
        packet = b'\x06\x04' + b'\x04\x00' + b'\xff\xff' + b'\x00' + b'\x00'
            
        ck_a, ck_b = self._ubx_checksum(packet)
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        self.gps.write(packet)

        return True


if __name__ == "__main__":
    gps = GPSReceive(20, 21)
    
    flag=gps.modulesetup()
    while not flag:
        flag=gps.modulesetup()
    
    flag = gps.setrate(2, 3)
    while not flag:
        flag = gps.setrate(2, 3)
    
    while True:
        lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp = gps.getdata()
        print(lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp)