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
            
            setrate(rate, measurements per nav solution) - Enables you to set the rate of data output from the module
            modulesetup() - Sets up the module to the settings required for my usage. Can be adapted to suit other uses. For the NEO-M8N and NEO-M8Q, this will need to be adjusted slightly. See the method for details.
        
        modulesetup() and setrate() return True, False or None depending on whether it received an ACK, NACK or nothing from the module.
        """

        self.gps = UART(2, baudrate=9600, tx=tx_pin, rx=rx_pin)
        
        self.data = {}
    
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
        
    def _update_data(self):
        num_sentences_read = 0
        
        while num_sentences_read < 5:
            new_data = self.gps.read(1)
            while new_data != b'$':
                new_data = self.gps.read(1)
            
            while not new_data[-1:] == b'\n':
                data_byte = self.gps.read(1)
                if data_byte:
                    new_data += data_byte
            
            if self._checksum(new_data):
                new_data = new_data.decode('utf-8')
                self.data[new_data[3:6]] = new_data
                
            num_sentences_read += 1
    
    def position(self):
        """
        Returns a list of [latitude, longitude, horizontal error, timestamp] all formatted as strings.
        
        Latitude/longitude are as degrees, with N/S or E/W
        Position error is in meters - fix error in 2D
        Timestamp (UTC) is formatted as hh:mm:ss
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
            lat = str(degrees+minutes/60) + gll_sentence[2]
            
            pos_minutes = gll_sentence[3].find(".")-2
            minutes = float(gll_sentence[3][pos_minutes:])
            degrees = int(gll_sentence[3][:pos_minutes])
            long = str(degrees+minutes/60) + gll_sentence[4]
            
            # This is the 2D horizontal position error
            hdop = float(gsa_sentence[-3])
            position_error = hdop * 2.5 # 68% confidence level, 1 sigma - estimated accuracy of GPS module is ~2.5m from datasheet
            
            return lat, long, position_error, time_stamp
        return 0, 0, 0, time_stamp
        
    def velocity(self, data_needs_updating=True):
        """
        Returns a list of [speed over ground, course over ground, magnetic variation, timestamp] all formatted as strings
        
        SOG is in Knots (Kn)
        COG is in degrees
        Magnetic variation is in degrees
        Timestamp (UTC) formatted as hh:mm:ss
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

            if rmc_sentence[10] or rmc_sentence[11]:
                mag_variation = rmc_sentence[10]+rmc_sentence[11] + "°"
            else:
                mag_variation = "0°"
            
            return sog, cog, mag_variation, time_stamp
        return 0, 0, 0, time_stamp

    def altitude(self, data_needs_updating=True):
        """
        Returns a list of [altitude, geoid separation, vertical error, timestamp] all as strings
        
        Altitude is in meters
        Geoid separation is in meters
        Vertical error is in meters
        Timestamp (UTC) is formatted as hh:mm:ss
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
            alt = gga_sentence[9] + "M AMSL"
            geo_sep = gga_sentence[11] + "M"
            
            vdop = float(gsa_sentence[-2])
            
            vertical_error = vdop * 2.5 # 68% confidence level, 1 sigma - estimated accuracy of GPS module is ~2.5m from datasheet
            
            return alt, geo_sep, vertical_error, time_stamp
        
        return 0, 0, 0, time_stamp
    
    def getdata(self):
        """
        Returns list of [latitude, longitude, altitude, position error, sog, cog, magnetic variation, geoid separation, timestamp]
        
        Latitude/longitude are as degrees, with N/S or E/W
        Altitude is in meters
        Position error is in meters - the error of the fix in 3D
        SOG is in Knots (Kn)
        COG is in degrees
        Magnetic variation is in degrees
        Geoid separation is in meters
        Timestamp (UTC) is formatted as hh:mm:ss
        """
        
        lat, long, position_error, timestamp_0 = self.position()
        sog, cog, mag_variation, timestamp_1 = self.velocity(data_needs_updating=False)
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
        
        total_error = 2.45 * (position_error*position_error + vertical_error*vertical_error)**0.5 # Combining errors into one 3D error. * 2.45 to get to ~95% confidence level (2 sigma)
        
        return lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp
    
    def _ubx_ack_nack(self):
        start = time.time()
        data = b''
        
        while time.time() < start+1:
            if self.gps.any() >= 10:
                data += self.gps.read()
                
                index = data.find(b'\xb5\x62\x05')
                if index != -1:
                    data = data[index:]
                    
                    if len(data) >= 4:
                        if data[3] == 0x01:
                            return True
                        if data[3] == 0x00:
                            return False
        return None
    
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
            - Galileo GNSS constellation enabled (as well as default GPS/GLONASS/SBAS)
            - Enabled interference detection
                - Broadband detection threshold is 7dB
                - Continuous wave detection threshold is 20dB
                - Active antenna
        """
        
        # Uses UBX-CFG-MSG sentence to disable the VTG NMEA sentence
        self.gps.write(b'\xb5\x62\x06\x01\x03\x00\xF0\x05\x00\xff\x19')
        flag = self._ubx_ack_nack()
        
        if flag:
            # Using UBX-CFG-NAV5 to set module to: airborne with <4g acceleration, 3D fix only, satellites 15 degrees above horizon to be used for a fix, static hold at <20cm/s and 1m, automatic UTC standard
            packet = b'\x06\x24' + b'$\x00' + b'G\x08' + b'\x08' + b'\x02' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x00' + b'\x14' + b'\x00' + b'\x00\x00' + b'\x00\x00' + b'\x00\x00' + b'\x00\x00' + b'\x14' + b'\x00' + b'\x00' + b'\x00' + b'\x00' + b'\x01' + b'\x00' + b'\x00\x00\x00\x00\x00\x00\x00'

            ck_a, ck_b = self._ubx_checksum(packet)
            packet = b'\xb5\x62' + packet + ck_a + ck_b
            self.gps.write(packet)
        
            flag = self._ubx_ack_nack()
            
        if flag:
            # Using UBX-CFG-NAVX5 to set module to: min. satellites for navigation=4, max. satellites for navigation=50, initial fix must be 3D, AssistNow Autonomous turned on, maximum AssistNow Autonomous orbit error=20m
            packet = b'\x06\x23' + b'(\x00' + b'\x00\x00' + b'D@' + b'\x00\x00\x00\x00' + b'\x00\x00' + b'\x04' + b'<' + b'\x00' + b'\x00' + b'\x01' + b'\x00' + b'\x00' + b'\x00\x00' + b'\x00\x00\x00\x00\x00\x00' + b'\x00' + b'\x01' + b'\x00\x00' + b'\x14\x00' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x00' + b'\x00'
        
            ck_a, ck_b = self._ubx_checksum(packet)
            packet = b'\xb5\x62' + packet + ck_a + ck_b
            self.gps.write(packet)
            
            flag = self._ubx_ack_nack()
        
        if 	flag:
            # Constructing UBX-CFG-GNSS message to enable Galileo GNSS constellation use as well as standard GPS/GLONASS/SBAS
            packet = b'\x06\x3e' + b'\x0c\x00' + b'\x00\x00\xff\x01' + b'\x02\x02\x08\x00\x01\x00\x10\x00'
            
            ck_a, ck_b = self._ubx_checksum(packet)
            packet = b'\xb5\x62' + packet + ck_a + ck_b
            self.gps.write(packet)
            
            flag = self._ubx_ack_nack()
        
        if flag:
            # Constructing UBX-CFG-ITFM message to configure interference/jamming monitoring on the module - enabling interference detection, broadband threshold=7dB, continuous wave threshold=20dB, active antenna
            packet = b'\x06\x39' + b'\x08\x00' + b'\xadb\xadG' + b'\x00\x00#\x1e'
            
            ck_a, ck_b = self._ubx_checksum(packet)
            packet = b'\xb5\x62' + packet + ck_a + ck_b
            self.gps.write(packet)
            
            flag = self._ubx_ack_nack()
            
        if flag:
            # Constructing UBX-CFG-CFG message: saving all the above configured settings into the module's programmable flash
            # This should be changed to saving into battery-backed RAM for NEO-M8Q and NEO-M8M which don't have programmable flash - do this by changing the byte b'\x02' below for the byte b'\x01' (assuming you have BBR, unless you want to save it into the SPI Flash)
            packet = b'\x06\x09' + b'\r\x00' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x1a' + b'\x00\x00\x00\x00' + b'\x02'
            
            ck_a, ck_b = self._ubx_checksum(packet)
            packet = b'\xb5\x62' + packet + ck_a + ck_b
            self.gps.write(packet)
            
            flag = self._ubx_ack_nack()
            
        return flag


if __name__ == "__main__":
    gps = GPSReceive(10, 9)
    
    flag = gps.setrate(2, 3)
    while not flag:
        flag = gps.setrate(2, 3)
    
    while True:
        lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp = gps.getdata()
        print(lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp)
