from machine import UART
from math import sqrt
import time, struct

class GPSReceive:
    def __init__(self, rx_pin, tx_pin):
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.gps = UART(2, baudrate=9600, tx=self.tx_pin, rx=self.rx_pin)
        
        self.data = {}
        
        # Uses UBX-CFG-MSG sentence to disable the VTG NMEA sentence
        self.gps.write(b'\xb5\x62\x06\x01\x03\x00\xF0\x05\x00\xff\x19')
    
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
        try:
            self._update_data()
        except UnicodeError:
            self._update_data()
            
        try:
            gll_sentence = self.data["GLL"].split(",")
            gsa_sentence = self.data["GSA"].split(",")
        except KeyError:
            return 0, 0, 0, 0
        # potential for there to be no GLL sentence, especially at first run of update_data code
        
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
            
            time_utc = gll_sentence[5]
            time_stamp = time_utc[:2] + ":" + time_utc[2:4] + ":" + time_utc[4:]
            
            # This is the 2D horizontal position error
            hdop = float(gsa_sentence[-3])
            position_error = hdop * 2.5 # 68% confidence level, 1 sigma - estimated accuracy of GPS module is ~2.5m from datasheet
            
            return lat, long, position_error, time_stamp
        return 0, 0, 0, 0
        
    def velocity(self, data_needs_updating=True):
        if data_needs_updating:
            try:
                self._update_data()
            except UnicodeError:
                self._update_data()
            
        try:
            rmc_sentence = self.data["RMC"].split(",")
        except KeyError:
            return 0, 0, 0, 0
        
        if rmc_sentence[2] == "A":
            time_utc = rmc_sentence[1]
            time_stamp = time_utc[:2] + ":" + time_utc[2:4] + ":" + time_utc[4:]
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
        return 0, 0, 0, 0

    def altitude(self, data_needs_updating=True):
        if data_needs_updating:
            try:
                self._update_data()
            except UnicodeError:
                self._update_data()
            
        try:
            gga_sentence = self.data["GGA"].split(",")
            gsa_sentence = self.data["GSA"].split(",")
        except KeyError:
            return 0, 0, 0, 0
        
        if gga_sentence[6] != "0":
            alt = gga_sentence[9] + "M AMSL"
            geo_sep = gga_sentence[11] + "M"
            
            time_utc = gga_sentence[1]
            time_stamp = time_utc[:2] + ":" + time_utc[2:4] + ":" + time_utc[4:]
            
            vdop = float(gsa_sentence[-2])
            
            vertical_error = vdop * 2.5 # 68% confidence level, 1 sigma - estimated accuracy of GPS module is ~2.5m from datasheet
            
            return alt, geo_sep, vertical_error, time_stamp
        
        return 0, 0, 0, 0
    
    def getdata(self):
        lat, long, position_error, timestamp = self.position()
        sog, cog, mag_variation, _ = self.velocity(data_needs_updating=False)
        alt, geo_sep, vertical_error, _ = self.altitude(data_needs_updating=False)
        
        total_error = 2.45 * sqrt(position_error*position_error + vertical_error*vertical_error) # Combining errors into one 3D error. * 2.45 to get to ~95% confidence level (2 sigma)
        
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
                    
                    if data[3] == 0x01:
                        return True
                    if data[3] == 0x00:
                        return False
        return None
    
    def setrate(self, rate, measurements_per_nav_solution):
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
        
    def setbaud(self, baud):
        # Putting together the bits of the packet that the checksum is calculated across. Second bit (b'\x00') is the Port ID
        #packet = b'\x06\x00\x14\x00' + b'\x00' + b'\x00\x00\x00\xd0\x08\x00\x00' + struct.pack("<L", baud) + b'\x01\x00\x02\x00\x00\x00\x00\x00'
        packet = b'\x06\x00\x14\x00' + b'\x00' + b'\x00' + b'\x00' + b'\x00' + struct.pack("<L", baud) + b'\x08\x00' + b'\x03\x00' + b'\x03\x00' + b'\x00\x00\x00\x00\x00\x00'
        
        ck_a, ck_b = self._ubx_checksum(packet)
        # Adding headers and checksum
        packet = b'\xb5\x62' + packet + ck_a + ck_b
        
        # Writing packet, then re-initialising the UART with the new baudrate
        self.gps.write(packet)
        
        # Checking for the ACK or NACK
        flag = self._ubx_ack_nack()
        if flag:
            # Re-initialising the UART if the module receives an ACk confirming the GPS has switched baud rate
            self.gps = UART(2, baudrate=baud, tx=self.tx_pin, rx=self.rx_pin)
        return flag
        

if __name__ == "__main__":
    gps = GPSReceive(10, 9)
    print(gps.setrate(2, 3))
    print(gps.setbaud(115200))
    while True:
        lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp = gps.getdata()
        print(lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp)