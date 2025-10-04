# ESP32 Micropython NEO-M8 GPS Driver #

### The Code: ###

This code reads data off of a UART pin on the ESP32. The Neo-M8 modules outputs NMEA sentences, which this code then converts into the useful data needed.

This code contains a modulesetup() method to configure the GPS module - this only needs to be run once per module. Please see below for details.

As standard, the module updates its navigation fixes at a rate of 1Hz. This can be changed by calling the setrate() method, which takes two arguments: the first is the navigation solution update rate, while the second is the number of measurements per navigation solution. The setrate() method returns True or False, depending on whether it received an ACK or a NACK (respectively) from the module. If None is returned, that means that the code didn't receive anything from the module.

To get faster GPS data reads, periodically call the update_buffer() method (takes very little time, call it without calling the main data processing methods). This simply loads data from the ESP32's UART RX buffer (defined as 128 bytes long, in this driver) into the driver's 512-byte sliding-window buffer of GPS data. This can reduce GPS data reads down to 0.002 seconds/read in my experiece, compared to 0.5-0.8 seconds/read when not calling the update_buffer() method regularly.

The getdata() method is an aggregator - it calls the other methods (ensuring that they only process the NMEA sentences from one data frame). This returns all the data you can get from the module - including a combined, 3D position error to a 2σ confidence level. Other errors (returned from the position and altitude methods) are only to a 1σ confidence level.

You can also call gnss_stop() and gnss_start() to stop/start the module's GNSS systems. gnss_stop() should be called before pulling the module's power, and these commands can also be used to reduce the module's power consumption when necessary.

If there are any issues with the data (i.e. the code can't process it), integer zeros will be returned for those values.

### Example Usage: ###

```python3
import gps_reading_data as gps

module = gps.GPSReceive(10, 9)

flag = module.modulesetup()
while not flag:
    flag = module.modulesetup()
```

The above code only needs to be run once per module. It configures the module's settings to be optimal for my usage, and then saves those settings into the programmable flash. If you have a NEO-M8Q or NEO-M8M, the settings need to be saved into bbattery-backed RAM - see the modulesetup() method for details of how to do this.

```python3
import gps_reading_data as gps

module = gps.GPSReceive(10, 9)
print(module.setrate(2, 3))

lat, long, position_error, time_stamp = module.position()
sog, cog, time_stamp = module.velocity()
alt, geo_sep, vertical_error, time_stamp = module.altitude()

lat, long, position_error, alt, vertical_error, sog, cog, geo_sep, timestamp = module.get_data()
```

To initialise the driver - the parameters the driver expects is the ESP32 pin that the GPS' TX pin is connected to, followed by the pin the GPS' RX pin is connected to. The above code is an example usage of the driver.

### Embedded C Module: ###

For higher performance, in the embedded_c_module folder you will find the .c, .h and .cmake files to compile the Neo-M8 driver into micropython firmware - there is a guide to compiling this below. 

This is currently still in development - only the update_buffer(), position(), velocity(), altitude() and getdata() methods are availible currently.

### Compiling the module into firmware: ###

To do this, you will need:
 - ESP-IDF cloned from github
 - Micropython cloned from github

1. Enter your esp-idf directory, and run ./install.sh (only needs to be run the first time)
2. Enter your esp-idf directory and run . ./export.sh (needs to be run every new terminal session)
3. Download the files from embedded_c_module and place them in a directory of your choosing
4. Enter your directory ~/micropython/ports/esp32 (can be replaced with whichever micropython board you are using)
5. Run the make command, specifying USER_C_MODULES=/path/to/NEO-M8_GPS/embedded_c_module (replace with your file path)

For me, with an ESP32-S3 that has octal SPIRAM, the full make command is:
```
make BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT USER_C_MODULES=/path/to/NEO-M8_GPS/embedded_c_module
```

### Settings the module is configured to: ###

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
 - All the above configured settings saved into the module's programmable flash

### References: ###
 - <https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf>
 - <https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf>
