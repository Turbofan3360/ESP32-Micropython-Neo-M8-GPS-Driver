# ESP32 Micropython NEO-M8 GPS Driver #

### The Code: ###

This code reads data off of a UART pin on the ESP32. The Neo-M8 modules outputs NMEA sentences, which this code then converts into the useful data needed. Note that all outputs are as strings with their correct units. 

This code contains a modulesetup() method to configure the GPS module - this only needs to be run once per module. Please see below for details.

As standard, the module updates its navigation fixes at a rate of 1Hz. This can be changed by calling the setrate() method, which takes two arguments: the first is the navigation solution update rate, while the second is the number of measurements per navigation solution. The setrate() method returns True or False, depending on whether it received an ACK or a NACK (respectively) from the module. If None is returned, that means that the code didn't receive anything from the module.

The getdata() method is an aggregator - it calls the other methods (ensuring that they only process the NMEA sentences from one data frame). This returns all the data you can get from the module - including a combined, 3D position error to a 2σ confidence level. Other errors (returned from the position and altitude methods) are only to a 1σ confidence level.

If there are any issues with the data (i.e. the code can't process it), integer zeros will be returned for those values.

### Example Usage: ###

```python3
import gps_reading_data.py as gps

module = gps.GPSReceive(10, 9)

flag = module.modulesetup()
while not flag:
    flag = module.modulesetup()
```

The above code only needs to be run once per module. It configures the module's settings to be optimal for my usage, and then saves those settings into the programmable flash. If you have a NEO-M8Q or NEO-M8M, the settings need to be saved into bbattery-backed RAM - see the modulesetup() method for details of how to do this.

```python3
import gps_reading_data.py as gps

module = gps.GPSReceive(10, 9)
print(module.setrate(2, 3))

lat, long, position_error, time_stamp = module.position()
sog, cog, mag_variation, time_stamp = module.velocity()
alt, geo_sep, vertical_error, time_stamp = module.altitude()

lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp = module.get_data()

```

To initialise the driver - the parameters the driver expects is the ESP32 pin that the GPS' TX pin is connected to, followed by the pin the GPS' RX pin is connected to. The above code is an example usage of the driver.

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
 - Galileo GNSS constellation enabled (as well as default GPS/GLONASS/SBAS)
 - Enabled interference detection
     - Broadband detection threshold is 7dB
     - Continuous wave detection threshold is 20dB
     - Active antenna
 - All the above configured settings saved into the module's programmable flash

### References: ###
 - <https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf>
 - <https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf>
