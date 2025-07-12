# ESP32 Micropython NEO-M8 GPS Driver #

### The Code: ###

This code reads data off of a UART pin on the ESP32. The Neo-M8 modules outputs NMEA sentences, which this code then converts into the useful data needed. Note that all outputs are as strings with their correct units. 

As standard, the module updates its navigation fixes at a rate of 1Hz. This can be changed by calling the setrate() method, which takes two arguments: the first is the navigation solution update rate, while the second is the number of measurements per navigation solution. The setrate() method returns True or False, depending on whether it received an ACK or a NACK (respectively) from the module. If None is returned, that means that the code didn't receive anything from the module.

The getdata() method is an aggregator - it calls the other methods (ensuring that they only process the NMEA sentences from one data frame). This returns all the data you can get from the module - including a combined, 3D position error to a 2σ confidence level. Other errors (returned from the position and altitude methods) are only to a 1σ confidence level.

If there are any issues with the data (i.e. the code can't process it), integer zeros will be returned for those values.

The code automatically disables the VTG NMEA sentence type, as it contains data that is duplicated in other NMEA sentences - and so is redundant.

### Example Usage: ###

```python3
import gps_reading_data.py as gps

module = gps.GPSReceive(10, 9)
print(module.setrate(2, 3))

lat, long, position_error, time_stamp = module.position()
sog, cog, mag_variation, time_stamp = module.velocity()
alt, geo_sep, vertical_error, time_stamp = module.altitude()

lat, long, alt, total_error, sog, cog, mag_variation, geo_sep, timestamp = module.get_data()

```

For the initialization - the parameters the driver expects is the ESP32 pin that the GPS' TX pin is connected to, followed by the pin the GPS' RX pin is connected to.

### NMEA Sentence Types & Details: ###

GSV: positions of satellites in view
GLL: lat,NS,lon,EW,time,status,posMode
GSA: error in satellite positions

RMC: time, status, lat, NS, lon, EW, spd, cog, date, mv, mvEW, posMode, navStatus

mv = magnetic variation, cog = course over ground, spd = speed

GGA: time, lat, NS,l on, EW, quality, numSV, HDOP, alt, altUnit, sep, sepUnit, diffAge, diffStation

numsv = num satellites, HDOP = measurement of error in height, altUnit/sepUnit always meters, sep = geoid separation, diffAge/diffStation used with differential GPS

Lat/Lon: first two digits before decimal point and everything after is minutes, other digits beforehand are degrees

Status flags: V - no fix/GNSS fix but user limits exceeded/dead reckoning fix but user limits exceeded
A - Dead reckoning fix, RTK float, RTK fixed, 2D GNSS fix, 3D GNSS fix, Combined GNSS/dead reckoning fix

posMode flags (GLL/VTG): N - no fix/GNSS fix but user limits exceeded
E - dead reckoning fix/dead reckoning fix but user limits exceeded
A/D (A = autonomous, D = differential GPS) - 2D/3D GNSS fix/combined GNSS and dead reckoning fix

quality flags (GGA): 0 - no pos fix/GNSS fix but user limits exceeded
5 - RTK float, 4 - RTK fixed
1/2 (1 = autonomous GNSS, 2 = differential GNSS) - 2D/3D GNSS fix/combined GNSS and dead reckoning fix

### References: ###
 - <https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf>
 - <https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf>
