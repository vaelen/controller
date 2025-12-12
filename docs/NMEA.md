# NMEA GPS Data Format

## Message structure

All transmitted data are printable ASCII characters between 0x20 (space) to 0x7e (~)
Data characters are all the above characters except the reserved characters (See next line)
Reserved characters are used by NMEA0183 for the following uses:

| ASCII | Hex  | Dec | Use                                                                        |
|-------|------|-----|----------------------------------------------------------------------------|
| <CR>  | 0x0d | 13  | Carriage return                                                            |
| <LF>  | 0x0a | 10  | Line feed, end delimiter                                                   |
| !     | 0x21 | 33  | Start of encapsulation sentence delimiter                                  |
| $     | 0x24 | 36  | Start delimiter                                                            |
| *     | 0x2a | 42  | Checksum delimiter                                                         |
| ,     | 0x2c | 44  | Field delimiter                                                            |
| \     | 0x5c | 92  | TAG block delimiter                                                        |
| ^     | 0x5e | 94  | Code delimiter for HEX representation of ISO/IEC 8859-1 (ASCII) characters |
| ~     | 0x7e | 126 | Reserved                                                                   |

- Messages have a maximum length of 82 characters, including the $ or ! starting character and the ending <LF>
- The start character for each message can be either a $ (For conventional field delimited messages) or ! (for messages that have special encapsulation in them)
- The next five characters identify the talker (two characters) and the type of message (three characters).
- All data fields that follow are comma-delimited.
- Where data is unavailable, the corresponding field remains blank (it contains no character before the next delimiter – see Sample file section below).
- The first character that immediately follows the last data field character is an asterisk, but it is only included if a checksum is supplied.
- The asterisk is immediately followed by a checksum represented as a two-digit hexadecimal number. The checksum is the bitwise exclusive OR of ASCII codes of all characters between the $ and *, not inclusive. According to the official specification, the checksum is optional for most data sentences, but is compulsory for RMA, RMB, and RMC (among others).
- <CR><LF> ends the message.

## NMEA sentence format

The main talker ID includes:
- GA: Galileo
- GB: BeiDou
- GP: GPS
- GL: GLONASS. When more than one constellation is used.
- GN: Combined GNSS position, for example, GPS and GLONASS.
- GQ: QZSS

NMEA messages mainly include the following "sentences" in the NMEA message:
| Sentence       | Description                                |
|----------------|--------------------------------------------|
| $Talker ID+GGA | Global Positioning System Fixed Data       |
| $Talker ID+GLL | Geographic Position—Latitude and Longitude |
| $Talker ID+GSA | GNSS DOP and active satellites             |
| $Talker ID+GSV | GNSS satellites in view                    |
| $Talker ID+RMC | Recommended minimum specific GPS data      |
| $Talker ID+VTG | Course over ground and ground speed        |

For example, the sentence for Global Positioning System Fixed Data for GPS should be "$GPGGA". 

## GGA sentence structure

Global Positioning System Fix Data

| Name                                         | Example Data   | Description                                      |
|----------------------------------------------|----------------|--------------------------------------------------|
| Sentence Identifier                          | $GPGGA         | Global Positioning System Fix Data               |
| Time                                         | 170834         | 17:08:34 Z                                       |
| Latitude                                     | 4124.8963, N   | 41d 24.8963' N or 41d 24' 54" N                  |
| Longitude                                    | 08151.6838, W  | 81d 51.6838' W or 81d 51' 41" W                  |
| Fix Quality: 0 = Invalid, 1 = GPS , 2 = DGPS | 1              | Data is from a GPS fix                           |
| Number of Satellites                         | 05             | 5 Satellites are in view                         |
| Horizontal Dilution of Precision (HDOP)      | 1.5            | Relative accuracy of horizontal position         |
| Altitude                                     | 280.2, M       | 280.2 meters above mean sea level                |
| Height of geoid above WGS84 ellipsoid        | -34.0, M       | -34.0 meters                                     |
| Time since last DGPS update                  | blank          | No last update                                   |
| DGPS reference station id                    | blank          | No station id                                    |
| Checksum                                     | 75             | Used by program to check for transmission errors |

Global Positioning System Fix Data. Time, position and fix related data for a GPS receiver.

Sentence format:

```nmea
$--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
```

Fields:
- hhmmss.ss = UTC of position
- llll.ll = latitude of position
- a = N or S
- yyyyy.yy = Longitude of position
- a = E or W
- x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
- xx = number of satellites in use
- x.x = horizontal dilution of precision
- x.x = Antenna altitude above mean-sea-level
- M = units of antenna altitude, meters
- x.x = Geoidal separation
- M = units of geoidal separation, meters
- x.x = Age of Differential GPS data (seconds)
- xxxx = Differential reference station ID
- hh = Checksum

Example:

```
$GNGGA,031622.000,3535.2305,N,13929.4041,E,1,18,0.63,65.1,M,39.4,M,,*43
```

In this example, the fix data is coming from the GNSS talker ID which means that the receiver is using multiple constellations.

## RMC sentence structure

Recommended Minimum Specific GNSS Data

| Name                                         | Example Data   | Description                                      |
|----------------------------------------------|----------------|--------------------------------------------------|
| Sentence Identifier                          | $GNRMC         | Recommended Minimum sentence                     |
| Time                                         | 031622.000     | 03:16:22.000 UTC                                 |
| Status                                       | A              | A=Active (valid), V=Void (invalid)               |
| Latitude                                     | 3535.2305, N   | 35d 35.2305' N                                   |
| Longitude                                    | 13929.4041, E  | 139d 29.4041' E                                  |
| Speed over ground                            | 0.00           | Speed in knots                                   |
| Course over ground                           | 328.71         | Course in degrees                                |
| Date                                         | 121225         | 12 December 2025 (ddmmyy)                        |
| Magnetic variation                           | blank          | Magnetic variation degrees                       |
| Magnetic variation direction                 | blank          | E or W                                           |
| Mode indicator                               | A              | A=Autonomous, D=Differential, E=Estimated        |
| Checksum                                     | 79             | XOR of all characters between $ and *            |

Sentence format:

```nmea
$--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a,a*hh
```

Fields:
- hhmmss.ss = UTC of position fix
- A = Status (A=active, V=void)
- llll.ll = Latitude of position
- a = N or S
- yyyyy.yy = Longitude of position
- a = E or W
- x.x = Speed over ground in knots
- x.x = Course over ground in degrees
- ddmmyy = Date (day, month, year)
- x.x = Magnetic variation in degrees
- a = E or W of magnetic variation
- a = Mode indicator (A=autonomous, D=differential, E=estimated, N=not valid)
- hh = Checksum

Example:

```
$GNRMC,031622.000,A,3535.2305,N,13929.4041,E,0.00,328.71,121225,,,A*79
```

## Example data

```nmea
$GNGLL,3535.2305,N,13929.4041,E,031621.000,A,A*45
$GNRMC,031622.000,A,3535.2305,N,13929.4041,E,0.00,328.71,121225,,,A*79
$GNVTG,328.71,T,,M,0.00,N,0.00,K,A*2C
$GNGGA,031622.000,3535.2305,N,13929.4041,E,1,18,0.63,65.1,M,39.4,M,,*43
$GPGSA,A,3,29,195,25,03,31,194,32,28,16,,,,0.94,0.63,0.70*0D
$BDGSA,A,3,08,13,14,26,06,29,22,21,07,,,,0.94,0.63,0.70*1E
$GPGSV,3,1,11,194,72,169,26,28,69,011,37,32,52,174,29,26,49,263,16*4F
$GPGSV,3,2,11,31,48,315,32,29,47,078,28,25,28,051,40,16,16,241,27*72
$GPGSV,3,3,11,03,11,304,24,195,06,178,22,49,,,*43
$BDGSV,5,1,18,16,53,315,,26,52,298,18,06,50,310,20,01,48,172,*60
$BDGSV,5,2,18,21,41,215,30,04,41,148,,03,39,224,24,09,37,298,*6C
$BDGSV,5,3,18,29,29,084,31,13,23,241,23,14,23,312,32,02,21,250,*6F
$BDGSV,5,4,18,08,20,225,26,07,17,201,23,24,11,271,,22,07,174,25*68
$BDGSV,5,5,18,30,06,039,,10,04,208,*61
$GNGLL,3535.2305,N,13929.4041,E,031622.000,A,A*46
$GNRMC,031623.000,A,3535.2305,N,13929.4041,E,0.00,328.71,121225,,,A*78
$GNVTG,328.71,T,,M,0.00,N,0.00,K,A*2C
$GNGGA,031623.000,3535.2305,N,13929.4041,E,1,18,0.63,65.1,M,39.4,M,,*42
$GPGSA,A,3,29,195,25,03,31,194,32,28,16,,,,0.94,0.63,0.70*0D
$BDGSA,A,3,08,13,14,26,06,29,22,21,07,,,,0.94,0.63,0.70*1E
$GPGSV,3,1,11,194,72,169,26,28,69,011,37,32,52,174,29,26,49,263,18*41
$GPGSV,3,2,11,31,48,315,32,29,47,078,28,25,28,051,40,16,16,241,27*72
$GPGSV,3,3,11,03,11,304,24,195,06,178,21,49,,,*40
$BDGSV,5,1,18,16,53,315,,26,52,298,17,06,50,310,20,01,48,172,*6F
$BDGSV,5,2,18,21,41,215,30,04,41,148,,03,39,224,24,09,37,298,*6C
$BDGSV,5,3,18,29,29,084,31,13,23,241,23,14,23,312,32,02,21,250,*6F
$BDGSV,5,4,18,08,20,225,26,07,17,201,23,24,11,271,,22,07,174,26*6B
$BDGSV,5,5,18,30,06,039,,10,04,208,*61
$GNGLL,3535.2305,N,13929.4041,E,031623.000,A,A*47
$GNRMC,031624.000,A,3535.2305,N,13929.4041,E,0.00,328.71,121225,,,A*7F
$GNVTG,328.71,T,,M,0.00,N,0.00,K,A*2C
$GNGGA,031624.000,3535.2305,N,13929.4041,E,1,19,0.62,65.1,M,39.4,M,,*45
$GPGSA,A,3,29,195,25,26,03,31,194,32,28,16,,,0.93,0.62,0.69*07
$BDGSA,A,3,08,13,14,26,06,29,22,21,07,,,,0.93,0.62,0.69*10
$GPGSV,3,1,11,194,72,169,26,28,69,011,37,32,52,174,29,26,49,263,18*41
$GPGSV,3,2,11,31,48,315,32,29,47,078,28,25,28,051,40,16,16,241,27*72
```
