# Module to Host Computer
The module uploads data with Flag=0x61 (acceleration, angular velocity, angle) by default. For Flag=0x71 (magnetic field), a command to read the corresponding register must be sent to receive the data. Bluetooth upload data format: Maximum 28 bytes per upload.
# Acceleration, Angular Velocity, Angle Data Packet (Default)
| Data Packet Header 1Byte | Flag 1Byte | axL | axH | ...... | YawL | YawH |
| --- | --- | --- | --- | --- | --- | --- |
| 0x55 | Flag | 0xNN | 0xNN | ...... | 0xNN | 0xNN |

Note: 0xNN is the specific value received. The data is returned in the order of acceleration (X, Y, Z), angular velocity (X, Y, Z), and angle (X, Y, Z), with the low byte first and the high byte second. For Flag = 0x61, the 18-byte data content is acceleration, angular velocity, and angle.

| Address | Definition |
| --- | --- |
| 0x55 | Data Packet Header |
| 0x61 | Flag |
| axL | X-axis acceleration low 8 bits |
| axH | X-axis acceleration high 8 bits |
| ayL | Y-axis acceleration low 8 bits |
| ayH | Y-axis acceleration high 8 bits |
| azL | Z-axis acceleration low 8 bits |
| azH | Z-axis acceleration high 8 bits |
| wxL | X-axis angular velocity low 8 bits |
| wxH | X-axis angular velocity high 8 bits |
| wyL | Y-axis angular velocity low 8 bits |
| wyH | Y-axis angular velocity high 8 bits |
| wzL | Z-axis angular velocity low 8 bits |
| wzH | Z-axis angular velocity high 8 bits |
| RollL | X-axis angle low 8 bits |
| RollH | X-axis angle high 8 bits |
| PitchL | Y-axis angle low 8 bits |
| PitchH | Y-axis angle high 8 bits |
| YawL | Z-axis angle low 8 bits |
| YawH | Z-axis angle high 8 bits |
| YY | Year |
| MM | Month |
| DD | Day |
| HH | Hour |
| MN | Minute |
| SS | Second |
| MSL | Millisecond low 8 bits |
| MSH | Millisecond high 8 bits |

### Acceleration Calculation Method: Unit g 
ax=((axH&lt;&lt;8)|axL)/32768*16g (g is gravity, can be 9.8m/s2)

ay=((ayH&lt;&lt;8)|ayL)/32768*16g (g is gravity, can be 9.8m/s2) 

az=((azH&lt;&lt;8)|azL)/32768*16g (g is gravity, can be 9.8m/s2) 

### Angular Velocity Calculation Method: Unit °/s
wx=((wxH&lt;&lt;8)|wxL)/32768*2000(°/s) 

wy=((wyH&lt;&lt;8)|wyL)/32768*2000(°/s) 

wz=((wzH&lt;&lt;8)|wzL)/32768*2000(°/s) 

### Angle Calculation Method: Unit ° 
Roll (x-axis) = ((RollH&lt;&lt;8)|RollL)/32768*180(°) 

Pitch (y-axis) = ((PitchH&lt;&lt;8)|PitchL)/32768*180(°) 

Yaw (z-axis) = ((YawH&lt;&lt;8)|YawL)/32768*180(°) 

Note：
1. The coordinate system used for attitude angle calculation is the North-East-Down (NED) system. When the module is placed in the standard orientation (as shown in "4 Pin Description"), the X-axis points left, the Y-axis points forward, and the Z-axis points up. The Euler angle rotation order is defined as Z-Y-X, meaning rotation is first around the Z-axis, then the Y-axis, and finally the X-axis.

2. Although the range of the roll angle is ±180 degrees, due to the Z-Y-X rotation order, the pitch angle (Y-axis) range is only ±90 degrees when representing attitude. If the pitch exceeds 90 degrees, it will be mapped to a value less than 90 degrees, and the X-axis angle will become greater than 180 degrees. For a detailed explanation, please search for information on Euler angles and attitude representation.

3. The three axes are coupled. They only exhibit independent changes at small angles. At large angles, the attitude angles will change in a coupled manner. For example, when the Y-axis angle approaches 90 degrees, even if the attitude only rotates around the Y-axis, the X-axis angle will also change significantly. This is an inherent characteristic of using Euler angles for attitude representation.

Explanation: 
1. Data is sent in hexadecimal format, not ASCII. 

2. Each data value is transmitted as a low byte and a high byte, which are combined to form a signed short. For example, for X-axis acceleration Ax, AxL is the low byte and AxH is the high byte. The conversion method is: Data = ((short)DataH&lt;&lt;8) | DataL. It is crucial to cast DataH to a signed short before the bit shift, and the data type of Data must also be a signed short to correctly represent negative numbers.

### Time Calculation Method
Millisecond calculation formula: Millisecond = ((MSH&lt;&lt;8)|MSL)
Note: 1. Time values are in hexadecimal and need to be converted to decimal for interpretation.

# Single Return Register Data Packet
To receive a single data packet from a register, you must first send a read register command with the following format: | FF AA 27 XX 00 |

--XX refers to the corresponding register number. See the register table for values. Example send commands:

| Function | Command |
| --- | --- |
| Read Magnetic Field | FF AA 27 3A 00 |
| Read Quaternions | FF AA 27 51 00 |
| Read Temperature | FF AA 27 40 00 |
| Read Battery Level | FF AA 27 64 00 |

After sending this command, the module will return a data packet starting with 0x55 0x71, which contains the starting register address and data for the next 8 registers (always uploads 8 registers). The return data format is as follows: Start Register (2 Bytes) + Register Data (16 Bytes, 8 registers)

| Header | Flag | Start Register Address Low | Start Register Address High | 1st Register Data Low | 1st Register Data High | ...... | 8th Register Data Low | 8th Register Data High |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0x55 | 0x71 | RegL | RegH | 0xNN | 0xNN | ...... | 0xNN | 0xNN |

Note: 0xNN is the specific value received, low byte first, high byte second.


### Magnetic Field Output
Note: The magnetic field data calculated from the raw data is in milli-Gauss (mG), which is different from the unit displayed on the host computer software. To convert to the same unit as the host computer, use the calculation below. 1. Link to magnetic field raw data unit conversion formula.

| 0x55 | 0x71 | 0x3A | 0x00 | HxL | HxH | HyL | HyH | HzL | HzH..... |

Calculation Method: Unit mG
Magnetic Field (x-axis) Hx=(( HxH&lt;&lt;8)| HxL) 
Magnetic Field (y-axis) Hy=(( HyH &lt;&lt;8)| HyL) 
Magnetic Field (z-axis) Hz =(( HzH&lt;&lt;8)| HzL) 
Example: Send read magnetic field command from APP: FF AA 27 3A 00 (See 7.2.8 Read Register Value) 
Module returns data to APP: 55 71 3A 00 68 01 69 00 7A 00 00 00 00 00 00 00 00 00 00 00 (20 bytes total). 
Decoding bytes 5 to 10 using the above method yields: Magnetic field x=360, y=105, z=122.

### Quaternion Output
| 0x55 | 0x71 | 0x51 | 0x00 | QxL | QxH | QyL | QyH | QzL | QzH..... |

Calculation Method: 
Q0=((Q0H&lt;&lt;8)|Q0L)/32768 
Q1=((Q1H&lt;&lt;8)|Q1L)/32768 
Q2=((Q2H&lt;&lt;8)|Q2L)/32768 
Q3=((Q3H&lt;&lt;8)|Q3L)/32768 

Checksum: Sum=0x55+0x59+Q0L+Q0H+Q1L +Q1H +Q2L+Q2H+Q3L+Q3H

### Temperature Output
| 0x55 | 0x71 | 0x40 | 0x00 | TL | TH | ...... |

Temperature calculation formula: T=((TH&lt;&lt;8)|TL) /100 ℃ 
# Host Computer to Module

## Send Command:

### Read Register Value
| FF AA 27 XX 00 | Read Register Value |

--XX refers to the corresponding register. Example:
Read Magnetic Field: FF AA 27 3A 00 
Read Quaternions: FF AA 27 51 00 
Read Temperature: FF AA 27 40 00 
After sending this command, the module will return a data packet starting with 0x55 0x71, containing data for the requested register and the next 7 registers (8 total). See the return data format for details.

### Accelerometer and Magnetometer Calibration
| FF AA 01 01 00 | Accelerometer Calibration |

### Save Configuration
| FF AA 00 SAVE 00 | Save Configuration |

SAVE: 
0: Save current configuration 
1: Restore default configuration and save

### Set Return Rate
| FF AA 03 RATE 00 | Set Return Rate |

RATE: Return Rate
0x01: 0.1Hz 
0x02: 0.5Hz 
0x03: 1Hz 
0x04: 2Hz 
0x05: 5Hz 
0x06: 10Hz (Default) 
0x07: 20Hz 
0x08: 50Hz 
0x09: 100Hz 
0x0A: 200Hz

### Set Angle Reference
| FF AA 01 08 00 | Set Angle Reference |

A save command must be sent after this command.

### Set Z-axis Angle to Zero
| FF AA 01 04 00 | Set z-axis angle to zero |

Before sending this command, you must switch to the 6-axis algorithm for it to take effect.

### Set Installation Direction
| FF AA 23 ORIENT 00 | Set Installation Direction |

ORIENT: Installation Direction
0(0x00): Horizontal Installation
1(0x01): Vertical Installation (Y-axis arrow must point up)

### Read Battery Level
| FF AA 27 64 00 | Read Module Battery Level |

Voltage to battery percentage correspondence:

| Register Value | Voltage | Battery Percentage |
| --- | --- | --- |
| &gt;396 | &gt; 3.96V | 100% |
| 393-396 | 3.93V-3.96V | 90% |
| 387-393 | 3.87V-3.93V | 75% |
| 382-387 | 3.82V-3.87V | 60% |
| 379-382 | 3.79V-3.82V | 50% |
| 377-379 | 3.77V-3.79V | 40% |
| 373-377 | 3.73V-3.77V | 30% |
| 370-373 | 3.70V-3.73V | 20% |
| 368-370 | 3.68V-3.70V | 15% |
| 350-368 | 3.50V-3.68V | 10% |
| 340-350 | 3.40V-3.50V | 5% |
| &lt;340 | &lt;3.40V | 0% |

## Register Address Table

| Address | Symbol | Meaning |
| --- | --- | --- |
| 0x00 | SAVE | Save current configuration |
| 0x01 | CALSW | Calibration |
| 0x02 | Reserved | 
| 0x03 | RATE | Return data rate |
| 0x04 | BAUD | Serial port baud rate |
| 0x05 | AXOFFSET | X-axis acceleration offset |
| 0x06 | AYOFFSET | Y-axis acceleration offset |
| 0x07 | AZOFFSET | Z-axis acceleration offset |
| 0x08 | GXOFFSET | X-axis angular velocity offset |
| 0x09 | GYOFFSET | Y-axis angular velocity offset |
| 0x0a | GZOFFSET | Z-axis angular velocity offset |
| 0x0b | HXOFFSET | X-axis magnetic field offset |
| 0x0c | HYOFFSET | Y-axis magnetic field offset |
| 0x0d | HZOFFSET | Z-axis magnetic field offset |
| 0x0e | D0MODE | D0 Mode |
| 0x0f | D1MODE | D1 Mode |
| 0x10 | D2MODE | D2 Mode |
| 0x11 | D3MODE | D3 Mode |
| 0x12 | Reserved | 
| 0x13 | Reserved | 
| 0x14 | Reserved | 
| 0x15 | Reserved | 
| 0x16 | Reserved | 
| 0x17 | Reserved | 
| 0x18 | Reserved | 
| 0x19 | Reserved | 
| 0x1a | Reserved | 
| 0x1b | Reserved | 
| ...... | ...... | ...... |
| 0x30 | YYMM | Year, Month |
| 0x31 | DDHH | Day, Hour |
| 0x32 | MMSS | Minute, Second |
| 0x33 | MS | Millisecond |
| 0x34 | AX | X-axis acceleration |
| 0x35 | AY | Y-axis acceleration |
| 0x36 | AZ | Z-axis acceleration |
| 0x37 | GX | X-axis angular velocity |
| 0x38 | GY | Y-axis angular velocity |
| 0x39 | GZ | Z-axis angular velocity |
| 0x3a | HX | X-axis magnetic field |
| 0x3b | HY | Y-axis magnetic field |
| 0x3c | HZ | Z-axis magnetic field |
| 0x3d | Roll | X-axis angle |
| 0x3e | Pitch | Y-axis angle |
| 0x3f | Yaw | Z-axis angle |
| 0x40 | TEMP | Module temperature |
| 0x49 | Reserved |
| 0x4a | Reserved |
| 0x4b | Reserved |
| 0x4c | Reserved |
| 0x4d | Reserved |
| 0x4e | Reserved |
| 0x4f | Reserved |
| 0x50 | Reserved |
| 0x51 | Q0 | Quaternion Q0 |
| 0x52 | Q1 | Quaternion Q1 |
| 0x53 | Q2 | Quaternion Q2 |
| 0x54 | Q3 | Quaternion Q3 |


# WT901SDCL-BT50 Recording Protocol

## Read File List
Send: FF AA 5A 01 00
Return: 55 AA WIT__.TXT 0D 0A

## Read File Content
Read file length command: 55 BB WIT__.TXT 0D 0A
Return >> 55 BB WIT__.TXT 3A __Packet Count__ 0D 0A
(Packet Count: Total bytes in file / 200 Bytes per packet. E.g., Packet Count = 801 / 200 = 5, the 5th packet has only 1 byte)
(The read file length command must be sent before this, otherwise it won't know which file to read)

Start reading file command: FF AA 5B 01 00 (No return)

Read file content command: 55 FF 00 00 (55 FF Packet Count, Packet Count is 16-bit, max length 65536 packets, approx. 13MB)
55 FF 00 01
.......
55 FF FF FF

Return >> 
55 FF 00 00 ___________________200Bytes of data_______________________________________
55 FF 00 01 ___________________200Bytes of data_______________________________________
.......
55 FF FF FF ________Last packet data_________________________________

App stop reading file command: FF AA 5B 02 00 (No return) 