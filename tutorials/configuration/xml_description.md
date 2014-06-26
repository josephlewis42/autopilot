Autopilot XML Description
=========================

Version: 2014-06-26 by Joseph Lewis <joseph@josephlewis.net>

The UDenver autopilot has consolidated what were once three independent XML files in to a
single source for configuration. This configuration is stored in `conig.xml`.

If you do not have a `config.xml` file included with the autopilot, simply run it once and
it will be generated.

All configuration occurs under the root node `<configuration>`. Continaed within it are two
general types of nodes, **driver** nodes and **logic** nodes. **Driver** nodes are used for
configuring different subsystems of the autopilot including the IMU, GPS, Waypoint Manager and
System Logging mechanism. **Logic** nodes are used for storing parameters about the helicopter
such as the mass, PID values and offsets from the center of gravity.


Driver Nodes
============

Each driver node has a few common configuration elements, these will be mentioned first, then
the specific elements for each individual driver.

Common Driver Elements
----------------------

* `debug`
    * Values: true/false
    * Use: Enables/Disables logging of `trace` messages as `debug` values in the output. If
    turned on for every item, these values may overtake the logging mechanism.
* `enable`
    * Values: true/false
    * Use: Enables/Disables this driver.
    * Note: some drivers do not comply with this setting, but most for removable devices should i.e. the IMU
    * Note: In some old configuration files you may see both `enable` and `enabled`, it should be safe to
    remove the `enabled` tag and just use the `enable` one.
* `terminate_if_init_failed`
    * Values: true/false
    * Use: Alerts the driver that if initialization fails it should try to halt the system. Use this
    as a failsafe in the field to ensure all critical drivers run before trying to fly.
    * Note: Not all drivers use this.
* `read_style`
    * Values: 0,1,2,3
    * Use: alerts the autopilot how the device that the driver depends on should be read
    * Value 0: the port should continue to be read until the requested number of bytes hs been received
    * Value 1: the port should continue to be read until the number of requested bytes has been read or
    a timeout is reached.
    * Value 2: perform a system `read()` call returning as much as possible.
    * Value 3: using the baudrate of the port, wait as long as it should take to transmit the requested
    number of bytes, then perform a `read()`

`servo` specific elements
-------------------------

This configuration is used to modify the servo switch.

* `serial_port`
    * Values: any valid serial port path on the system, usually starts with `/dev/tty`
    * Use: alerts the driver where it should read the values from.

`gx3` specific elements
-----------------------

This configuration is used to modify the GX3 IMU.


* `serial_port`
    * Values: any valid serial port path on the system, usually starts with `/dev/tty`
    * Use: alerts the driver where it should read the values from.
* `termianal > IMU1 > baudrate`
    * Values: 9600, 19200, 38400, 57600, 115200, (others if supported by operating system and chipset)
    * Use: Sets the baudrate trying to be used by the device
* `terminal > IMU1 > parity`
    * Values: 8N1, 7E1, 7O1, 7M1, 7S1
    * Use: Sets the number of bits, parity check, and stop bits of the terminal
    * Start Bits: 8/7
    * Stop Bits: 1
    * Parity: **N**one, **E**ven, **O**dd, **M**ark, **S**pace
* `terminal > IMU1 > hardware_flow_control`
    * Values: true/false
    * Use: Enables/Disables hardware flow control for this device.
    * Note: This should almost always be set to false. Must be set to false with
    three wire serial cables (only TX/RX/GND connected)
* `terminal > IMU1 > raw_mode`
    * Values: true/false
    * Use: Enables/disables raw_mode for this device.
    * Note: Always set to True unless explicitly asked for non-raw mode.
* `vehicle_offset_x_meters`
    * Values: All Real Numbers
    * Use: Sets the x offset of the gx3 on the vehicle in meters
* `vehicle_offset_y_meters`
    * Values: All Real Numbers
    * Use: Sets the y offset of the gx3 on the vehicle in meters
* `vehicle_offset_z_meters`
    * Values: All Real Numbers
    * Use: Sets the z offset of the gx3 on the vehicle in meters
* `antenna_offset_x_meters`
    * Values: All Real Numbers
    * Use: Sets the x offset of the gx3 antenna on the vehicle in meters
* `antenna_offset_y_meters`
    * Values: All Real Numbers
    * Use: Sets the y offset of the gx3 antenna on the vehicle in meters
* `antenna_offset_z_meters`
    * Values: All Real Numbers
    * Use: Sets the z offset of the gx3 antenna on the vehicle in meters
* `use_external_gps`
    * Values: true/false
    * Use: Tells the GX3 to use an external gps when doing its filtering
    if true, otherwise uses the internal GPS.
* `position_message_rate_hz`
    * Values: Non-Negative Integers
    * Use: Sets the number of times the gx3 specific position message will be
    sent by this driver in hertz.
* `attitude_message_rate_hz`
    * Values: Non-Negative Integers
    * Use: Sets the number of times/sec the gx3 specific attitude message will
    be transmitted to qgroundcontrol.


`qgroundcontrol` specific elements
----------------------------------

This configuration is used to modify the mavlink communication channel established
with QGroundControl.


* `host > ip`
    * Values: Any valid IPV4 address
    * Use: The autopilot transmits mavlink messages to this host.
* `host > port`
    * Values: Any valid UDP port specifier 1 - 65536
    * Use: Sets the port that the mavlink messages will be sent to.
    * Note: QGroundControl by default listens on 14550
* `UASidentifier`
    * Values: 100
    * Use: This sets the UAS identifier that will be sent to QGroundControl
    * Note: As of 2014-06-26 this will not change all identifiers throughout
    the autopilot system, meaning it may not fully work.

`novatel` specific elements
---------------------------

This configuration is used to modify the properties of the NovAtel GPS unit.


* `enable_fallback_gps`
    * IGNORE THIS TAG, IT IS BEING REMOVED
* `log_fallback_gps`
    * IGNORE THIS TAG, IT IS BEING REMOVED


`mdl_altimeter` specific elements
---------------------------------

This configuration is used to modify the properties of the MDL altimeter.


* `device`
    * Values: any valid serial port path on the system, usually starts with `/dev/tty`
    * Use: alerts the driver where it should read the values from.
* `termianal > Altimeter1 > baudrate`
    * Values: 9600, 19200, 38400, 57600, 115200, (others if supported by operating system and chipset)
    * Use: Sets the baudrate trying to be used by the device
* `terminal > Altimeter1 > parity`
    * Values: 8N1, 7E1, 7O1, 7M1, 7S1
    * Use: Sets the number of bits, parity check, and stop bits of the terminal
    * Start Bits: 8/7
    * Stop Bits: 1
    * Parity: **N**one, **E**ven, **O**dd, **M**ark, **S**pace
* `terminal > Altimeter1 > hardware_flow_control`
    * Values: true/false
    * Use: Enables/Disables hardware flow control for this device.
    * Note: This should almost always be set to false. Must be set to false with
    three wire serial cables (only TX/RX/GND connected)
* `terminal > Altimeter1 > raw_mode`
    * Values: true/false
    * Use: Enables/disables raw_mode for this device.
    * Note: Always set to True unless explicitly asked for non-raw mode.


`tcpserial` specific elements
-----------------------------

The TCP Serial driver allows all data recieved on a TCP port to be forwarded to a serial
port and vice-versa.

* `serial_path`
    * Values: any valid serial port path on the system, usually starts with `/dev/tty`
    * Use: alerts the driver where it should read the values from.
* `termianal > serial_settings > baudrate`
    * Values: 9600, 19200, 38400, 57600, 115200, (others if supported by operating system and chipset)
    * Use: Sets the baudrate trying to be used by the device
* `terminal > serial_settings > parity`
    * Values: 8N1, 7E1, 7O1, 7M1, 7S1
    * Use: Sets the number of bits, parity check, and stop bits of the terminal
    * Start Bits: 8/7
    * Stop Bits: 1
    * Parity: **N**one, **E**ven, **O**dd, **M**ark, **S**pace
* `terminal > serial_settings > hardware_flow_control`
    * Values: true/false
    * Use: Enables/Disables hardware flow control for this device.
    * Note: This should almost always be set to false. Must be set to false with
    three wire serial cables (only TX/RX/GND connected)
* `terminal > serial_settings > raw_mode`
    * Values: true/false
    * Use: Enables/disables raw_mode for this device.
    * Note: Always set to True unless explicitly asked for non-raw mode.

* `TCP_PORT`
    * Values: Any valid TCP port specifier 1 - 65536
    * Use: This is the port that the bridge will listen on for incoming data
    * Note: if you choose a port below 1024 you may need extra permissions
    from the operating system to use it.
* `TCP_backlog`
    * Values: Non-Zero Integers
    * Use: Tells the autopilot how many backup connections can be waiting
    in case the first one terminates, further attempts will simply be
    dropped.

`linux_cpu_info` specific elements
----------------------------------

This driver transmits system information back to the ground station, including CPU and
memory usage statistics.

**No Specific Elements**


`common_messages` specific elements
-----------------------------------

Common Messages transmits the Mavlink common messages including battery state,
system status, etc.

* `send_system_status_message`
    * Values: true/false
    * Use: whether or not to send the system_status_message
* `message_send_rate_hz`
    * Values: non-zero Integers
    * Use: The rate (in hertz) to send messages back to QGroundControl


`waypoint_manager` specific elements
-----------------------------------

The Waypoint Manager communicates with QGroundControl to accept waypoints that have
been set through the interface.

**No Specific Elements**




Logic Nodes
===========

The logic nodes have been copied mostly verbatim from Bryan Godbolt's original XML
implementation.

`channels` Element
------------------

* `two > gyro > value`
    * Values: unkonwn
    * Use: contains the calibration data for the pulse normalization functions
    * Units: unknown
* `three > aileron > value`
    * Values: unkonwn
    * Use: contains the calibration data for the pulse normalization functions
    * Units: unknown
* `three > elevator > value`
    * Values: unkonwn
    * Use: contains the calibration data for the pulse normalization functions
    * Units: unknown
* `three > rudder > value`
    * Values: unkonwn
    * Use: contains the calibration data for the pulse normalization functions
    * Units: unknown
* `five > throttle > value`
    * Values: unkonwn
    * Use: contains the calibration data for the pulse normalization functions
    * Units: unknown
* `five > pitch > value`
    * Values: unkonwn
    * Use: contains the calibration data for the pulse normalization functions
    * Units: unknown

`controller_params` Element
---------------------------

* `mix > roll`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `mix > pitch`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `mode`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `trajectory`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > roll > gain > proportional`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > roll > gain > derivative`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > roll > gain > integral`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > roll > trim`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > pitch > gain > proportional`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > pitch > gain > derivative`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > pitch > gain > integral`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `attitude_pid > pitch > trim`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_pid > x > proportional`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_pid > x > derivitive`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_pid > x > integral`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_pid > y > proportional`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_pid > y > derivitive`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_pid > y > integral`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_pid > travel`
    * Values: unkonwn
    * Use: unknown
    * Units: unknown
* `translation_outer_sbf > ned_x > proportional`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_sbf > ned_x > derivitive`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_sbf > ned_x > integral`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_sbf > ned_y > proportional`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_sbf > ned_y > derivitive`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_sbf > ned_y > integral`
    * Values: unknown
    * Use: unknown
    * Units: Unknown
* `translation_outer_sbf > travel`
    * Values: unkonwn
    * Use: unknown
    * Units: Unknown
* `line > xtravel`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `line > ytravel`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `line > hover`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `line > speed`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `circle > radius`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `circle > hover`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `circle > speed`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > mass`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > main_hub_offset > x`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > main_hub_offset > y`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > main_hub_offset > z`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > tail_hub_offset > x`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > tail_hub_offset > y`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > tail_hub_offset > z`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > inerta > x`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > inerta > y`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
* `physical_params > inerta > z`
    * Values: unknown
    * Use: Unknown
    * Units: Unknown
