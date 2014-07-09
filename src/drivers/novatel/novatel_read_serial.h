/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013 Joseph Lewis III <joehms22@gmail.com>
 *
 * This file is part of ANCL Autopilot.
 *
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#ifndef NOVATEL_READ_SERIAL_H_
#define NOVATEL_READ_SERIAL_H_

/* c headers */
#include <stdint.h>
#include <math.h>

/* STL Headers */
#include <vector>

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;

/* Project Headers */
#include "GPS.h"
#include "ThreadSafeVariable.h"

/**
 * @brief Class to send commands to, and receive data from the GPS unit.
 * The class parses the incoming data and stores the message in a
 * double vector.
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Aakash Vasudevan <avasudev@ualberta.ca>
 * @date April 27, 2012: rewrote and refactored to integrate novatel with gx3
 */
class GPS::ReadSerial
{
public:
    ReadSerial();

    void operator()();

    /**
     * Function to read data from the serial port and send the output data to
     * GPS class
     * Also, tracks the number of successive GPS data with solution type less than 34 through
     * the variables GPS::_pos_count and GPS::_vel_count
     */
    void readPort();

    std::string solStatusToString(uint32_t status);
    std::string posVelTypeToString(uint32_t type);
    enum OEM6_SOL_STATUS
    {
        SOL_COMPUTED = 0,
        INSUFFICIENT_OBS = 1,
        NO_CONVERGENCE = 2,
        SINGULARITY = 3,
        COV_TRACE = 4,
        COLD_START = 5,
        V_H_LIMIT = 6,
        VARIANCE = 7,
        INTEGRITY_WARNING = 8,
        PENDING = 9,
        INVALID_FIX = 10,
        UNAUTHORIZED = 11
    };

    enum OEM6_POS_VEL_TYPE
    {
        NONE = 0,
        FIXEDPOS=1,
        FIXEDHEIGHT=2,
        DOPPLER_VELOCITY=8,
        SINGLE=16,
        PSRDIFF=17,
        WAAS=18,
        PROPAGATED=19,
        OMNISTAR=20,
        L1_FLOAT=32,
        IONOFREE_FLOAT = 33,
        NARROW_FLOAT = 34,
        L1_INT=48,
        NARROW_INT=50,
        OMNISTART_HP =64,
        OMNISTAR_XP=65
    };


    // The possible errors that could be recieved when using the OEM6 devices.
    // p349 of the NovAtel OEM 6 Communication Manual
    enum OEM6_RETURN_CODE
    {
        OEM6_OK = 1,
        OEM6_REQUESTED_LOG_DOES_NOT_EXIST = 2,
        OEM6_NOT_ENOUGH_RESOURCES_IN_SYSTEM = 3,
        OEM6_DATA_PACKET_DOESNT_VERIFY = 4,
        OEM6_COMMAND_FAILED_ON_RECEIVER = 5,
        OEM6_INVALID_MESSAGE_ID = 6,
        OEM6_INVALID_MESSAGE = 7,
        OEM6_INVALID_CHECKSUM = 8,
        OEM6_MESSAGE_MISSING_FIELD = 9,
        OEM6_ARRAY_SIZE_FOR_FIELD_X_EXCEEDS_MAX = 10,
        OEM6_PARAMETER_IS_OUT_OF_RANGE = 11,
        OEM6_TRIGGER_NOT_VALID_FOR_THIS_LOG = 14,
        OEM6_AUTHCODE_TABLE_FULL_RELOAD_SOFTWARE = 15,
        OEM6_INVALID_DATE_FORMAT = 16,
        OEM6_INVALID_AUTHCODE_ENTERED = 17,
        OEM6_NO_MATCHING_MODEL_TO_REMOVE = 18,
        OEM6_NOT_VALID_AUTH_CODE_FOR_THAT_MODEL = 19,
        OEM6_CHANNEL_IS_INVALID = 20,
        OEM6_REQUESTED_RATE_IS_INVALID = 21,
        OEM6_WORD_HAS_NO_MASK_FOR_THIS_TYPE = 22,
        OEM6_CHANNELS_LOCKED_DUE_TO_ERROR = 23,
        OEM6_INJECTED_TIME_INVALID = 24,
        OEM6_COM_PORT_NOT_SUPPORTED = 25,
        OEM6_MESSAGE_IS_INCORRECT = 26,
        OEM6_INVALID_PRN = 27,
        OEM6_PRN_NOT_LOCKED_OUT = 28,
        OEM6_PRN_LOCKOUT_LIST_IS_FULL = 29,
        OEM6_PRN_ALREADY_LOCKED_OUT = 30,
        OEM6_MESSAGE_TIMED_OUT = 31,
        OEM6_UNKNOWN_COM_PORT_REQUESTED = 33,
        OEM6_HEX_STRING_NOT_FORMATTED_CORRECTLY = 34,
        OEM6_INVALID_BAUD_RATE = 35,
        OEM6_MESSAGE_IS_INVALID_FOR_THIS_MODEL = 36,
        OEM6_COMMAND_ONLY_VALID_IF_IN_NVM_FAIL_MODE = 40,
        OEM6_INVALID_OFFSET = 41,
        OEM6_MAXIMUM_NUMBER_OF_USER_MESSAGES_REACHED = 78,
        OEM6_GPS_PRECISE_TIME_IS_ALREADY_KNOWN = 84
    };

    // The GPS time codes from the NovAtel OEM6, NovAtel OEM6 communications manual p 32
    enum OEM6_GPS_TIME_CODE
    {
        OEM6_GPS_TIME_UNKNOWN = 20,
        OEM6_GPS_TIME_APPROXIMATE = 60,
        OEM6_GPS_TIME_COARSEADJUSTING = 80,
        OEM6_GPS_TIME_COARSE = 100,
        OEM6_GPS_TIME_COARSESTEERING = 120,
        OEM6_GPS_TIME_FREEWHEELING = 130,
        OEM6_GPS_TIME_FINEADJUSTING = 140,
        OEM6_GPS_TIME_FINE = 160,
        OEM6_GPS_TIME_FINEBACKUPSTEERING = 170,
        OEM6_GPS_TIME_FINESTEERING = 180,
        OEM6_GPS_TIME_SATTIME = 200
    };

    // A list of the OEM6 message codes supported from the OEM6 communications manual p 49
    enum OEM6_MESSAGE_CODE
    {
        OEM6_COMMAND_LOG = 1, //Requests a log
        OEM6_COMMAND_INTERFACEMODE = 3, //Sets interface type for a port, Receive (Rx)
        OEM6_COMMAND_COM = 4, //Sets COM port configuration
        OEM6_COMMAND_CLOCKADJUST = 15, //Enables clock adjustments
        OEM6_COMMAND_RESET = 18, //Performs a hardware reset
        OEM6_COMMAND_SAVECONFIG = 19, //Saves current configuration
        OEM6_COMMAND_FRESET = 20, //Resets receiver to factory default
        OEM6_COMMAND_MODEL = 22, //Switches receiver to a previously AUTHed model
        OEM6_COMMAND_ASSIGN = 27, //Assigns individual satellite channel
        OEM6_COMMAND_ASSIGNALL = 28, //Assigns all satellite channels
        OEM6_COMMAND_UNASSIGN = 29, //Unassigns a previously ASSIGNed channel
        OEM6_COMMAND_UNASSIGNALL = 30, //Unassigns all previously ASSIGNed channels
        OEM6_COMMAND_UNLOG = 36, //Removes a log from the logging control
        OEM6_COMMAND_UNLOGALL = 38, //Removes all logs from the logging control
        OEM6_COMMAND_FIX = 44, //Constrains receiver height or position
        OEM6_COMMAND_AUTH = 49, //Adds authorization code for new model
        OEM6_COMMAND_ECUTOFF = 50, //Sets satellite tracking elevation cut-off
        OEM6_COMMAND_USERDATUM = 78, //Sets user customized datum
        OEM6_COMMAND_RTKELEVMASK = 91, //Sets the RTK elevation mask
        OEM6_COMMAND_RTKSVENTRIES = 92, //Sets number of satellites in corrections
        OEM6_COMMAND_STATUSCONFIG = 95, //Configures various status mask fields in RXSTATUSEVENT log
        OEM6_COMMAND_RTKCOMMAND = 97, //Resets the RTK filter or sets the filter to default settings
        OEM6_COMMAND_ANTENNAPOWER = 98, //Controls power to Low Noise Amplifier (LNA) of an active antenna
        OEM6_COMMAND_SETAPPROXTIME = 102, //Sets an approximate GPS reference time
        OEM6_COMMAND_SETRTCM16 = 131, //Enters ASCII text for RTCM data stream
        OEM6_COMMAND_LOCKOUT = 137, //Prevents the receiver from using a satellite by specifying its PRN
        OEM6_COMMAND_UNLOCKOUT = 138, //Reinstates a satellite in the solution
        OEM6_COMMAND_UNLOCKOUTALL = 139, //Reinstates all previously locked out satellites
        OEM6_COMMAND_DGPSEPHEMDELAY = 142, //Sets DGPS ephemeris delay
        OEM6_COMMAND_DGPSTXID = 144, //Sets DGPS station ID
        OEM6_COMMAND_LOGFILE = 157, //Opens and closes log files in the internal flash memory
        OEM6_COMMAND_DATUM = 160, //Chooses a DATUM name type
        OEM6_COMMAND_SETNAV = 162, //Sets waypoints
        OEM6_COMMAND_POSAVE = 173, //Implements base station position averaging
        OEM6_COMMAND_SEND = 177, //Sends ASCII message to a port
        OEM6_COMMAND_SENDHEX = 178, //Sends non-printable characters to a port
        OEM6_COMMAND_MAGVAR = 180, //Sets magnetic variation correction
        OEM6_COMMAND_RTKDYNAMICS = 183, //Sets up the RTK dynamics mode
        OEM6_COMMAND_NVMRESTORE = 197, //Restores NVM data after a failure in NVM
        OEM6_COMMAND_SOFTPOWER = 213, //Shuts down the receiver
        OEM6_COMMAND_UNDULATION = 214, //Sets ellipsoid-geoid separation
        OEM6_COMMAND_EXTERNALCLOCK = 230, //Allows the OEM6 card to operate with an optional external oscillator
        OEM6_COMMAND_FREQUENCYOUT = 232, //Sets the output pulse train available on VARF
        OEM6_COMMAND_DYNAMICS = 258, //Adjusts tracking state transition time-out
        OEM6_COMMAND_SETAPPROXPOS = 377, //Sets an approximate position
        OEM6_COMMAND_APPLICATION = 413, //Starts/stops the application
        OEM6_COMMAND_PDPFILTER = 424, //Enables, disables or resets the Pseudorange/deltaphase filter
        OEM6_COMMAND_ADJUST1PPS = 429, //Adjusts the receiver clock
        OEM6_COMMAND_CLOCKCALIBRATE = 430, //Adjusts clock steering parameters
        OEM6_COMMAND_COMCONTROL = 431, //Controls the hardware control lines of the RS232 ports
        OEM6_COMMAND_SOFTLOADCOMMIT = 475, //Commits to the soft load module
        OEM6_COMMAND_SOFTLOADRESET = 476, //Restart soft load process
        OEM6_COMMAND_SOFTLOADSREC = 477, //Configures soft load process
        OEM6_COMMAND_PSRDIFFSOURCE = 493, //Sets the pseudorange correction source
        OEM6_COMMAND_RTKSOURCE = 494, //Sets the RTK correction source
        OEM6_COMMAND_CLOCKOFFSET = 596, //Adjusts for delay in 1PPS output
        OEM6_COMMAND_POSTIMEOUT = 612, //Sets the position time out
        OEM6_COMMAND_PPSCONTROL = 613, //Controls the PPS output
        OEM6_COMMAND_MARKCONTROL = 614, //Controls processing of the mark inputs
        OEM6_COMMAND_SBASCONTROL = 652, //Sets SBAS test mode and PRN
        OEM6_COMMAND_SETDIFFCODEBIASES = 687, //Sets satellite differential code biases
        OEM6_COMMAND_GGAQUALITY = 691, //Customizes the GPGGA GPS quality indicator
        OEM6_COMMAND_SETIONOTYPE = 711, //Sets the ionospheric corrections model
        OEM6_COMMAND_ASSIGNLBAND = 729, //Sets L-Band satellite communication parameters
        OEM6_COMMAND_GLOECUTOFF = 735, //Sets the GLONASS satellite elevation cut-off angle
        OEM6_COMMAND_UTMZONE = 749, //Sets UTM parameters
        OEM6_COMMAND_FIXPOSDATUM = 761, //Sets the position in a specified datum
        OEM6_COMMAND_MOVINGBASESTATION = 763, //Sets ability to use a moving base station
        OEM6_COMMAND_COMVOUT = 779, //Controls power to the COM ports
        OEM6_COMMAND_HPSTATICINIT = 780, //Sets static initialization of OmniSTAR HP/XP
        OEM6_COMMAND_HPSEED = 782, //Specifies the initial position for OmniSTAR HP/XP
        OEM6_COMMAND_USEREXPDATUM = 783, //Sets custom expanded datum
        OEM6_COMMAND_FORCEGPSL2CODE = 796, //Forces the receiver to track L2C or P-code
        OEM6_COMMAND_SETBESTPOSCRITERIA = 839, //Selects criteria for BESTPOS
        OEM6_COMMAND_RTKQUALITYLEVEL = 844, //Chooses an RTK quality mode
        OEM6_COMMAND_CNOUPDATE = 849, //Sets C/No update rate and resolution
        OEM6_COMMAND_RTKANTENNA = 858, //Specifies L1 phase center (PC) or ARP and enable/disable PC modeling
        OEM6_COMMAND_NMEATALKER = 861, //Sets the NMEA talker ID
        OEM6_COMMAND_BASEANTENNAMODEL = 870, //Enters/changes base antenna model
        OEM6_COMMAND_LOCKOUTSYSTEM = 871, //Prevents the receiver from using a system
        OEM6_COMMAND_SETRTCM36 = 880, //Enters ASCII text with Russian characters
        OEM6_COMMAND_UNLOCKOUTSYSTEM = 908, //Reinstates in the solution computation a system previously locked out
        OEM6_COMMAND_RTKTIMEOUT = 910, //Sets maximum age of RTK data
        OEM6_COMMAND_DIFFCODEBIASCONTROL = 913, //Enables or disable satellite differential code biases
        OEM6_COMMAND_LOCALIZEDCORRECTION_DATUM = 947, //Sets a local datum
        OEM6_COMMAND_RTKNETWORK = 951, //Specifies the RTK network mode
        OEM6_COMMAND_TUNNELESCAPE = 962, //Breaks out of an established tunnel
        OEM6_COMMAND_PDPMODE = 970, //Selects the PDP mode and dynamics
        OEM6_COMMAND_SBASECUTOFF = 1000, //Sets SBAS satellite elevation cut-off
        OEM6_COMMAND_SBASTIMEOUT = 1001, //Sets the amount of time the receiver remains in an
//		OEM6_COMMAND_position = SBAS, //if it stops receiving SBAS corrections
        OEM6_COMMAND_DLLTIMECONST = 1011, //Replaces the GLOCSMOOTH and CSMOOTH commands
        OEM6_COMMAND_HDTOUTTHRESHOLD = 1062, //Controls the NMEA GPHDT log output
        OEM6_COMMAND_HEADINGOFFSET = 1082, //Adds specified offset to heading and pitch values
        OEM6_COMMAND_SETCANNAME = 1091, //Sets the CAN name fields
        OEM6_COMMAND_GALECUTOFF = 1114, //Sets elevation cut-off angle for Galileo satellites
        OEM6_COMMAND_SETROVERID = 1135, //Sets the Rover ID output in ROVERPOS, HEADI
        OEM6_COMMAND_SELECTCHANGCONFIG = 1149, //Sets the channel configuration
        OEM6_COMMAND_SETUTCLEAPSECONDS = 1150, //Sets when future leap seconds take effect
        OEM6_COMMAND_OMNIUSEGLONASS = 1199, //Enables/disables GLONASS in OmniStar
        OEM6_COMMAND_ASSIGNLBAND2 = 1200, //Sets L-Band satellite communication parameters
        OEM6_COMMAND_IONOCONDITION = 1215, //Sets ionospheric condition
        OEM6_COMMAND_SETRTCMRXVERSION = 1216, //Sets the RTCM transmission standard
        OEM6_COMMAND_FORCEGLOL2CODE = 1217, //Forces the receiver to track L2C or P-code
        OEM6_COMMAND_SOFTLOADDATA = 1218, //Uploads data for soft load
        OEM6_COMMAND_SOFTLOADSETUP = 1219, //Configures the soft load process
        OEM6_COMMAND_SETTIMEBASE = 1237, //Sets primary and backup systems for time base
        OEM6_COMMAND_IPCONFIG = 1243, //Configures network IP settings
        OEM6_COMMAND_DNSCONFIG = 1244, //Manually configures DNS servers
        OEM6_COMMAND_ETHCONFIG = 1245, //Configures Ethernet physical layer
        OEM6_COMMAND_SERIALCONFIG = 1246, //Configures serial port settings
        OEM6_COMMAND_ECHO = 1247, //Sets port echo
        OEM6_COMMAND_ICOMCONFIG = 1248, //Configures IP virtual COM port
        OEM6_COMMAND_NTRIPCONFIG = 1249, //Configures NTRIP
        OEM6_COMMAND_GENERATERTKCORRECTIONS = 1260, //Sends a preconfigured set of RTK corrections
        OEM6_COMMAND_RAIMMODE = 1285, //Configures RAIM mode
        OEM6_COMMAND_GENERATEDIFFCORRECTIONS = 1296, //Sends a preconfigured set of differential corrections
        OEM6_COMMAND_SOFTLOADFILE = 1302, //Loads a file to the OEM638
        OEM6_COMMAND_SETRTCMTXVERSION = 1322, //Sets the expected RTCM transmission standard input
        OEM6_COMMAND_ALIGNAUTOMATION = 1323, //Configures the ALIGN plug and play feature
        OEM6_COMMAND_TRACKSV = 1326, //Overrides the automatic satellite/channel assignment for all satellites with manual instructions
        OEM6_COMMAND_NTRIPSOURCETABLE = 1343, //Sets the NTRIPCASTER ENDPOINTS to be used for the SOURCETABLE log
        OEM6_COMMAND_GENERATEALIGNCORRECTIONS = 1349, //Configures the ALIGN Master and start sending out ALIGN RTCA corrections through the specified port
        OEM6_COMMAND_QZSSECUTOFF = 1350, //Sets the elevation cut-off angle for QZSS satellites
        OEM6_COMMAND_DOSCMD = 1355, //Issues DOS commands to the file system
        OEM6_COMMAND_SETBASERECEIVERTYPE = 1374, //Specifies the base receiver type to aid GLONASS ambiguity fixing in RTK
        OEM6_COMMAND_PROFILE = 1411, //Configures multiple profiles in the non-volatile memor
        OEM6_COMMAND_BASEANTENNAPCO = 1415, //Sets the PCO model of the base receiver
        OEM6_COMMAND_BASEANTENNAPCV = 1416, //Sets the PCV model of the base receiver
        OEM6_COMMAND_THISANTENNAPCO = 1417, //Sets the PCO model of this receiver
        OEM6_COMMAND_THISANTENNAPCV = 1418, //Sets the PCV model of this receiver
        OEM6_COMMAND_BASEANTENNATYPE = 1419, //Sets the antenna type of the base receiver
        OEM6_COMMAND_THISANTENNATYPE = 1420, //Sets the antenna type of this receiver
        OEM6_COMMAND_SETTROPOMODEL = 1434, //Sets the troposphere model used to correct ranges in the PSRPOS and PDPPOS solutions
        OEM6_COMMAND_SERIALPROTOCOL = 1440, //Sets the protocol used by a serial port
        OEM6_COMMAND_RTKSOURCETIMEOUT = 1445, //Sets duration after RTK corrections switched from one source to another
        OEM6_COMMAND_RTKMATCHEDTIMEOUT = 1447, //Sets duration after which matched RTK filter is reset
        OEM6_COMMAND_PSRDIFFSOURCETIMEOUT = 1449, //Sets duration after differential corrections switched from one source to another
        OEM6_COMMAND_PSRDIFFTIMEOUT = 1450, //Sets maximum age of differential data
        OEM6_COMMAND_LEDCONFIG = 1498, //Configures LED indicators on the ProPak6
        OEM6_COMMAND_DATADECODESIGNAL = 1532, //Enable/Disable navigation data decoding for GNSS signal
        OEM6_COMMAND_NMEAVERSION = 1574, //Sets the NMEA version for output
        OEM6_COMMAND_IPSERVICE = 1575, //Configures the availability of network ports and services
        OEM6_COMMAND_SETADMINPASSWORD = 1579, //Sets the administration password
        OEM6_COMMAND_SETFILECOPYMODE = 1581, //Configures the internal memory copy function
        OEM6_COMMAND_BLUETOOTHCONFIG = 1609, //Configures the Bluetooth parameters
        OEM6_COMMAND_WIFICLICONFIG = 1614, //Configure Wi-Fi client
        OEM6_COMMAND_WIFICLICONTROL = 1615, //Controls the Wi-Fi client
        OEM6_COMMAND_WIFICONFIG = 1617, //Configure the Wi-Fi radio power and operating mode
        OEM6_COMMAND_EVENTOUTCONTROL = 1636, //Controls Event-Out properties
        OEM6_COMMAND_EVENTINCONTROL = 1637, //Controls Event-In properties
        OEM6_COMMAND_DUALANTENNAPOWER = 1639, //Controls the power to the Low Noise Amplifier (LNA) of an active antenna connected to the Ant 2 connector on a ProPak6 receiver.
        OEM6_COMMAND_IOCONFIG = 1663, //Sets the behavior of multiplexed I/O pins
        OEM6_COMMAND_WIFIAPCONFIG = 1665, //Configure the Wi-Fi Access Point
        OEM6_COMMAND_LOGIN = 1671, //Start a secure ICOM connection to the receiver
        OEM6_COMMAND_LOGOUT = 1672, //End a secure ICOM session started using the LOGIN command
        OEM6_COMMAND_AIRPLANEMODE = 1674, //Enables or disables Airplane mode
        OEM6_COMMAND_SAVEETHERNETDATA = 1679, //Save the configuration data associated with an Ethernet interface
        OEM6_COMMAND_CELLULARCONFIG = 1683, //Configure the cellular parameters
        OEM6_COMMAND_SETPREFERREDNETIF = 1688, //Set the network interface for DNS and default gateway configuration
        OEM6_COMMAND_BLUETOOTHDISCOVERABILITY = 1690 //Controls Bluetooth discoverability
    };

    // Port identifiers (partial) OEM6 manual pg 26
    enum OEM6_PORT_IDENTIFIER
    {
        OEM6_PORT_NONE = 0,
        OEM6_PORT_COM1ALL = 1,
        OEM6_PORT_COM2ALL = 2,
        OEM6_PORT_COM3ALL = 3,
        OEM6_PORT_THISPORT_ALL = 6,
        OEM6_PORT_FILE_ALL = 7,
        OEM6_PORT_ALL_PORTS = 8,
        OEM6_PORT_XCOM1_ALL = 9,
        OEM6_PORT_XCOM2_ALL = 10,
        OEM6_PORT_USB1_ALL = 13,
        OEM6_PORT_USB2_ALL = 14,
        OEM6_PORT_USB3_ALL = 15,
        OEM6_PORT_AUX_ALL = 16,
        OEM6_PORT_XCOM3_ALL = 17,
        OEM6_PORT_COM4_ALL = 18,
        OEM6_PORT_ICOM1_ALL = 23,
        OEM6_PORT_ICOM2_ALL = 24,
        OEM6_PORT_ICOM3_ALL = 25,
        OEM6_PORT_NCOM1_ALL = 26,
        OEM6_PORT_NCOM2_ALL = 27,
        OEM6_PORT_NCOM3_ALL = 28,
        OEM6_PORT_COM1 = 32,
        OEM6_PORT_COM2 = 64,
        OEM6_PORT_COM3 = 96,
        OEM6_PORT_SPECIAL = 160,
        OEM6_PORT_THISPORT = 192,
        OEM6_PORT_FILE = 224
    };




    // From the OEM6 Family firmware reference manual pg 179
    enum OEM6_LOG_TRIGGERS
    {
        ONNEW = 0,	//Does not output current message but outputs when the message is updated (not necessarily changed)
        ONCHANGED = 1,	// Outputs the current message and then continues to output when the message is changed
        ONTIME = 2, // Output on a time interval
        ONNEXT = 3,	// Output only the next message
        ONCE = 4,	// Output only the current message
        ONMARK = 5	// Output when a pulse is detected on the mark 1 input, MK1I
    };

    // From the OEM6 communication manual pg 326
    enum OEM6_LOG
    {
        OEM6_LOG_LOGLIST = 5, //A list of system logs
        OEM6_LOG_GPSEPHEM = 7, //GPS ephemeris data
        OEM6_LOG_IONUTC = 8, //Ionospheric and UTC model information
        OEM6_LOG_CLOCKMODEL = 16, //Current clock model matrices
        OEM6_LOG_RAWGPSSUBFRAME = 25, //Raw subframe data
        OEM6_LOG_CLOCKSTEERING = 26, //Clock steering status
        OEM6_LOG_VERSION = 37, //Receiver hardware and software version numbers
        OEM6_LOG_RAWEPHEM = 41, //Raw ephemeris
        OEM6_LOG_BESTPOS = 42, //Best position data
        OEM6_LOG_RANGE = 43, //Satellite range information
        OEM6_LOG_PSRPOS = 47, //Pseudorange position information
        OEM6_LOG_SATVIS = 48, //Satellite visibility
        OEM6_LOG_PORTSTATS = 72, //COM or USB port statistics
        OEM6_LOG_ALMANAC = 73, //Current almanac information
        OEM6_LOG_RAWALM = 74, //Raw almanac
        OEM6_LOG_TRACKSTAT = 83, //Satellite tracking status
        OEM6_LOG_RXSTATUS = 93, //Self-test status
        OEM6_LOG_RXSTATUSEVENT = 94, //Status event indicator
        OEM6_LOG_MATCHEDPOS = 96, //RTK Computed Position – Time Matched
        OEM6_LOG_BESTVEL = 99, //Velocity data
        OEM6_LOG_PSRVEL = 100, //Pseudorange velocity information
        OEM6_LOG_TIME = 101, //Receiver time information
        OEM6_LOG_RXCONFIG = 128, //Receiver configuration status
        OEM6_LOG_RANGECMP = 140, //Compressed version of the RANGE log
        OEM6_LOG_RTKPOS = 141, //RTK low latency position data
        OEM6_LOG_DIRENT = 159, //Onboard memory file list
        OEM6_LOG_NAVIGATE = 161, //Navigation waypoint status
        OEM6_LOG_AVEPOS = 172, //Position averaging
        OEM6_LOG_PSRDOP = 174, //DOP of SVs currently tracking
        OEM6_LOG_REFSTATION = 175, //Base station position and health
        OEM6_LOG_MARKPOS = 181, //Position at time of mark1 input event
        OEM6_LOG_VALIDMODELS = 206, //Model and expiry date information for receiver
        OEM6_LOG_RTKVEL = 216, //RTK velocity
        OEM6_LOG_MARKTIME = 231, //Time of mark1 input event
        OEM6_LOG_PASSCOM1 = 233, //Pass-through logs
        OEM6_LOG_PASSCOM2 = 234, //Pass-through logs
        OEM6_LOG_PASSCOM3 = 235, //Pass-through logs
        OEM6_LOG_BESTXYZ = 241, //Cartesian coordinate position data
        OEM6_LOG_MATCHEDXYZ = 242, //RTK Time Matched cartesian coordinate position data
        OEM6_LOG_PSRXYZ = 243, //Pseudorange cartesian coordinate position information
        OEM6_LOG_RTKXYZ = 244, //RTK cartesian coordinate position data
        OEM6_LOG_RAWSBASFRAME = 287,//Raw SBAS frame data
        OEM6_LOG_PASSXCOM1 = 405, //Pass-through logs
        OEM6_LOG_PASSXCOM2 = 406, //Pass-through logs
        OEM6_LOG_RAWGPSWORD = 407, //Raw navigation word
        OEM6_LOG_PDPPOS = 469, //PDP filter position
        OEM6_LOG_PDPVEL = 470, //PDP filter velocity
        OEM6_LOG_PDPXYZ = 471, //PDP filter Cartesian position and velocity
        OEM6_LOG_TIMESYNC = 492, //Synchronize time between receivers
        OEM6_LOG_OMNIHPPOS = 495, //OmniSTAR HP/XP position data
        OEM6_LOG_APPLICATIONSTATUS = 520, //Provides application status information
        OEM6_LOG_PASSUSB1 = 607, //Pass-through logs (for receivers that support USB)
        OEM6_LOG_PASSUSB2 = 608, //Pass-through logs (for receivers that support USB)
        OEM6_LOG_PASSUSB3 = 609, //Pass-through logs (for receivers that support USB)
        OEM6_LOG_MARK2POS = 615, //Time of mark input2 event
        OEM6_LOG_MARK2TIME = 616, //Position at time of mark2 input event
        OEM6_LOG_RANGEGPSL1 = 631, //L1 version of the RANGE log
        OEM6_LOG_BSLNXYZ = 686, //RTK XYZ baseline
        OEM6_LOG_PASSAUX = 690, //Pass-through log for AUX port
        OEM6_LOG_GLOALMANAC = 718, //GLONASS almanac data
        OEM6_LOG_GLOCLOCK = 719, //GLONASS clock information
        OEM6_LOG_GLORAWALM = 720, //Raw GLONASS almanac data
        OEM6_LOG_GLORAWFRAME = 721, //Raw GLONASS frame data
        OEM6_LOG_GLORAWSTRING = 722, //Raw GLONASS string data
        OEM6_LOG_GLOEPHEMERIS = 723, //GLONASS ephemeris data
        OEM6_LOG_BESTUTM = 726, //Best available UTM data
        OEM6_LOG_LBANDINFO = 730, //L-Band configuration information
        OEM6_LOG_LBANDSTAT = 731, //L-Band status information
        OEM6_LOG_RAWLBANDFRAME = 732, //Raw L-Band frame data
        OEM6_LOG_RAWLBANDPACKET = 733, //Raw L-Band data packet
        OEM6_LOG_GLORAWEPHEM = 792, //Raw GLONASS ephemeris data
        OEM6_LOG_PASSXCOM3 = 795, //Pass through log
        OEM6_LOG_OMNIVIS = 860, //OmniSTAR satellite visibility list
        OEM6_LOG_PSRTIME = 881, //Time offsets from the pseudorange filter
        OEM6_LOG_RTKDOP = 952, //Values from the RTK fast filter
        OEM6_LOG_HWMONITOR = 963, //Monitor Hardware Levels
        OEM6_LOG_HEADING = 971, //Heading information with the ALIGN feature
        OEM6_LOG_RAWSBASFRAME_2 = 973, //Raw SBAS frame data
        OEM6_LOG_SBAS0 = 976, //Remove PRN from the solution
        OEM6_LOG_SBAS1 = 977, //PRN mask assignments
        OEM6_LOG_SBAS10 = 978, //Degradation factor
        OEM6_LOG_SBAS12 = 979, //SBAS network time and UTC
        OEM6_LOG_SBAS17 = 980, //GEO almanac message
        OEM6_LOG_SBAS18 = 981, //IGP mask
        OEM6_LOG_SBAS2 = 982, //Fast correction slots 0-12
        OEM6_LOG_SBAS24 = 983, //Mixed fast/slow corrections
        OEM6_LOG_SBAS25 = 984, //Long term slow satellite corrections
        OEM6_LOG_SBAS26 = 985, //Ionospheric delay corrections
        OEM6_LOG_SBAS27 = 986, //SBAS service message
        OEM6_LOG_SBAS3 = 987, //Fast correction slots 13-25
        OEM6_LOG_SBAS32 = 988, //CDGPS Fast Corrections slots 0-10
        OEM6_LOG_SBAS33 = 989, //CDGPS Fast Corrections slots 11-21
        OEM6_LOG_SBAS34 = 990, //CDGPS Fast Corrections slots 22-32
        OEM6_LOG_SBAS35 = 991, //CDGPS Fast Corrections slots 32-43
        OEM6_LOG_SBAS4 = 992, //Fast correction slots 26-38
        OEM6_LOG_SBAS45 = 993, //CDGPS Slow Corrections
        OEM6_LOG_SBAS5 = 994, //Fast corrections slots 39-50
        OEM6_LOG_SBAS6 = 995, //Integrity Message
        OEM6_LOG_SBAS7 = 996, //Fast Correction Degradation
        OEM6_LOG_SBAS9 = 997, //Geo Nav Message
        OEM6_LOG_SBASCORR = 998, //SBAS range corrections used
        OEM6_LOG_SATVIS2 = 1043, //Satellite visibility
        OEM6_LOG_MASTERPOS = 1051, //Displays the master position with the ALIGN feature
        OEM6_LOG_ROVERPOS = 1052, //Displays the rover position with the ALIGN feature
        OEM6_LOG_RAWCNAVFRAME = 1066, //Raw L2C frame data
        OEM6_LOG_MARK3TIME = 1075, //Position at time of mark3 input event
        OEM6_LOG_MARK4TIME = 1076, //Position at time of mark4 input event
        OEM6_LOG_GALALMANAC = 1120, //Decoded Galileo almanac parameters from Galileonavigation messages
        OEM6_LOG_GALCLOCK = 1121, //Galileo time information
        OEM6_LOG_GALEPHEMERIS = 1122, //Galileo ephemeris information is available through the GALEPHEMERIS log
        OEM6_LOG_GALIONO = 1127, //Decoded Galileo ionospheric corrections
        OEM6_LOG_LOGFILESTATUS = 1146, //Current state of file and recording
        OEM6_LOG_CHANCONFIGLIST = 1148, //Channel configuration list
        OEM6_LOG_PSRSATS = 1162, //Satellites used in PSRPOS solution
        OEM6_LOG_PSRDOP2 = 1163, //Pseudorange least squares DOP
        OEM6_LOG_RTKDOP2 = 1172, //Values from the RTK Fast Filter
        OEM6_LOG_RTKSATS = 1174, //Satellites used in RTKPOS solution
        OEM6_LOG_MATCHEDSATS = 1176, //Lists the used and unused satellites for the corresponding MATCHEDPOS solution
        OEM6_LOG_BESTSATS = 1194, //Satellites used in BESTPOS
        OEM6_LOG_OMNIHPSATS = 1197, //Satellites used in the OMNIHPPOS solution
        OEM6_LOG_PASSETH1 = 1209, //Pass through log
        OEM6_LOG_PDPSATS = 1234, //Satellites used in PDPPOS solution
        OEM6_LOG_SOFTLOADSTATUS = 1235, //Status of the soft load process
        OEM6_LOG_PASSICOM1 = 1250, //Pass through log
        OEM6_LOG_PASSICOM2 = 1251, //Pass through log
        OEM6_LOG_PASSICOM3 = 1252, //Pass through log
        OEM6_LOG_PASSNCOM1 = 1253, //Pass through log
        OEM6_LOG_PASSNCOM2 = 1254, //Pass through log
        OEM6_LOG_PASSNCOM3 = 1255, //Pass through log
        OEM6_LOG_RANGECMP2 = 1273, //RANGE data compressed to handle more channels and types
        OEM6_LOG_RAIMSTATUS = 1286, //RAIM status
        OEM6_LOG_ETHSTATUS = 1288, //Current Ethernet status
        OEM6_LOG_IPSTATUS = 1289, //Current network configuration status
        OEM6_LOG_ALIGNBSLNXYZ = 1314, //Outputs the RTK quality XYZ baselines from ALIGN
        OEM6_LOG_ALIGNBSLNENU = 1315, //Outputs the RTK quality ENU baselines from ALIGN
        OEM6_LOG_HEADINGSATS = 1316, //Outputs the satellite information from ALIGN filter
        OEM6_LOG_REFSTATIONINFO = 1325, //Reference station position and health information
        OEM6_LOG_QZSSRAWEPHEM = 1330, //Contains the raw binary information for subframes one, two and three from the satellite with the parity information removed
        OEM6_LOG_QZSSRAWSUBFRAME = 1331, //A raw QZSS subframe is 300 bits in total, includes the parity bits which are interspersed with the raw data ten times, in six bit chunks, for a total of 60 parity bits
        OEM6_LOG_ALIGNDOP = 1332, //Outputs the DOP computed using the satellites used in solution
        OEM6_LOG_HEADING2 = 1335, //Outputs same information as HEADING log with an additional Rover ID field
        OEM6_LOG_QZSSEPHEMERIS = 1336, //Single set of QZSS ephemeris parameters
        OEM6_LOG_RTCAOBS3 = 1340, //Proprietary message that carries dual-frequency GPS and GLO measurements and is used in ALIGN. Also carries SBAS measurements if the Master receiver is single-frequency (L1-only) receiver to enable SBAS-ALIGN at the L1-only ALIGN Rover
        OEM6_LOG_PASSTHROUGH = 1342, //Outputs pass-through data from all receiver ports
        OEM6_LOG_SOURCETABLE = 1344, //Outputs the NTRIP source table entries from the NTRIPCASTER set by the NTRIPSOURCETABLE command
        OEM6_LOG_QZSSRAWALMANAC = 1345, //Contains the undecoded almanac subframes as received from the QZSS satellite
        OEM6_LOG_QZSSALMANAC = 1346, //Contains the decoded almanac parameters as received from the satellite with the parity information removed and appropriate scaling applied
        OEM6_LOG_QZSSIONUTC = 1347, //Ionospheric Model parameters (ION) and the Universal Time Coordinated parameters (UTC) for QZSS are provided
        OEM6_LOG_AUTHCODES = 1348, //Contains all authorization codes (auth codes) entered into the system since the last complete firmware reload
        OEM6_LOG_PASSCOM4 = 1384, //Pass through log
        OEM6_LOG_PROFILEINFO = 1412, //Outputs a list of Profiles
        OEM6_LOG_GALFNAVRAWPAGE = 1413, //Contains the raw Galileo F/Nav page data
        OEM6_LOG_GALINAVRAWWORD = 1414, //Contains the raw Galileo I/Nav word data
        OEM6_LOG_SBASALMANAC = 1425, //A collection of all current SBAS almanacs decoded by the receiver
        OEM6_LOG_SATXYZ2 = 1451, //Combined with a RANGE log, this data set contains the decoded satellite information necessary to compute the solution
        OEM6_LOG_PASSCOM5 = 1576, //Pass through log
        OEM6_LOG_PASSCOM6 = 1577, //Pass through log
        OEM6_LOG_BLUETOOTHSTATUS = 1608, //Bluetooth radio module status
        OEM6_LOG_WIFICLISTATUS = 1613, //Wi-Fi client connection status
        OEM6_LOG_WIFICLISCANRESULTS = 1616, //Wi-Fi AP scan results
        OEM6_LOG_NOVATELXOBS = 1618, //NovAtel proprietary RTK correction
        OEM6_LOG_NOVATELXREF = 1620, //NovAtel proprietary reference station message for use in ALIGN
        OEM6_LOG_WIFIAPSTATUS = 1666, //Wi-Fi Access Point Status
        OEM6_LOG_IPSTATS = 1669, //IP statistics
        OEM6_LOG_CELLULARSTATUS = 1685, //Cellular modem and network status information
        OEM6_LOG_CELLULARINFO = 1686 //Cellular modem and network information
    };

private:
    /**Valid values for the high rate logging are 0.05, 0.1, 0.2, 0.25 and 0.5. For logging
    slower than 1Hz any integer value is accepted

    OEM6 family firmware reference manual pg 179
    **/
    static const double OEM6_LOG_20_HZ;
    static const double OEM6_LOG_10_HZ;
    static const double OEM6_LOG_5_HZ;
    static const double OEM6_LOG_4_HZ;
    static const double OEM6_LOG_2_HZ;

    static const uint8_t HEADER_SYNC_BYTES[];
    static const uint8_t HEADER_SYNC_BYTES_LENGTH;
    static const uint8_t HEADER_LENGTH_BYTES;

    /**
     * Initialize the serial port that NovAtel is configured to run on.
     * @return true if serial was set up correctly, false otherwise
     */
    bool initPort();

    /**
     * Requests all desired logs from the GPS unit.
     */
    void setupLogging();

    /**
     * Send the 'unlog' command to the GPS unit. This command should remove all of the
     * recurring logs that were started in setupLogging.
     */
    void send_unlog_command();

    /// generate a message header for the novatel
    static std::vector<uint8_t> generate_header(uint16_t message_id, uint16_t message_length);

    /// compute the checksum for a message
    static std::vector<uint8_t> compute_checksum(const std::vector<uint8_t>& message);

    /// parse the header and append the relevant field to the log
    void parse_header(const std::vector<uint8_t>& header, std::vector<double>& log);

    /// parse the message and append the relevant fields to the log
    void parse_log(const std::vector<uint8_t>& data, std::vector<double>& log);

    /// Parses rtkxyz commands to display them
    void parse_rtkxyz(const std::vector<uint8_t>& data);


    ///Used by compute_checksum function
    static unsigned long CRC32Value(int i);

    static const uint32_t CRC32_POLYNOMIAL = 0xEDB88320L;

    /// serial port file descriptor
    int fd_ser;

    /// extract an enum field from the novatel message
    uint parse_enum(const std::vector<uint8_t>& log, int offset = 0);
    /// extract a 3 vector of floating point type (double of float) from the novatel message
    template<typename FloatingType>
    static blas::vector<FloatingType> parse_3floats(const std::vector<uint8_t>& log, int offset = 0);

    /// rotate vectors in ecef into ned frame using the llh position parameter
    template<typename FloatingType>
    static blas::vector<FloatingType> ecef_to_ned(const blas::vector<FloatingType>& ecef, const blas::vector<double>& llh);

    /// convert ecef position measurement into llh @note llh is in radians (easier for trig computations - must be converted to degrees for gx3)
    static blas::vector<double> ecef_to_llh(const blas::vector<double>& ecef);

    /// check whether the header is from a response message
    static bool is_response(const std::vector<uint8_t>& header);

    /// stores the last time data was successfully received (for error handling)
    long last_data;

    /**
     * Creates and sends a generic log signal to the NovAtel
     *
     * @param port - the port to send this message to
     * @param message - the message to send
     * @param trigger - what is going to signal the message to send
     * @param period - how often the message will send, use any integer >= 1 or one of the OEM6_LOG_XX_HZ constants
     */
    void _genericLog(OEM6_PORT_IDENTIFIER port, OEM6_LOG message, OEM6_LOG_TRIGGERS trigger, double period);

    /**
     * Tells the NovAtel to stop sending the given log.
     */
    void _genericUnlog(OEM6_LOG message);

    /**
     * Reads a packet from the NovAtel. Returns false if the system was terminated
     * before synch could happen.
     */
    bool synchronize();

    /// convert a raw string of bytes to a signed integer
    template <typename ReturnType, typename IteratorType>
    static ReturnType raw_to_int(IteratorType first, IteratorType last);

    /**
     * @code
     * uint16_t message_id = raw_to_int<uint16_t>(header.begin() + 1);
     * @endcode
     */
    template <typename ReturnType, typename IteratorType>
    static inline ReturnType raw_to_int(IteratorType first)
    {
        return raw_to_int<ReturnType>(first, first + sizeof(ReturnType));
    }

    template <typename FloatingType, typename IteratorType>
    static FloatingType raw_to_float(IteratorType first, IteratorType last);

    /// convert an integer type (signed or unsigned) to raw
    template <typename IntegerType>
    static std::vector<uint8_t> int_to_raw(const IntegerType i);

    /// convert a floating point type to raw
    template <typename FloatingType>
    static std::vector<uint8_t> float_to_raw(const FloatingType f);

};
template<typename FloatingType>
blas::vector<FloatingType> GPS::ReadSerial::parse_3floats(const std::vector<uint8_t>& log, int offset)
{
    blas::vector<FloatingType> floats(3);
    for (int i=0; i<3; i++)
        floats[i] = raw_to_float<FloatingType>(log.begin() + offset + sizeof(FloatingType)*i,
                                               log.begin() + offset + sizeof(FloatingType)*(i+1));
    return floats;
}

template<typename FloatingType>
blas::vector<FloatingType> GPS::ReadSerial::ecef_to_ned(const blas::vector<FloatingType>& ecef, const blas::vector<double>& llh)
{

    blas::matrix<double> ned_rotation(3,3);
    ned_rotation(0,0) = -sin(llh[0])*cos(llh[1]);
    ned_rotation(0,1) = -sin(llh[0])*sin(llh[1]);
    ned_rotation(0,2) = cos(llh[0]);
    ned_rotation(1,0) = -sin(llh[1]);
    ned_rotation(1,1) = cos(llh[1]);
    ned_rotation(1,2) = 0;
    ned_rotation(2,0) = -cos(llh[0])*cos(llh[1]);
    ned_rotation(2,1) = -cos(llh[0])*sin(llh[1]);
    ned_rotation(2,2) = -sin(llh[0]);

    return prod(ned_rotation, ecef);
}



template <typename ReturnType, typename IteratorType>
ReturnType GPS::ReadSerial::raw_to_int(IteratorType first, IteratorType last)
{
    uint32_t data = 0; // int is largest int type from novatel
    for (IteratorType it = last - 1; it != first - 1; --it)
    {
        data <<= 8;
        data += *it;
    }
    return *reinterpret_cast<ReturnType*>(&data);

}

template <typename FloatingType, typename IteratorType>
FloatingType GPS::ReadSerial::raw_to_float(IteratorType first, IteratorType last)
{
    uint64_t result = 0;
    for (IteratorType it = last - 1; it != first - 1; --it)
    {
        result <<= 8;
        result += *it;
    }
    if (last - first == 8)
        return *reinterpret_cast<double*>(&result);
    else
        return *reinterpret_cast<float*>(&result);

}

template <typename IntegerType>
std::vector<uint8_t> GPS::ReadSerial::int_to_raw(const IntegerType i)
{
    std::vector<uint8_t> result(sizeof(IntegerType));
    const uint8_t* byte = reinterpret_cast<const uint8_t*>(&i);
    for (uint32_t i=0; i<sizeof(IntegerType); i++)
    {
        result[i] = byte[i];
    }
    return result;
}

template <typename FloatingType>
std::vector<uint8_t> GPS::ReadSerial::float_to_raw(const FloatingType f)

{
    std::vector<uint8_t> result(sizeof(FloatingType));
    const uint8_t* byte = reinterpret_cast<const uint8_t*>(&f);
    for (size_t i = 0; i< sizeof(FloatingType); i++)
        result[i] = byte[i];
    return result;
}


#endif
