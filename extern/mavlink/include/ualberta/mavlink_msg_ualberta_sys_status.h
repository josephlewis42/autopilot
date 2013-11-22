// MESSAGE UALBERTA_SYS_STATUS PACKING

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS 222

typedef struct __mavlink_ualberta_sys_status_t
{
 float collective; ///< 
 float receiver_voltage; ///< 
 float avionics_voltage; ///< 
 uint16_t engine_rpm; ///< 
 uint16_t rotor_rpm; ///< 
 uint8_t mode; ///< System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 uint8_t gx3_mode; ///< Navigation mode, see UALBERTA_GX3_MODE ENUM
 uint8_t pilot_mode; ///< Pilot mode, see UALBERTA_PILOT_MODE
 uint8_t control_mode; ///< Controller Mode
 uint8_t attitude_source; ///< 
 uint8_t trajectory; ///< 
} mavlink_ualberta_sys_status_t;

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN 22
#define MAVLINK_MSG_ID_222_LEN 22



#define MAVLINK_MESSAGE_INFO_UALBERTA_SYS_STATUS { \
	"UALBERTA_SYS_STATUS", \
	11, \
	{  { "collective", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ualberta_sys_status_t, collective) }, \
         { "receiver_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ualberta_sys_status_t, receiver_voltage) }, \
         { "avionics_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ualberta_sys_status_t, avionics_voltage) }, \
         { "engine_rpm", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_ualberta_sys_status_t, engine_rpm) }, \
         { "rotor_rpm", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_ualberta_sys_status_t, rotor_rpm) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ualberta_sys_status_t, mode) }, \
         { "gx3_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_ualberta_sys_status_t, gx3_mode) }, \
         { "pilot_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_ualberta_sys_status_t, pilot_mode) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_ualberta_sys_status_t, control_mode) }, \
         { "attitude_source", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_ualberta_sys_status_t, attitude_source) }, \
         { "trajectory", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_ualberta_sys_status_t, trajectory) }, \
         } \
}


/**
 * @brief Pack a ualberta_sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param gx3_mode Navigation mode, see UALBERTA_GX3_MODE ENUM
 * @param pilot_mode Pilot mode, see UALBERTA_PILOT_MODE
 * @param control_mode Controller Mode
 * @param attitude_source 
 * @param engine_rpm 
 * @param rotor_rpm 
 * @param collective 
 * @param receiver_voltage 
 * @param avionics_voltage 
 * @param trajectory 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t mode, uint8_t gx3_mode, uint8_t pilot_mode, uint8_t control_mode, uint8_t attitude_source, uint16_t engine_rpm, uint16_t rotor_rpm, float collective, float receiver_voltage, float avionics_voltage, uint8_t trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_float(buf, 0, collective);
	_mav_put_float(buf, 4, receiver_voltage);
	_mav_put_float(buf, 8, avionics_voltage);
	_mav_put_uint16_t(buf, 12, engine_rpm);
	_mav_put_uint16_t(buf, 14, rotor_rpm);
	_mav_put_uint8_t(buf, 16, mode);
	_mav_put_uint8_t(buf, 17, gx3_mode);
	_mav_put_uint8_t(buf, 18, pilot_mode);
	_mav_put_uint8_t(buf, 19, control_mode);
	_mav_put_uint8_t(buf, 20, attitude_source);
	_mav_put_uint8_t(buf, 21, trajectory);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 22);
#else
	mavlink_ualberta_sys_status_t packet;
	packet.collective = collective;
	packet.receiver_voltage = receiver_voltage;
	packet.avionics_voltage = avionics_voltage;
	packet.engine_rpm = engine_rpm;
	packet.rotor_rpm = rotor_rpm;
	packet.mode = mode;
	packet.gx3_mode = gx3_mode;
	packet.pilot_mode = pilot_mode;
	packet.control_mode = control_mode;
	packet.attitude_source = attitude_source;
	packet.trajectory = trajectory;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 22);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 22, 122);
}

/**
 * @brief Pack a ualberta_sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param gx3_mode Navigation mode, see UALBERTA_GX3_MODE ENUM
 * @param pilot_mode Pilot mode, see UALBERTA_PILOT_MODE
 * @param control_mode Controller Mode
 * @param attitude_source 
 * @param engine_rpm 
 * @param rotor_rpm 
 * @param collective 
 * @param receiver_voltage 
 * @param avionics_voltage 
 * @param trajectory 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t mode,uint8_t gx3_mode,uint8_t pilot_mode,uint8_t control_mode,uint8_t attitude_source,uint16_t engine_rpm,uint16_t rotor_rpm,float collective,float receiver_voltage,float avionics_voltage,uint8_t trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_float(buf, 0, collective);
	_mav_put_float(buf, 4, receiver_voltage);
	_mav_put_float(buf, 8, avionics_voltage);
	_mav_put_uint16_t(buf, 12, engine_rpm);
	_mav_put_uint16_t(buf, 14, rotor_rpm);
	_mav_put_uint8_t(buf, 16, mode);
	_mav_put_uint8_t(buf, 17, gx3_mode);
	_mav_put_uint8_t(buf, 18, pilot_mode);
	_mav_put_uint8_t(buf, 19, control_mode);
	_mav_put_uint8_t(buf, 20, attitude_source);
	_mav_put_uint8_t(buf, 21, trajectory);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 22);
#else
	mavlink_ualberta_sys_status_t packet;
	packet.collective = collective;
	packet.receiver_voltage = receiver_voltage;
	packet.avionics_voltage = avionics_voltage;
	packet.engine_rpm = engine_rpm;
	packet.rotor_rpm = rotor_rpm;
	packet.mode = mode;
	packet.gx3_mode = gx3_mode;
	packet.pilot_mode = pilot_mode;
	packet.control_mode = control_mode;
	packet.attitude_source = attitude_source;
	packet.trajectory = trajectory;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 22);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 22, 122);
}

/**
 * @brief Encode a ualberta_sys_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
	return mavlink_msg_ualberta_sys_status_pack(system_id, component_id, msg, ualberta_sys_status->mode, ualberta_sys_status->gx3_mode, ualberta_sys_status->pilot_mode, ualberta_sys_status->control_mode, ualberta_sys_status->attitude_source, ualberta_sys_status->engine_rpm, ualberta_sys_status->rotor_rpm, ualberta_sys_status->collective, ualberta_sys_status->receiver_voltage, ualberta_sys_status->avionics_voltage, ualberta_sys_status->trajectory);
}

/**
 * @brief Send a ualberta_sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param gx3_mode Navigation mode, see UALBERTA_GX3_MODE ENUM
 * @param pilot_mode Pilot mode, see UALBERTA_PILOT_MODE
 * @param control_mode Controller Mode
 * @param attitude_source 
 * @param engine_rpm 
 * @param rotor_rpm 
 * @param collective 
 * @param receiver_voltage 
 * @param avionics_voltage 
 * @param trajectory 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_sys_status_send(mavlink_channel_t chan, uint8_t mode, uint8_t gx3_mode, uint8_t pilot_mode, uint8_t control_mode, uint8_t attitude_source, uint16_t engine_rpm, uint16_t rotor_rpm, float collective, float receiver_voltage, float avionics_voltage, uint8_t trajectory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_float(buf, 0, collective);
	_mav_put_float(buf, 4, receiver_voltage);
	_mav_put_float(buf, 8, avionics_voltage);
	_mav_put_uint16_t(buf, 12, engine_rpm);
	_mav_put_uint16_t(buf, 14, rotor_rpm);
	_mav_put_uint8_t(buf, 16, mode);
	_mav_put_uint8_t(buf, 17, gx3_mode);
	_mav_put_uint8_t(buf, 18, pilot_mode);
	_mav_put_uint8_t(buf, 19, control_mode);
	_mav_put_uint8_t(buf, 20, attitude_source);
	_mav_put_uint8_t(buf, 21, trajectory);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, buf, 22, 122);
#else
	mavlink_ualberta_sys_status_t packet;
	packet.collective = collective;
	packet.receiver_voltage = receiver_voltage;
	packet.avionics_voltage = avionics_voltage;
	packet.engine_rpm = engine_rpm;
	packet.rotor_rpm = rotor_rpm;
	packet.mode = mode;
	packet.gx3_mode = gx3_mode;
	packet.pilot_mode = pilot_mode;
	packet.control_mode = control_mode;
	packet.attitude_source = attitude_source;
	packet.trajectory = trajectory;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, (const char *)&packet, 22, 122);
#endif
}

#endif

// MESSAGE UALBERTA_SYS_STATUS UNPACKING


/**
 * @brief Get field mode from ualberta_sys_status message
 *
 * @return System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field gx3_mode from ualberta_sys_status message
 *
 * @return Navigation mode, see UALBERTA_GX3_MODE ENUM
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_gx3_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field pilot_mode from ualberta_sys_status message
 *
 * @return Pilot mode, see UALBERTA_PILOT_MODE
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_pilot_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field control_mode from ualberta_sys_status message
 *
 * @return Controller Mode
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_control_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field attitude_source from ualberta_sys_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_attitude_source(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field engine_rpm from ualberta_sys_status message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_get_engine_rpm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field rotor_rpm from ualberta_sys_status message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_get_rotor_rpm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field collective from ualberta_sys_status message
 *
 * @return 
 */
static inline float mavlink_msg_ualberta_sys_status_get_collective(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field receiver_voltage from ualberta_sys_status message
 *
 * @return 
 */
static inline float mavlink_msg_ualberta_sys_status_get_receiver_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field avionics_voltage from ualberta_sys_status message
 *
 * @return 
 */
static inline float mavlink_msg_ualberta_sys_status_get_avionics_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field trajectory from ualberta_sys_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_trajectory(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a ualberta_sys_status message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_sys_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_sys_status_decode(const mavlink_message_t* msg, mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	ualberta_sys_status->collective = mavlink_msg_ualberta_sys_status_get_collective(msg);
	ualberta_sys_status->receiver_voltage = mavlink_msg_ualberta_sys_status_get_receiver_voltage(msg);
	ualberta_sys_status->avionics_voltage = mavlink_msg_ualberta_sys_status_get_avionics_voltage(msg);
	ualberta_sys_status->engine_rpm = mavlink_msg_ualberta_sys_status_get_engine_rpm(msg);
	ualberta_sys_status->rotor_rpm = mavlink_msg_ualberta_sys_status_get_rotor_rpm(msg);
	ualberta_sys_status->mode = mavlink_msg_ualberta_sys_status_get_mode(msg);
	ualberta_sys_status->gx3_mode = mavlink_msg_ualberta_sys_status_get_gx3_mode(msg);
	ualberta_sys_status->pilot_mode = mavlink_msg_ualberta_sys_status_get_pilot_mode(msg);
	ualberta_sys_status->control_mode = mavlink_msg_ualberta_sys_status_get_control_mode(msg);
	ualberta_sys_status->attitude_source = mavlink_msg_ualberta_sys_status_get_attitude_source(msg);
	ualberta_sys_status->trajectory = mavlink_msg_ualberta_sys_status_get_trajectory(msg);
#else
	memcpy(ualberta_sys_status, _MAV_PAYLOAD(msg), 22);
#endif
}
