// MESSAGE UALBERTA_POSITION PACKING

#define MAVLINK_MSG_ID_UALBERTA_POSITION 224

typedef struct __mavlink_ualberta_position_t
{
 float llh_pos[3]; ///< Postion in lat long height(Estimated)
 float ned_pos[3]; ///< Position in local tangent frame(Estimated)
 float ned_vel[3]; ///< Velocity in local tangent frame(Estimated)
 float ned_origin[3]; ///< Local tangent frame origin in LLH(Estimated)
 float reference_position[3]; ///< Reference Position for translation control (NED)
 float position_error_body[3]; ///< Position error in body frame
 float position_error_ned[3]; ///< Position error in navigation frame
 uint32_t time_boot_ms; ///< 
} mavlink_ualberta_position_t;

#define MAVLINK_MSG_ID_UALBERTA_POSITION_LEN 88
#define MAVLINK_MSG_ID_224_LEN 88

#define MAVLINK_MSG_ID_UALBERTA_POSITION_CRC 247
#define MAVLINK_MSG_ID_224_CRC 247

#define MAVLINK_MSG_UALBERTA_POSITION_FIELD_LLH_POS_LEN 3
#define MAVLINK_MSG_UALBERTA_POSITION_FIELD_NED_POS_LEN 3
#define MAVLINK_MSG_UALBERTA_POSITION_FIELD_NED_VEL_LEN 3
#define MAVLINK_MSG_UALBERTA_POSITION_FIELD_NED_ORIGIN_LEN 3
#define MAVLINK_MSG_UALBERTA_POSITION_FIELD_REFERENCE_POSITION_LEN 3
#define MAVLINK_MSG_UALBERTA_POSITION_FIELD_POSITION_ERROR_BODY_LEN 3
#define MAVLINK_MSG_UALBERTA_POSITION_FIELD_POSITION_ERROR_NED_LEN 3

#define MAVLINK_MESSAGE_INFO_UALBERTA_POSITION { \
	"UALBERTA_POSITION", \
	8, \
	{  { "llh_pos", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_ualberta_position_t, llh_pos) }, \
         { "ned_pos", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_ualberta_position_t, ned_pos) }, \
         { "ned_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_ualberta_position_t, ned_vel) }, \
         { "ned_origin", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_ualberta_position_t, ned_origin) }, \
         { "reference_position", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_ualberta_position_t, reference_position) }, \
         { "position_error_body", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_ualberta_position_t, position_error_body) }, \
         { "position_error_ned", NULL, MAVLINK_TYPE_FLOAT, 3, 72, offsetof(mavlink_ualberta_position_t, position_error_ned) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 84, offsetof(mavlink_ualberta_position_t, time_boot_ms) }, \
         } \
}


/**
 * @brief Pack a ualberta_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param llh_pos Postion in lat long height(Estimated)
 * @param ned_pos Position in local tangent frame(Estimated)
 * @param ned_vel Velocity in local tangent frame(Estimated)
 * @param ned_origin Local tangent frame origin in LLH(Estimated)
 * @param reference_position Reference Position for translation control (NED)
 * @param position_error_body Position error in body frame
 * @param position_error_ned Position error in navigation frame
 * @param time_boot_ms 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *llh_pos, const float *ned_pos, const float *ned_vel, const float *ned_origin, const float *reference_position, const float *position_error_body, const float *position_error_ned, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UALBERTA_POSITION_LEN];
	_mav_put_uint32_t(buf, 84, time_boot_ms);
	_mav_put_float_array(buf, 0, llh_pos, 3);
	_mav_put_float_array(buf, 12, ned_pos, 3);
	_mav_put_float_array(buf, 24, ned_vel, 3);
	_mav_put_float_array(buf, 36, ned_origin, 3);
	_mav_put_float_array(buf, 48, reference_position, 3);
	_mav_put_float_array(buf, 60, position_error_body, 3);
	_mav_put_float_array(buf, 72, position_error_ned, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#else
	mavlink_ualberta_position_t packet;
	packet.time_boot_ms = time_boot_ms;
	mav_array_memcpy(packet.llh_pos, llh_pos, sizeof(float)*3);
	mav_array_memcpy(packet.ned_pos, ned_pos, sizeof(float)*3);
	mav_array_memcpy(packet.ned_vel, ned_vel, sizeof(float)*3);
	mav_array_memcpy(packet.ned_origin, ned_origin, sizeof(float)*3);
	mav_array_memcpy(packet.reference_position, reference_position, sizeof(float)*3);
	mav_array_memcpy(packet.position_error_body, position_error_body, sizeof(float)*3);
	mav_array_memcpy(packet.position_error_ned, position_error_ned, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN, MAVLINK_MSG_ID_UALBERTA_POSITION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif
}

/**
 * @brief Pack a ualberta_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param llh_pos Postion in lat long height(Estimated)
 * @param ned_pos Position in local tangent frame(Estimated)
 * @param ned_vel Velocity in local tangent frame(Estimated)
 * @param ned_origin Local tangent frame origin in LLH(Estimated)
 * @param reference_position Reference Position for translation control (NED)
 * @param position_error_body Position error in body frame
 * @param position_error_ned Position error in navigation frame
 * @param time_boot_ms 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *llh_pos,const float *ned_pos,const float *ned_vel,const float *ned_origin,const float *reference_position,const float *position_error_body,const float *position_error_ned,uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UALBERTA_POSITION_LEN];
	_mav_put_uint32_t(buf, 84, time_boot_ms);
	_mav_put_float_array(buf, 0, llh_pos, 3);
	_mav_put_float_array(buf, 12, ned_pos, 3);
	_mav_put_float_array(buf, 24, ned_vel, 3);
	_mav_put_float_array(buf, 36, ned_origin, 3);
	_mav_put_float_array(buf, 48, reference_position, 3);
	_mav_put_float_array(buf, 60, position_error_body, 3);
	_mav_put_float_array(buf, 72, position_error_ned, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#else
	mavlink_ualberta_position_t packet;
	packet.time_boot_ms = time_boot_ms;
	mav_array_memcpy(packet.llh_pos, llh_pos, sizeof(float)*3);
	mav_array_memcpy(packet.ned_pos, ned_pos, sizeof(float)*3);
	mav_array_memcpy(packet.ned_vel, ned_vel, sizeof(float)*3);
	mav_array_memcpy(packet.ned_origin, ned_origin, sizeof(float)*3);
	mav_array_memcpy(packet.reference_position, reference_position, sizeof(float)*3);
	mav_array_memcpy(packet.position_error_body, position_error_body, sizeof(float)*3);
	mav_array_memcpy(packet.position_error_ned, position_error_ned, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN, MAVLINK_MSG_ID_UALBERTA_POSITION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif
}

/**
 * @brief Encode a ualberta_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_position_t* ualberta_position)
{
	return mavlink_msg_ualberta_position_pack(system_id, component_id, msg, ualberta_position->llh_pos, ualberta_position->ned_pos, ualberta_position->ned_vel, ualberta_position->ned_origin, ualberta_position->reference_position, ualberta_position->position_error_body, ualberta_position->position_error_ned, ualberta_position->time_boot_ms);
}

/**
 * @brief Encode a ualberta_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ualberta_position_t* ualberta_position)
{
	return mavlink_msg_ualberta_position_pack_chan(system_id, component_id, chan, msg, ualberta_position->llh_pos, ualberta_position->ned_pos, ualberta_position->ned_vel, ualberta_position->ned_origin, ualberta_position->reference_position, ualberta_position->position_error_body, ualberta_position->position_error_ned, ualberta_position->time_boot_ms);
}

/**
 * @brief Send a ualberta_position message
 * @param chan MAVLink channel to send the message
 *
 * @param llh_pos Postion in lat long height(Estimated)
 * @param ned_pos Position in local tangent frame(Estimated)
 * @param ned_vel Velocity in local tangent frame(Estimated)
 * @param ned_origin Local tangent frame origin in LLH(Estimated)
 * @param reference_position Reference Position for translation control (NED)
 * @param position_error_body Position error in body frame
 * @param position_error_ned Position error in navigation frame
 * @param time_boot_ms 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_position_send(mavlink_channel_t chan, const float *llh_pos, const float *ned_pos, const float *ned_vel, const float *ned_origin, const float *reference_position, const float *position_error_body, const float *position_error_ned, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UALBERTA_POSITION_LEN];
	_mav_put_uint32_t(buf, 84, time_boot_ms);
	_mav_put_float_array(buf, 0, llh_pos, 3);
	_mav_put_float_array(buf, 12, ned_pos, 3);
	_mav_put_float_array(buf, 24, ned_vel, 3);
	_mav_put_float_array(buf, 36, ned_origin, 3);
	_mav_put_float_array(buf, 48, reference_position, 3);
	_mav_put_float_array(buf, 60, position_error_body, 3);
	_mav_put_float_array(buf, 72, position_error_ned, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, buf, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN, MAVLINK_MSG_ID_UALBERTA_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, buf, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif
#else
	mavlink_ualberta_position_t packet;
	packet.time_boot_ms = time_boot_ms;
	mav_array_memcpy(packet.llh_pos, llh_pos, sizeof(float)*3);
	mav_array_memcpy(packet.ned_pos, ned_pos, sizeof(float)*3);
	mav_array_memcpy(packet.ned_vel, ned_vel, sizeof(float)*3);
	mav_array_memcpy(packet.ned_origin, ned_origin, sizeof(float)*3);
	mav_array_memcpy(packet.reference_position, reference_position, sizeof(float)*3);
	mav_array_memcpy(packet.position_error_body, position_error_body, sizeof(float)*3);
	mav_array_memcpy(packet.position_error_ned, position_error_ned, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, (const char *)&packet, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN, MAVLINK_MSG_ID_UALBERTA_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, (const char *)&packet, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_UALBERTA_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ualberta_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *llh_pos, const float *ned_pos, const float *ned_vel, const float *ned_origin, const float *reference_position, const float *position_error_body, const float *position_error_ned, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 84, time_boot_ms);
	_mav_put_float_array(buf, 0, llh_pos, 3);
	_mav_put_float_array(buf, 12, ned_pos, 3);
	_mav_put_float_array(buf, 24, ned_vel, 3);
	_mav_put_float_array(buf, 36, ned_origin, 3);
	_mav_put_float_array(buf, 48, reference_position, 3);
	_mav_put_float_array(buf, 60, position_error_body, 3);
	_mav_put_float_array(buf, 72, position_error_ned, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, buf, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN, MAVLINK_MSG_ID_UALBERTA_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, buf, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif
#else
	mavlink_ualberta_position_t *packet = (mavlink_ualberta_position_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	mav_array_memcpy(packet->llh_pos, llh_pos, sizeof(float)*3);
	mav_array_memcpy(packet->ned_pos, ned_pos, sizeof(float)*3);
	mav_array_memcpy(packet->ned_vel, ned_vel, sizeof(float)*3);
	mav_array_memcpy(packet->ned_origin, ned_origin, sizeof(float)*3);
	mav_array_memcpy(packet->reference_position, reference_position, sizeof(float)*3);
	mav_array_memcpy(packet->position_error_body, position_error_body, sizeof(float)*3);
	mav_array_memcpy(packet->position_error_ned, position_error_ned, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, (const char *)packet, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN, MAVLINK_MSG_ID_UALBERTA_POSITION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_POSITION, (const char *)packet, MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE UALBERTA_POSITION UNPACKING


/**
 * @brief Get field llh_pos from ualberta_position message
 *
 * @return Postion in lat long height(Estimated)
 */
static inline uint16_t mavlink_msg_ualberta_position_get_llh_pos(const mavlink_message_t* msg, float *llh_pos)
{
	return _MAV_RETURN_float_array(msg, llh_pos, 3,  0);
}

/**
 * @brief Get field ned_pos from ualberta_position message
 *
 * @return Position in local tangent frame(Estimated)
 */
static inline uint16_t mavlink_msg_ualberta_position_get_ned_pos(const mavlink_message_t* msg, float *ned_pos)
{
	return _MAV_RETURN_float_array(msg, ned_pos, 3,  12);
}

/**
 * @brief Get field ned_vel from ualberta_position message
 *
 * @return Velocity in local tangent frame(Estimated)
 */
static inline uint16_t mavlink_msg_ualberta_position_get_ned_vel(const mavlink_message_t* msg, float *ned_vel)
{
	return _MAV_RETURN_float_array(msg, ned_vel, 3,  24);
}

/**
 * @brief Get field ned_origin from ualberta_position message
 *
 * @return Local tangent frame origin in LLH(Estimated)
 */
static inline uint16_t mavlink_msg_ualberta_position_get_ned_origin(const mavlink_message_t* msg, float *ned_origin)
{
	return _MAV_RETURN_float_array(msg, ned_origin, 3,  36);
}

/**
 * @brief Get field reference_position from ualberta_position message
 *
 * @return Reference Position for translation control (NED)
 */
static inline uint16_t mavlink_msg_ualberta_position_get_reference_position(const mavlink_message_t* msg, float *reference_position)
{
	return _MAV_RETURN_float_array(msg, reference_position, 3,  48);
}

/**
 * @brief Get field position_error_body from ualberta_position message
 *
 * @return Position error in body frame
 */
static inline uint16_t mavlink_msg_ualberta_position_get_position_error_body(const mavlink_message_t* msg, float *position_error_body)
{
	return _MAV_RETURN_float_array(msg, position_error_body, 3,  60);
}

/**
 * @brief Get field position_error_ned from ualberta_position message
 *
 * @return Position error in navigation frame
 */
static inline uint16_t mavlink_msg_ualberta_position_get_position_error_ned(const mavlink_message_t* msg, float *position_error_ned)
{
	return _MAV_RETURN_float_array(msg, position_error_ned, 3,  72);
}

/**
 * @brief Get field time_boot_ms from ualberta_position message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_ualberta_position_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  84);
}

/**
 * @brief Decode a ualberta_position message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_position_decode(const mavlink_message_t* msg, mavlink_ualberta_position_t* ualberta_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_ualberta_position_get_llh_pos(msg, ualberta_position->llh_pos);
	mavlink_msg_ualberta_position_get_ned_pos(msg, ualberta_position->ned_pos);
	mavlink_msg_ualberta_position_get_ned_vel(msg, ualberta_position->ned_vel);
	mavlink_msg_ualberta_position_get_ned_origin(msg, ualberta_position->ned_origin);
	mavlink_msg_ualberta_position_get_reference_position(msg, ualberta_position->reference_position);
	mavlink_msg_ualberta_position_get_position_error_body(msg, ualberta_position->position_error_body);
	mavlink_msg_ualberta_position_get_position_error_ned(msg, ualberta_position->position_error_ned);
	ualberta_position->time_boot_ms = mavlink_msg_ualberta_position_get_time_boot_ms(msg);
#else
	memcpy(ualberta_position, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_UALBERTA_POSITION_LEN);
#endif
}
