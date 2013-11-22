// MESSAGE UALBERTA_ATTITUDE PACKING

#define MAVLINK_MSG_ID_UALBERTA_ATTITUDE 227

typedef struct __mavlink_ualberta_attitude_t
{
 float nav_euler[3]; ///< 
 float nav_euler_rate[3]; ///< 
 float ahrs_euler[3]; ///< 
 float ahrs_euler_rate[3]; ///< 
 float attitude_reference[2]; ///< 
 uint32_t time_boot_ms; ///< 
} mavlink_ualberta_attitude_t;

#define MAVLINK_MSG_ID_UALBERTA_ATTITUDE_LEN 60
#define MAVLINK_MSG_ID_227_LEN 60

#define MAVLINK_MSG_UALBERTA_ATTITUDE_FIELD_NAV_EULER_LEN 3
#define MAVLINK_MSG_UALBERTA_ATTITUDE_FIELD_NAV_EULER_RATE_LEN 3
#define MAVLINK_MSG_UALBERTA_ATTITUDE_FIELD_AHRS_EULER_LEN 3
#define MAVLINK_MSG_UALBERTA_ATTITUDE_FIELD_AHRS_EULER_RATE_LEN 3
#define MAVLINK_MSG_UALBERTA_ATTITUDE_FIELD_ATTITUDE_REFERENCE_LEN 2

#define MAVLINK_MESSAGE_INFO_UALBERTA_ATTITUDE { \
	"UALBERTA_ATTITUDE", \
	6, \
	{  { "nav_euler", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_ualberta_attitude_t, nav_euler) }, \
         { "nav_euler_rate", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_ualberta_attitude_t, nav_euler_rate) }, \
         { "ahrs_euler", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_ualberta_attitude_t, ahrs_euler) }, \
         { "ahrs_euler_rate", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_ualberta_attitude_t, ahrs_euler_rate) }, \
         { "attitude_reference", NULL, MAVLINK_TYPE_FLOAT, 2, 48, offsetof(mavlink_ualberta_attitude_t, attitude_reference) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 56, offsetof(mavlink_ualberta_attitude_t, time_boot_ms) }, \
         } \
}


/**
 * @brief Pack a ualberta_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nav_euler 
 * @param nav_euler_rate 
 * @param ahrs_euler 
 * @param ahrs_euler_rate 
 * @param attitude_reference 
 * @param time_boot_ms 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *nav_euler, const float *nav_euler_rate, const float *ahrs_euler, const float *ahrs_euler_rate, const float *attitude_reference, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[60];
	_mav_put_uint32_t(buf, 56, time_boot_ms);
	_mav_put_float_array(buf, 0, nav_euler, 3);
	_mav_put_float_array(buf, 12, nav_euler_rate, 3);
	_mav_put_float_array(buf, 24, ahrs_euler, 3);
	_mav_put_float_array(buf, 36, ahrs_euler_rate, 3);
	_mav_put_float_array(buf, 48, attitude_reference, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 60);
#else
	mavlink_ualberta_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	mav_array_memcpy(packet.nav_euler, nav_euler, sizeof(float)*3);
	mav_array_memcpy(packet.nav_euler_rate, nav_euler_rate, sizeof(float)*3);
	mav_array_memcpy(packet.ahrs_euler, ahrs_euler, sizeof(float)*3);
	mav_array_memcpy(packet.ahrs_euler_rate, ahrs_euler_rate, sizeof(float)*3);
	mav_array_memcpy(packet.attitude_reference, attitude_reference, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 60);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_ATTITUDE;
	return mavlink_finalize_message(msg, system_id, component_id, 60, 134);
}

/**
 * @brief Pack a ualberta_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_euler 
 * @param nav_euler_rate 
 * @param ahrs_euler 
 * @param ahrs_euler_rate 
 * @param attitude_reference 
 * @param time_boot_ms 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *nav_euler,const float *nav_euler_rate,const float *ahrs_euler,const float *ahrs_euler_rate,const float *attitude_reference,uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[60];
	_mav_put_uint32_t(buf, 56, time_boot_ms);
	_mav_put_float_array(buf, 0, nav_euler, 3);
	_mav_put_float_array(buf, 12, nav_euler_rate, 3);
	_mav_put_float_array(buf, 24, ahrs_euler, 3);
	_mav_put_float_array(buf, 36, ahrs_euler_rate, 3);
	_mav_put_float_array(buf, 48, attitude_reference, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 60);
#else
	mavlink_ualberta_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	mav_array_memcpy(packet.nav_euler, nav_euler, sizeof(float)*3);
	mav_array_memcpy(packet.nav_euler_rate, nav_euler_rate, sizeof(float)*3);
	mav_array_memcpy(packet.ahrs_euler, ahrs_euler, sizeof(float)*3);
	mav_array_memcpy(packet.ahrs_euler_rate, ahrs_euler_rate, sizeof(float)*3);
	mav_array_memcpy(packet.attitude_reference, attitude_reference, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 60);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_ATTITUDE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 60, 134);
}

/**
 * @brief Encode a ualberta_attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_attitude_t* ualberta_attitude)
{
	return mavlink_msg_ualberta_attitude_pack(system_id, component_id, msg, ualberta_attitude->nav_euler, ualberta_attitude->nav_euler_rate, ualberta_attitude->ahrs_euler, ualberta_attitude->ahrs_euler_rate, ualberta_attitude->attitude_reference, ualberta_attitude->time_boot_ms);
}

/**
 * @brief Send a ualberta_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param nav_euler 
 * @param nav_euler_rate 
 * @param ahrs_euler 
 * @param ahrs_euler_rate 
 * @param attitude_reference 
 * @param time_boot_ms 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_attitude_send(mavlink_channel_t chan, const float *nav_euler, const float *nav_euler_rate, const float *ahrs_euler, const float *ahrs_euler_rate, const float *attitude_reference, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[60];
	_mav_put_uint32_t(buf, 56, time_boot_ms);
	_mav_put_float_array(buf, 0, nav_euler, 3);
	_mav_put_float_array(buf, 12, nav_euler_rate, 3);
	_mav_put_float_array(buf, 24, ahrs_euler, 3);
	_mav_put_float_array(buf, 36, ahrs_euler_rate, 3);
	_mav_put_float_array(buf, 48, attitude_reference, 2);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ATTITUDE, buf, 60, 134);
#else
	mavlink_ualberta_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	mav_array_memcpy(packet.nav_euler, nav_euler, sizeof(float)*3);
	mav_array_memcpy(packet.nav_euler_rate, nav_euler_rate, sizeof(float)*3);
	mav_array_memcpy(packet.ahrs_euler, ahrs_euler, sizeof(float)*3);
	mav_array_memcpy(packet.ahrs_euler_rate, ahrs_euler_rate, sizeof(float)*3);
	mav_array_memcpy(packet.attitude_reference, attitude_reference, sizeof(float)*2);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ATTITUDE, (const char *)&packet, 60, 134);
#endif
}

#endif

// MESSAGE UALBERTA_ATTITUDE UNPACKING


/**
 * @brief Get field nav_euler from ualberta_attitude message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_attitude_get_nav_euler(const mavlink_message_t* msg, float *nav_euler)
{
	return _MAV_RETURN_float_array(msg, nav_euler, 3,  0);
}

/**
 * @brief Get field nav_euler_rate from ualberta_attitude message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_attitude_get_nav_euler_rate(const mavlink_message_t* msg, float *nav_euler_rate)
{
	return _MAV_RETURN_float_array(msg, nav_euler_rate, 3,  12);
}

/**
 * @brief Get field ahrs_euler from ualberta_attitude message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_attitude_get_ahrs_euler(const mavlink_message_t* msg, float *ahrs_euler)
{
	return _MAV_RETURN_float_array(msg, ahrs_euler, 3,  24);
}

/**
 * @brief Get field ahrs_euler_rate from ualberta_attitude message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_attitude_get_ahrs_euler_rate(const mavlink_message_t* msg, float *ahrs_euler_rate)
{
	return _MAV_RETURN_float_array(msg, ahrs_euler_rate, 3,  36);
}

/**
 * @brief Get field attitude_reference from ualberta_attitude message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_attitude_get_attitude_reference(const mavlink_message_t* msg, float *attitude_reference)
{
	return _MAV_RETURN_float_array(msg, attitude_reference, 2,  48);
}

/**
 * @brief Get field time_boot_ms from ualberta_attitude message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_ualberta_attitude_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  56);
}

/**
 * @brief Decode a ualberta_attitude message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_attitude_decode(const mavlink_message_t* msg, mavlink_ualberta_attitude_t* ualberta_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_ualberta_attitude_get_nav_euler(msg, ualberta_attitude->nav_euler);
	mavlink_msg_ualberta_attitude_get_nav_euler_rate(msg, ualberta_attitude->nav_euler_rate);
	mavlink_msg_ualberta_attitude_get_ahrs_euler(msg, ualberta_attitude->ahrs_euler);
	mavlink_msg_ualberta_attitude_get_ahrs_euler_rate(msg, ualberta_attitude->ahrs_euler_rate);
	mavlink_msg_ualberta_attitude_get_attitude_reference(msg, ualberta_attitude->attitude_reference);
	ualberta_attitude->time_boot_ms = mavlink_msg_ualberta_attitude_get_time_boot_ms(msg);
#else
	memcpy(ualberta_attitude, _MAV_PAYLOAD(msg), 60);
#endif
}
