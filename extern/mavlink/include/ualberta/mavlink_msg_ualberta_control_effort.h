// MESSAGE UALBERTA_CONTROL_EFFORT PACKING

#define MAVLINK_MSG_ID_UALBERTA_CONTROL_EFFORT 228

typedef struct __mavlink_ualberta_control_effort_t
{
 float normalized_control_effort[6]; ///< 
} mavlink_ualberta_control_effort_t;

#define MAVLINK_MSG_ID_UALBERTA_CONTROL_EFFORT_LEN 24
#define MAVLINK_MSG_ID_228_LEN 24

#define MAVLINK_MSG_UALBERTA_CONTROL_EFFORT_FIELD_NORMALIZED_CONTROL_EFFORT_LEN 6

#define MAVLINK_MESSAGE_INFO_UALBERTA_CONTROL_EFFORT { \
	"UALBERTA_CONTROL_EFFORT", \
	1, \
	{  { "normalized_control_effort", NULL, MAVLINK_TYPE_FLOAT, 6, 0, offsetof(mavlink_ualberta_control_effort_t, normalized_control_effort) }, \
         } \
}


/**
 * @brief Pack a ualberta_control_effort message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param normalized_control_effort 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_control_effort_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *normalized_control_effort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];

	_mav_put_float_array(buf, 0, normalized_control_effort, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_ualberta_control_effort_t packet;

	mav_array_memcpy(packet.normalized_control_effort, normalized_control_effort, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_CONTROL_EFFORT;
	return mavlink_finalize_message(msg, system_id, component_id, 24, 204);
}

/**
 * @brief Pack a ualberta_control_effort message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param normalized_control_effort 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_control_effort_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *normalized_control_effort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];

	_mav_put_float_array(buf, 0, normalized_control_effort, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_ualberta_control_effort_t packet;

	mav_array_memcpy(packet.normalized_control_effort, normalized_control_effort, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_CONTROL_EFFORT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24, 204);
}

/**
 * @brief Encode a ualberta_control_effort struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_control_effort C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_control_effort_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_control_effort_t* ualberta_control_effort)
{
	return mavlink_msg_ualberta_control_effort_pack(system_id, component_id, msg, ualberta_control_effort->normalized_control_effort);
}

/**
 * @brief Send a ualberta_control_effort message
 * @param chan MAVLink channel to send the message
 *
 * @param normalized_control_effort 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_control_effort_send(mavlink_channel_t chan, const float *normalized_control_effort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];

	_mav_put_float_array(buf, 0, normalized_control_effort, 6);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_CONTROL_EFFORT, buf, 24, 204);
#else
	mavlink_ualberta_control_effort_t packet;

	mav_array_memcpy(packet.normalized_control_effort, normalized_control_effort, sizeof(float)*6);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_CONTROL_EFFORT, (const char *)&packet, 24, 204);
#endif
}

#endif

// MESSAGE UALBERTA_CONTROL_EFFORT UNPACKING


/**
 * @brief Get field normalized_control_effort from ualberta_control_effort message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_control_effort_get_normalized_control_effort(const mavlink_message_t* msg, float *normalized_control_effort)
{
	return _MAV_RETURN_float_array(msg, normalized_control_effort, 6,  0);
}

/**
 * @brief Decode a ualberta_control_effort message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_control_effort C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_control_effort_decode(const mavlink_message_t* msg, mavlink_ualberta_control_effort_t* ualberta_control_effort)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_ualberta_control_effort_get_normalized_control_effort(msg, ualberta_control_effort->normalized_control_effort);
#else
	memcpy(ualberta_control_effort, _MAV_PAYLOAD(msg), 24);
#endif
}
