// MESSAGE UALBERTA_GX3_MESSAGE PACKING

#define MAVLINK_MSG_ID_UALBERTA_GX3_MESSAGE 225

typedef struct __mavlink_ualberta_gx3_message_t
{
 char message[50]; ///< 
} mavlink_ualberta_gx3_message_t;

#define MAVLINK_MSG_ID_UALBERTA_GX3_MESSAGE_LEN 50
#define MAVLINK_MSG_ID_225_LEN 50

#define MAVLINK_MSG_UALBERTA_GX3_MESSAGE_FIELD_MESSAGE_LEN 50

#define MAVLINK_MESSAGE_INFO_UALBERTA_GX3_MESSAGE { \
	"UALBERTA_GX3_MESSAGE", \
	1, \
	{  { "message", NULL, MAVLINK_TYPE_CHAR, 50, 0, offsetof(mavlink_ualberta_gx3_message_t, message) }, \
         } \
}


/**
 * @brief Pack a ualberta_gx3_message message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param message 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_gx3_message_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[50];

	_mav_put_char_array(buf, 0, message, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 50);
#else
	mavlink_ualberta_gx3_message_t packet;

	mav_array_memcpy(packet.message, message, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 50);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_GX3_MESSAGE;
	return mavlink_finalize_message(msg, system_id, component_id, 50, 195);
}

/**
 * @brief Pack a ualberta_gx3_message message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param message 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_gx3_message_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[50];

	_mav_put_char_array(buf, 0, message, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 50);
#else
	mavlink_ualberta_gx3_message_t packet;

	mav_array_memcpy(packet.message, message, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 50);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_GX3_MESSAGE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 50, 195);
}

/**
 * @brief Encode a ualberta_gx3_message struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_gx3_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_gx3_message_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_gx3_message_t* ualberta_gx3_message)
{
	return mavlink_msg_ualberta_gx3_message_pack(system_id, component_id, msg, ualberta_gx3_message->message);
}

/**
 * @brief Send a ualberta_gx3_message message
 * @param chan MAVLink channel to send the message
 *
 * @param message 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_gx3_message_send(mavlink_channel_t chan, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[50];

	_mav_put_char_array(buf, 0, message, 50);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_GX3_MESSAGE, buf, 50, 195);
#else
	mavlink_ualberta_gx3_message_t packet;

	mav_array_memcpy(packet.message, message, sizeof(char)*50);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_GX3_MESSAGE, (const char *)&packet, 50, 195);
#endif
}

#endif

// MESSAGE UALBERTA_GX3_MESSAGE UNPACKING


/**
 * @brief Get field message from ualberta_gx3_message message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_ualberta_gx3_message_get_message(const mavlink_message_t* msg, char *message)
{
	return _MAV_RETURN_char_array(msg, message, 50,  0);
}

/**
 * @brief Decode a ualberta_gx3_message message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_gx3_message C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_gx3_message_decode(const mavlink_message_t* msg, mavlink_ualberta_gx3_message_t* ualberta_gx3_message)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_ualberta_gx3_message_get_message(msg, ualberta_gx3_message->message);
#else
	memcpy(ualberta_gx3_message, _MAV_PAYLOAD(msg), 50);
#endif
}
