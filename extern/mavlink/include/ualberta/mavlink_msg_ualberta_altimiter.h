// MESSAGE UALBERTA_ALTIMETER PACKING

#define MAVLINK_MSG_ID_UALBERTA_ALTIMETER 229

typedef struct __mavlink_ualberta_altimeter_t
{
 float dist; ///< 
} mavlink_ualberta_altimeter_t;

#define MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN 4
#define MAVLINK_MSG_ID_229_LEN 4

#define MAVLINK_MSG_ID_UALBERTA_ALTIMETER_CRC 199
#define MAVLINK_MSG_ID_229_CRC 199



#define MAVLINK_MESSAGE_INFO_UALBERTA_ALTIMETER { \
	"UALBERTA_ALTIMETER", \
	1, \
	{  { "dist", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ualberta_altimeter_t, dist) }, \
         } \
}


/**
 * @brief Pack a ualberta_altimeter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dist 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_altimeter_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float dist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN];
	_mav_put_float(buf, 0, dist);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#else
	mavlink_ualberta_altimeter_t packet;
	packet.dist = dist;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_ALTIMETER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif
}

/**
 * @brief Pack a ualberta_altimeter message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dist 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_altimeter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float dist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN];
	_mav_put_float(buf, 0, dist);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#else
	mavlink_ualberta_altimeter_t packet;
	packet.dist = dist;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_ALTIMETER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif
}

/**
 * @brief Encode a ualberta_altimeter struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_altimeter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_altimeter_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_altimeter_t* ualberta_altimeter)
{
	return mavlink_msg_ualberta_altimeter_pack(system_id, component_id, msg, ualberta_altimeter->dist);
}

/**
 * @brief Encode a ualberta_altimeter struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_altimeter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_altimeter_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ualberta_altimeter_t* ualberta_altimeter)
{
	return mavlink_msg_ualberta_altimeter_pack_chan(system_id, component_id, chan, msg, ualberta_altimeter->dist);
}

/**
 * @brief Send a ualberta_altimeter message
 * @param chan MAVLink channel to send the message
 *
 * @param dist 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_altimeter_send(mavlink_channel_t chan, float dist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN];
	_mav_put_float(buf, 0, dist);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, buf, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, buf, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif
#else
	mavlink_ualberta_altimeter_t packet;
	packet.dist = dist;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, (const char *)&packet, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, (const char *)&packet, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ualberta_altimeter_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float dist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, dist);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, buf, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, buf, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif
#else
	mavlink_ualberta_altimeter_t *packet = (mavlink_ualberta_altimeter_t *)msgbuf;
	packet->dist = dist;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, (const char *)packet, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ALTIMETER, (const char *)packet, MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE UALBERTA_ALTIMETER UNPACKING


/**
 * @brief Get field dist from ualberta_altimeter message
 *
 * @return 
 */
static inline float mavlink_msg_ualberta_altimeter_get_dist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a ualberta_altimeter message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_altimeter C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_altimeter_decode(const mavlink_message_t* msg, mavlink_ualberta_altimeter_t* ualberta_altimeter)
{
#if MAVLINK_NEED_BYTE_SWAP
	ualberta_altimeter->dist = mavlink_msg_ualberta_altimeter_get_dist(msg);
#else
	memcpy(ualberta_altimeter, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_UALBERTA_ALTIMETER_LEN);
#endif
}
