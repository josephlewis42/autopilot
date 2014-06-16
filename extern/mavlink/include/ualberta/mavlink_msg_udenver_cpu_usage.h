// MESSAGE UDENVER_CPU_USAGE PACKING

#define MAVLINK_MSG_ID_UDENVER_CPU_USAGE 230

typedef struct __mavlink_udenver_cpu_usage_t
{
 float cpu_usage; ///< 
} mavlink_udenver_cpu_usage_t;

#define MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN 4
#define MAVLINK_MSG_ID_230_LEN 4

#define MAVLINK_MSG_ID_UDENVER_CPU_USAGE_CRC 254
#define MAVLINK_MSG_ID_230_CRC 254



#define MAVLINK_MESSAGE_INFO_UDENVER_CPU_USAGE { \
	"UDENVER_CPU_USAGE", \
	1, \
	{  { "cpu_usage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_udenver_cpu_usage_t, cpu_usage) }, \
         } \
}


/**
 * @brief Pack a udenver_cpu_usage message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cpu_usage 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_udenver_cpu_usage_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float cpu_usage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN];
	_mav_put_float(buf, 0, cpu_usage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#else
	mavlink_udenver_cpu_usage_t packet;
	packet.cpu_usage = cpu_usage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UDENVER_CPU_USAGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif
}

/**
 * @brief Pack a udenver_cpu_usage message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cpu_usage 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_udenver_cpu_usage_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float cpu_usage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN];
	_mav_put_float(buf, 0, cpu_usage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#else
	mavlink_udenver_cpu_usage_t packet;
	packet.cpu_usage = cpu_usage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UDENVER_CPU_USAGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif
}

/**
 * @brief Encode a udenver_cpu_usage struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param udenver_cpu_usage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_udenver_cpu_usage_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_udenver_cpu_usage_t* udenver_cpu_usage)
{
	return mavlink_msg_udenver_cpu_usage_pack(system_id, component_id, msg, udenver_cpu_usage->cpu_usage);
}

/**
 * @brief Encode a udenver_cpu_usage struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param udenver_cpu_usage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_udenver_cpu_usage_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_udenver_cpu_usage_t* udenver_cpu_usage)
{
	return mavlink_msg_udenver_cpu_usage_pack_chan(system_id, component_id, chan, msg, udenver_cpu_usage->cpu_usage);
}

/**
 * @brief Send a udenver_cpu_usage message
 * @param chan MAVLink channel to send the message
 *
 * @param cpu_usage 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_udenver_cpu_usage_send(mavlink_channel_t chan, float cpu_usage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN];
	_mav_put_float(buf, 0, cpu_usage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, buf, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, buf, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif
#else
	mavlink_udenver_cpu_usage_t packet;
	packet.cpu_usage = cpu_usage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, (const char *)&packet, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, (const char *)&packet, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_udenver_cpu_usage_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float cpu_usage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, cpu_usage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, buf, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, buf, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif
#else
	mavlink_udenver_cpu_usage_t *packet = (mavlink_udenver_cpu_usage_t *)msgbuf;
	packet->cpu_usage = cpu_usage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, (const char *)packet, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UDENVER_CPU_USAGE, (const char *)packet, MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE UDENVER_CPU_USAGE UNPACKING


/**
 * @brief Get field cpu_usage from udenver_cpu_usage message
 *
 * @return 
 */
static inline float mavlink_msg_udenver_cpu_usage_get_cpu_usage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a udenver_cpu_usage message into a struct
 *
 * @param msg The message to decode
 * @param udenver_cpu_usage C-struct to decode the message contents into
 */
static inline void mavlink_msg_udenver_cpu_usage_decode(const mavlink_message_t* msg, mavlink_udenver_cpu_usage_t* udenver_cpu_usage)
{
#if MAVLINK_NEED_BYTE_SWAP
	udenver_cpu_usage->cpu_usage = mavlink_msg_udenver_cpu_usage_get_cpu_usage(msg);
#else
	memcpy(udenver_cpu_usage, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_UDENVER_CPU_USAGE_LEN);
#endif
}
