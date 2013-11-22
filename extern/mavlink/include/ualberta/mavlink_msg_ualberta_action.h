// MESSAGE UALBERTA_ACTION PACKING

#define MAVLINK_MSG_ID_UALBERTA_ACTION 226

typedef struct __mavlink_ualberta_action_t
{
 uint8_t action; ///< 
 uint8_t param; ///< Additional parameter for action command
} mavlink_ualberta_action_t;

#define MAVLINK_MSG_ID_UALBERTA_ACTION_LEN 2
#define MAVLINK_MSG_ID_226_LEN 2



#define MAVLINK_MESSAGE_INFO_UALBERTA_ACTION { \
	"UALBERTA_ACTION", \
	2, \
	{  { "action", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ualberta_action_t, action) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_ualberta_action_t, param) }, \
         } \
}


/**
 * @brief Pack a ualberta_action message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param action 
 * @param param Additional parameter for action command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_action_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t action, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, action);
	_mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_ualberta_action_t packet;
	packet.action = action;
	packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_ACTION;
	return mavlink_finalize_message(msg, system_id, component_id, 2, 244);
}

/**
 * @brief Pack a ualberta_action message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param action 
 * @param param Additional parameter for action command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_action_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t action,uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, action);
	_mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_ualberta_action_t packet;
	packet.action = action;
	packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_UALBERTA_ACTION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 244);
}

/**
 * @brief Encode a ualberta_action struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_action C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_action_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_action_t* ualberta_action)
{
	return mavlink_msg_ualberta_action_pack(system_id, component_id, msg, ualberta_action->action, ualberta_action->param);
}

/**
 * @brief Send a ualberta_action message
 * @param chan MAVLink channel to send the message
 *
 * @param action 
 * @param param Additional parameter for action command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_action_send(mavlink_channel_t chan, uint8_t action, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, action);
	_mav_put_uint8_t(buf, 1, param);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ACTION, buf, 2, 244);
#else
	mavlink_ualberta_action_t packet;
	packet.action = action;
	packet.param = param;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_ACTION, (const char *)&packet, 2, 244);
#endif
}

#endif

// MESSAGE UALBERTA_ACTION UNPACKING


/**
 * @brief Get field action from ualberta_action message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_ualberta_action_get_action(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param from ualberta_action message
 *
 * @return Additional parameter for action command
 */
static inline uint8_t mavlink_msg_ualberta_action_get_param(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a ualberta_action message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_action C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_action_decode(const mavlink_message_t* msg, mavlink_ualberta_action_t* ualberta_action)
{
#if MAVLINK_NEED_BYTE_SWAP
	ualberta_action->action = mavlink_msg_ualberta_action_get_action(msg);
	ualberta_action->param = mavlink_msg_ualberta_action_get_param(msg);
#else
	memcpy(ualberta_action, _MAV_PAYLOAD(msg), 2);
#endif
}
