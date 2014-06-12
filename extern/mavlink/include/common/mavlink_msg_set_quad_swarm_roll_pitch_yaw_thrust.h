// MESSAGE SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST PACKING

#define MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST 61

typedef struct __mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t
{
 int16_t roll[6]; ///< Desired roll angle in radians, scaled to int16 for 6 quadrotors: 0..5
 int16_t pitch[6]; ///< Desired pitch angle in radians, scaled to int16 for 6 quadrotors: 0..5
 int16_t yaw[6]; ///< Desired yaw angle in radians, scaled to int16 for 6 quadrotors: 0..5
 uint16_t thrust[6]; ///< Collective thrust, scaled to uint16 for 6 quadrotors: 0..5
 uint8_t target_systems[6]; ///< System IDs for 6 quadrotors: 0..5, the ID's are the MAVLink IDs
} mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t;

#define MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN 54
#define MAVLINK_MSG_ID_61_LEN 54

#define MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_CRC 200
#define MAVLINK_MSG_ID_61_CRC 200

#define MAVLINK_MSG_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_FIELD_ROLL_LEN 6
#define MAVLINK_MSG_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_FIELD_PITCH_LEN 6
#define MAVLINK_MSG_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_FIELD_YAW_LEN 6
#define MAVLINK_MSG_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_FIELD_THRUST_LEN 6
#define MAVLINK_MSG_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_FIELD_TARGET_SYSTEMS_LEN 6

#define MAVLINK_MESSAGE_INFO_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST { \
	"SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST", \
	5, \
	{  { "roll", NULL, MAVLINK_TYPE_INT16_T, 6, 0, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 6, 12, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 6, 24, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_UINT16_T, 6, 36, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, thrust) }, \
         { "target_systems", NULL, MAVLINK_TYPE_UINT8_T, 6, 48, offsetof(mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t, target_systems) }, \
         } \
}


/**
 * @brief Pack a set_quad_swarm_roll_pitch_yaw_thrust message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_systems System IDs for 6 quadrotors: 0..5, the ID's are the MAVLink IDs
 * @param roll Desired roll angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param pitch Desired pitch angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param yaw Desired yaw angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param thrust Collective thrust, scaled to uint16 for 6 quadrotors: 0..5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint8_t *target_systems, const int16_t *roll, const int16_t *pitch, const int16_t *yaw, const uint16_t *thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN];

	_mav_put_int16_t_array(buf, 0, roll, 6);
	_mav_put_int16_t_array(buf, 12, pitch, 6);
	_mav_put_int16_t_array(buf, 24, yaw, 6);
	_mav_put_uint16_t_array(buf, 36, thrust, 6);
	_mav_put_uint8_t_array(buf, 48, target_systems, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#else
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet;

	mav_array_memcpy(packet.roll, roll, sizeof(int16_t)*6);
	mav_array_memcpy(packet.pitch, pitch, sizeof(int16_t)*6);
	mav_array_memcpy(packet.yaw, yaw, sizeof(int16_t)*6);
	mav_array_memcpy(packet.thrust, thrust, sizeof(uint16_t)*6);
	mav_array_memcpy(packet.target_systems, target_systems, sizeof(uint8_t)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
}

/**
 * @brief Pack a set_quad_swarm_roll_pitch_yaw_thrust message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_systems System IDs for 6 quadrotors: 0..5, the ID's are the MAVLink IDs
 * @param roll Desired roll angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param pitch Desired pitch angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param yaw Desired yaw angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param thrust Collective thrust, scaled to uint16 for 6 quadrotors: 0..5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint8_t *target_systems,const int16_t *roll,const int16_t *pitch,const int16_t *yaw,const uint16_t *thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN];

	_mav_put_int16_t_array(buf, 0, roll, 6);
	_mav_put_int16_t_array(buf, 12, pitch, 6);
	_mav_put_int16_t_array(buf, 24, yaw, 6);
	_mav_put_uint16_t_array(buf, 36, thrust, 6);
	_mav_put_uint8_t_array(buf, 48, target_systems, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#else
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet;

	mav_array_memcpy(packet.roll, roll, sizeof(int16_t)*6);
	mav_array_memcpy(packet.pitch, pitch, sizeof(int16_t)*6);
	mav_array_memcpy(packet.yaw, yaw, sizeof(int16_t)*6);
	mav_array_memcpy(packet.thrust, thrust, sizeof(uint16_t)*6);
	mav_array_memcpy(packet.target_systems, target_systems, sizeof(uint8_t)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
}

/**
 * @brief Encode a set_quad_swarm_roll_pitch_yaw_thrust struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_quad_swarm_roll_pitch_yaw_thrust C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t* set_quad_swarm_roll_pitch_yaw_thrust)
{
	return mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack(system_id, component_id, msg, set_quad_swarm_roll_pitch_yaw_thrust->target_systems, set_quad_swarm_roll_pitch_yaw_thrust->roll, set_quad_swarm_roll_pitch_yaw_thrust->pitch, set_quad_swarm_roll_pitch_yaw_thrust->yaw, set_quad_swarm_roll_pitch_yaw_thrust->thrust);
}

/**
 * @brief Encode a set_quad_swarm_roll_pitch_yaw_thrust struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_quad_swarm_roll_pitch_yaw_thrust C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t* set_quad_swarm_roll_pitch_yaw_thrust)
{
	return mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_pack_chan(system_id, component_id, chan, msg, set_quad_swarm_roll_pitch_yaw_thrust->target_systems, set_quad_swarm_roll_pitch_yaw_thrust->roll, set_quad_swarm_roll_pitch_yaw_thrust->pitch, set_quad_swarm_roll_pitch_yaw_thrust->yaw, set_quad_swarm_roll_pitch_yaw_thrust->thrust);
}

/**
 * @brief Send a set_quad_swarm_roll_pitch_yaw_thrust message
 * @param chan MAVLink channel to send the message
 *
 * @param target_systems System IDs for 6 quadrotors: 0..5, the ID's are the MAVLink IDs
 * @param roll Desired roll angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param pitch Desired pitch angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param yaw Desired yaw angle in radians, scaled to int16 for 6 quadrotors: 0..5
 * @param thrust Collective thrust, scaled to uint16 for 6 quadrotors: 0..5
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_send(mavlink_channel_t chan, const uint8_t *target_systems, const int16_t *roll, const int16_t *pitch, const int16_t *yaw, const uint16_t *thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN];

	_mav_put_int16_t_array(buf, 0, roll, 6);
	_mav_put_int16_t_array(buf, 12, pitch, 6);
	_mav_put_int16_t_array(buf, 24, yaw, 6);
	_mav_put_uint16_t_array(buf, 36, thrust, 6);
	_mav_put_uint8_t_array(buf, 48, target_systems, 6);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
#else
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t packet;

	mav_array_memcpy(packet.roll, roll, sizeof(int16_t)*6);
	mav_array_memcpy(packet.pitch, pitch, sizeof(int16_t)*6);
	mav_array_memcpy(packet.yaw, yaw, sizeof(int16_t)*6);
	mav_array_memcpy(packet.thrust, thrust, sizeof(uint16_t)*6);
	mav_array_memcpy(packet.target_systems, target_systems, sizeof(uint8_t)*6);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, (const char *)&packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, (const char *)&packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *target_systems, const int16_t *roll, const int16_t *pitch, const int16_t *yaw, const uint16_t *thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_int16_t_array(buf, 0, roll, 6);
	_mav_put_int16_t_array(buf, 12, pitch, 6);
	_mav_put_int16_t_array(buf, 24, yaw, 6);
	_mav_put_uint16_t_array(buf, 36, thrust, 6);
	_mav_put_uint8_t_array(buf, 48, target_systems, 6);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, buf, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
#else
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t *packet = (mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t *)msgbuf;

	mav_array_memcpy(packet->roll, roll, sizeof(int16_t)*6);
	mav_array_memcpy(packet->pitch, pitch, sizeof(int16_t)*6);
	mav_array_memcpy(packet->yaw, yaw, sizeof(int16_t)*6);
	mav_array_memcpy(packet->thrust, thrust, sizeof(uint16_t)*6);
	mav_array_memcpy(packet->target_systems, target_systems, sizeof(uint8_t)*6);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, (const char *)packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST, (const char *)packet, MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST UNPACKING


/**
 * @brief Get field target_systems from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return System IDs for 6 quadrotors: 0..5, the ID's are the MAVLink IDs
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_target_systems(const mavlink_message_t* msg, uint8_t *target_systems)
{
	return _MAV_RETURN_uint8_t_array(msg, target_systems, 6,  48);
}

/**
 * @brief Get field roll from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Desired roll angle in radians, scaled to int16 for 6 quadrotors: 0..5
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_roll(const mavlink_message_t* msg, int16_t *roll)
{
	return _MAV_RETURN_int16_t_array(msg, roll, 6,  0);
}

/**
 * @brief Get field pitch from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Desired pitch angle in radians, scaled to int16 for 6 quadrotors: 0..5
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_pitch(const mavlink_message_t* msg, int16_t *pitch)
{
	return _MAV_RETURN_int16_t_array(msg, pitch, 6,  12);
}

/**
 * @brief Get field yaw from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Desired yaw angle in radians, scaled to int16 for 6 quadrotors: 0..5
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_yaw(const mavlink_message_t* msg, int16_t *yaw)
{
	return _MAV_RETURN_int16_t_array(msg, yaw, 6,  24);
}

/**
 * @brief Get field thrust from set_quad_swarm_roll_pitch_yaw_thrust message
 *
 * @return Collective thrust, scaled to uint16 for 6 quadrotors: 0..5
 */
static inline uint16_t mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_thrust(const mavlink_message_t* msg, uint16_t *thrust)
{
	return _MAV_RETURN_uint16_t_array(msg, thrust, 6,  36);
}

/**
 * @brief Decode a set_quad_swarm_roll_pitch_yaw_thrust message into a struct
 *
 * @param msg The message to decode
 * @param set_quad_swarm_roll_pitch_yaw_thrust C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(const mavlink_message_t* msg, mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t* set_quad_swarm_roll_pitch_yaw_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_roll(msg, set_quad_swarm_roll_pitch_yaw_thrust->roll);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_pitch(msg, set_quad_swarm_roll_pitch_yaw_thrust->pitch);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_yaw(msg, set_quad_swarm_roll_pitch_yaw_thrust->yaw);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_thrust(msg, set_quad_swarm_roll_pitch_yaw_thrust->thrust);
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_get_target_systems(msg, set_quad_swarm_roll_pitch_yaw_thrust->target_systems);
#else
	memcpy(set_quad_swarm_roll_pitch_yaw_thrust, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST_LEN);
#endif
}
