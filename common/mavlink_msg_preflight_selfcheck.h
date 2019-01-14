#pragma once
// MESSAGE PREFLIGHT_SELFCHECK PACKING

#define MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK 12

MAVPACKED(
typedef struct __mavlink_preflight_selfcheck_t {
 uint8_t target_system; /*<  The system doing self-check*/
}) mavlink_preflight_selfcheck_t;

#define MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN 1
#define MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN 1
#define MAVLINK_MSG_ID_12_LEN 1
#define MAVLINK_MSG_ID_12_MIN_LEN 1

#define MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC 219
#define MAVLINK_MSG_ID_12_CRC 219



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PREFLIGHT_SELFCHECK { \
    12, \
    "PREFLIGHT_SELFCHECK", \
    1, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_preflight_selfcheck_t, target_system) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PREFLIGHT_SELFCHECK { \
    "PREFLIGHT_SELFCHECK", \
    1, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_preflight_selfcheck_t, target_system) }, \
         } \
}
#endif

/**
 * @brief Pack a preflight_selfcheck message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  The system doing self-check
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_preflight_selfcheck_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN);
#else
    mavlink_preflight_selfcheck_t packet;
    packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC);
}

/**
 * @brief Pack a preflight_selfcheck message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  The system doing self-check
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_preflight_selfcheck_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN);
#else
    mavlink_preflight_selfcheck_t packet;
    packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC);
}

/**
 * @brief Encode a preflight_selfcheck struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param preflight_selfcheck C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_preflight_selfcheck_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_preflight_selfcheck_t* preflight_selfcheck)
{
    return mavlink_msg_preflight_selfcheck_pack(system_id, component_id, msg, preflight_selfcheck->target_system);
}

/**
 * @brief Encode a preflight_selfcheck struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param preflight_selfcheck C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_preflight_selfcheck_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_preflight_selfcheck_t* preflight_selfcheck)
{
    return mavlink_msg_preflight_selfcheck_pack_chan(system_id, component_id, chan, msg, preflight_selfcheck->target_system);
}

/**
 * @brief Send a preflight_selfcheck message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  The system doing self-check
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_preflight_selfcheck_send(mavlink_channel_t chan, uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN];
    _mav_put_uint8_t(buf, 0, target_system);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK, buf, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC);
#else
    mavlink_preflight_selfcheck_t packet;
    packet.target_system = target_system;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK, (const char *)&packet, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC);
#endif
}

/**
 * @brief Send a preflight_selfcheck message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_preflight_selfcheck_send_struct(mavlink_channel_t chan, const mavlink_preflight_selfcheck_t* preflight_selfcheck)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_preflight_selfcheck_send(chan, preflight_selfcheck->target_system);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK, (const char *)preflight_selfcheck, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC);
#endif
}

#if MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_preflight_selfcheck_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK, buf, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC);
#else
    mavlink_preflight_selfcheck_t *packet = (mavlink_preflight_selfcheck_t *)msgbuf;
    packet->target_system = target_system;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK, (const char *)packet, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_MIN_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_CRC);
#endif
}
#endif

#endif

// MESSAGE PREFLIGHT_SELFCHECK UNPACKING


/**
 * @brief Get field target_system from preflight_selfcheck message
 *
 * @return  The system doing self-check
 */
static inline uint8_t mavlink_msg_preflight_selfcheck_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a preflight_selfcheck message into a struct
 *
 * @param msg The message to decode
 * @param preflight_selfcheck C-struct to decode the message contents into
 */
static inline void mavlink_msg_preflight_selfcheck_decode(const mavlink_message_t* msg, mavlink_preflight_selfcheck_t* preflight_selfcheck)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    preflight_selfcheck->target_system = mavlink_msg_preflight_selfcheck_get_target_system(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN? msg->len : MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN;
        memset(preflight_selfcheck, 0, MAVLINK_MSG_ID_PREFLIGHT_SELFCHECK_LEN);
    memcpy(preflight_selfcheck, _MAV_PAYLOAD(msg), len);
#endif
}
