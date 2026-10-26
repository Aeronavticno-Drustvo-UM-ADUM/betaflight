#pragma once
// MESSAGE ADUM_AUX_CHANNELS PACKING

#define MAVLINK_MSG_ID_ADUM_AUX_CHANNELS 13002


typedef struct __mavlink_adum_aux_channels_t {
 uint16_t motor[3];       // [us] Motor 1-3 output, as the INAV motor[] value (pulse width in microseconds).
                          // Index 0 is motor 1.
 uint16_t servo[8];       // [us] Servo 1-8 output, as the INAV servo[] value (pulse width in microseconds).
                          // Index 0 is servo 1.
 uint8_t arming_status;   // Arming status (ADUM_ARMING_STATUS):
                          //    0 DISARMED    Motors must not be driven.
                          //    1 ARMED       Motor values are valid commands.
} mavlink_adum_aux_channels_t;

#define MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN 23
#define MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN 23
#define MAVLINK_MSG_ID_13002_LEN 23
#define MAVLINK_MSG_ID_13002_MIN_LEN 23

#define MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC 190
#define MAVLINK_MSG_ID_13002_CRC 190

#define MAVLINK_MSG_ADUM_AUX_CHANNELS_FIELD_MOTOR_LEN 3
#define MAVLINK_MSG_ADUM_AUX_CHANNELS_FIELD_SERVO_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADUM_AUX_CHANNELS { \
    13002, \
    "ADUM_AUX_CHANNELS", \
    3, \
    {  { "arming_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_adum_aux_channels_t, arming_status) }, \
         { "motor", NULL, MAVLINK_TYPE_UINT16_T, 3, 0, offsetof(mavlink_adum_aux_channels_t, motor) }, \
         { "servo", NULL, MAVLINK_TYPE_UINT16_T, 8, 6, offsetof(mavlink_adum_aux_channels_t, servo) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADUM_AUX_CHANNELS { \
    "ADUM_AUX_CHANNELS", \
    3, \
    {  { "arming_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_adum_aux_channels_t, arming_status) }, \
         { "motor", NULL, MAVLINK_TYPE_UINT16_T, 3, 0, offsetof(mavlink_adum_aux_channels_t, motor) }, \
         { "servo", NULL, MAVLINK_TYPE_UINT16_T, 8, 6, offsetof(mavlink_adum_aux_channels_t, servo) }, \
         } \
}
#endif

/**
 * @brief Pack a adum_aux_channels message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param arming_status  Arming status (ADUM_ARMING_STATUS):
 *       0 DISARMED    Motors must not be driven.
 *       1 ARMED       Motor values are valid commands.
 * @param motor [us] Motor 1-3 output, as the INAV motor[] value (pulse width in microseconds).
 *   Index 0 is motor 1.
 * @param servo [us] Servo 1-8 output, as the INAV servo[] value (pulse width in microseconds).
 *   Index 0 is servo 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_aux_channels_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t arming_status, const uint16_t *motor, const uint16_t *servo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN];
    _mav_put_uint8_t(buf, 22, arming_status);
    _mav_put_uint16_t_array(buf, 0, motor, 3);
    _mav_put_uint16_t_array(buf, 6, servo, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
#else
    mavlink_adum_aux_channels_t packet;
    packet.arming_status = arming_status;
    mav_array_memcpy(packet.motor, motor, sizeof(uint16_t)*3);
    mav_array_memcpy(packet.servo, servo, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_AUX_CHANNELS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
}

/**
 * @brief Pack a adum_aux_channels message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param arming_status  Arming status (ADUM_ARMING_STATUS):
 *       0 DISARMED    Motors must not be driven.
 *       1 ARMED       Motor values are valid commands.
 * @param motor [us] Motor 1-3 output, as the INAV motor[] value (pulse width in microseconds).
 *   Index 0 is motor 1.
 * @param servo [us] Servo 1-8 output, as the INAV servo[] value (pulse width in microseconds).
 *   Index 0 is servo 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_aux_channels_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t arming_status, const uint16_t *motor, const uint16_t *servo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN];
    _mav_put_uint8_t(buf, 22, arming_status);
    _mav_put_uint16_t_array(buf, 0, motor, 3);
    _mav_put_uint16_t_array(buf, 6, servo, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
#else
    mavlink_adum_aux_channels_t packet;
    packet.arming_status = arming_status;
    mav_array_memcpy(packet.motor, motor, sizeof(uint16_t)*3);
    mav_array_memcpy(packet.servo, servo, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_AUX_CHANNELS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
#endif
}

/**
 * @brief Pack a adum_aux_channels message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arming_status  Arming status (ADUM_ARMING_STATUS):
 *       0 DISARMED    Motors must not be driven.
 *       1 ARMED       Motor values are valid commands.
 * @param motor [us] Motor 1-3 output, as the INAV motor[] value (pulse width in microseconds).
 *   Index 0 is motor 1.
 * @param servo [us] Servo 1-8 output, as the INAV servo[] value (pulse width in microseconds).
 *   Index 0 is servo 1.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_aux_channels_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t arming_status,const uint16_t *motor,const uint16_t *servo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN];
    _mav_put_uint8_t(buf, 22, arming_status);
    _mav_put_uint16_t_array(buf, 0, motor, 3);
    _mav_put_uint16_t_array(buf, 6, servo, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
#else
    mavlink_adum_aux_channels_t packet;
    packet.arming_status = arming_status;
    mav_array_memcpy(packet.motor, motor, sizeof(uint16_t)*3);
    mav_array_memcpy(packet.servo, servo, sizeof(uint16_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_AUX_CHANNELS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
}

/**
 * @brief Encode a adum_aux_channels struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adum_aux_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_aux_channels_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adum_aux_channels_t* adum_aux_channels)
{
    return mavlink_msg_adum_aux_channels_pack(system_id, component_id, msg, adum_aux_channels->arming_status, adum_aux_channels->motor, adum_aux_channels->servo);
}

/**
 * @brief Encode a adum_aux_channels struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adum_aux_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_aux_channels_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adum_aux_channels_t* adum_aux_channels)
{
    return mavlink_msg_adum_aux_channels_pack_chan(system_id, component_id, chan, msg, adum_aux_channels->arming_status, adum_aux_channels->motor, adum_aux_channels->servo);
}

/**
 * @brief Encode a adum_aux_channels struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param adum_aux_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_aux_channels_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_adum_aux_channels_t* adum_aux_channels)
{
    return mavlink_msg_adum_aux_channels_pack_status(system_id, component_id, _status, msg,  adum_aux_channels->arming_status, adum_aux_channels->motor, adum_aux_channels->servo);
}

/**
 * @brief Send a adum_aux_channels message
 * @param chan MAVLink channel to send the message
 *
 * @param arming_status  Arming status (ADUM_ARMING_STATUS):
 *       0 DISARMED    Motors must not be driven.
 *       1 ARMED       Motor values are valid commands.
 * @param motor [us] Motor 1-3 output, as the INAV motor[] value (pulse width in microseconds).
 *   Index 0 is motor 1.
 * @param servo [us] Servo 1-8 output, as the INAV servo[] value (pulse width in microseconds).
 *   Index 0 is servo 1.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adum_aux_channels_send(mavlink_channel_t chan, uint8_t arming_status, const uint16_t *motor, const uint16_t *servo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN];
    _mav_put_uint8_t(buf, 22, arming_status);
    _mav_put_uint16_t_array(buf, 0, motor, 3);
    _mav_put_uint16_t_array(buf, 6, servo, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS, buf, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
#else
    mavlink_adum_aux_channels_t packet;
    packet.arming_status = arming_status;
    mav_array_memcpy(packet.motor, motor, sizeof(uint16_t)*3);
    mav_array_memcpy(packet.servo, servo, sizeof(uint16_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS, (const char *)&packet, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
#endif
}

/**
 * @brief Send a adum_aux_channels message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adum_aux_channels_send_struct(mavlink_channel_t chan, const mavlink_adum_aux_channels_t* adum_aux_channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adum_aux_channels_send(chan, adum_aux_channels->arming_status, adum_aux_channels->motor, adum_aux_channels->servo);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS, (const char *)adum_aux_channels, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adum_aux_channels_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t arming_status, const uint16_t *motor, const uint16_t *servo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 22, arming_status);
    _mav_put_uint16_t_array(buf, 0, motor, 3);
    _mav_put_uint16_t_array(buf, 6, servo, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS, buf, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
#else
    mavlink_adum_aux_channels_t *packet = (mavlink_adum_aux_channels_t *)msgbuf;
    packet->arming_status = arming_status;
    mav_array_memcpy(packet->motor, motor, sizeof(uint16_t)*3);
    mav_array_memcpy(packet->servo, servo, sizeof(uint16_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS, (const char *)packet, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC);
#endif
}
#endif

#endif

// MESSAGE ADUM_AUX_CHANNELS UNPACKING


/**
 * @brief Get field arming_status from adum_aux_channels message
 *
 * @return  Arming status (ADUM_ARMING_STATUS):
 *       0 DISARMED    Motors must not be driven.
 *       1 ARMED       Motor values are valid commands.
 */
static inline uint8_t mavlink_msg_adum_aux_channels_get_arming_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field motor from adum_aux_channels message
 *
 * @return [us] Motor 1-3 output, as the INAV motor[] value (pulse width in microseconds).
 *   Index 0 is motor 1.
 */
static inline uint16_t mavlink_msg_adum_aux_channels_get_motor(const mavlink_message_t* msg, uint16_t *motor)
{
    return _MAV_RETURN_uint16_t_array(msg, motor, 3,  0);
}

/**
 * @brief Get field servo from adum_aux_channels message
 *
 * @return [us] Servo 1-8 output, as the INAV servo[] value (pulse width in microseconds).
 *   Index 0 is servo 1.
 */
static inline uint16_t mavlink_msg_adum_aux_channels_get_servo(const mavlink_message_t* msg, uint16_t *servo)
{
    return _MAV_RETURN_uint16_t_array(msg, servo, 8,  6);
}

/**
 * @brief Decode a adum_aux_channels message into a struct
 *
 * @param msg The message to decode
 * @param adum_aux_channels C-struct to decode the message contents into
 */
static inline void mavlink_msg_adum_aux_channels_decode(const mavlink_message_t* msg, mavlink_adum_aux_channels_t* adum_aux_channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adum_aux_channels_get_motor(msg, adum_aux_channels->motor);
    mavlink_msg_adum_aux_channels_get_servo(msg, adum_aux_channels->servo);
    adum_aux_channels->arming_status = mavlink_msg_adum_aux_channels_get_arming_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN? msg->len : MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN;
        memset(adum_aux_channels, 0, MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN);
    memcpy(adum_aux_channels, _MAV_PAYLOAD(msg), len);
#endif
}
