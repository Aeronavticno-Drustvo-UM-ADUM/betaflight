#pragma once
// MESSAGE ADUM_MAGNETOMETER PACKING

#define MAVLINK_MSG_ID_ADUM_MAGNETOMETER 13001


typedef struct __mavlink_adum_magnetometer_t {
 uint32_t time_ms;   // [ms] Timestamp of the measurement (milliseconds, in the sender's own time base).
 int16_t mag_x;      // [mgauss] Magnetic field, X axis, pointing to the front of the vehicle.
 int16_t mag_y;      // [mgauss] Magnetic field, Y axis, pointing to the right of the vehicle.
 int16_t mag_z;      // [mgauss] Magnetic field, Z axis, pointing down.
 uint8_t instance;   // Sensor instance index, 0 for the first magnetometer.
} mavlink_adum_magnetometer_t;

#define MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN 11
#define MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN 11
#define MAVLINK_MSG_ID_13001_LEN 11
#define MAVLINK_MSG_ID_13001_MIN_LEN 11

#define MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC 12
#define MAVLINK_MSG_ID_13001_CRC 12



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADUM_MAGNETOMETER { \
    13001, \
    "ADUM_MAGNETOMETER", \
    5, \
    {  { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_adum_magnetometer_t, instance) }, \
         { "time_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adum_magnetometer_t, time_ms) }, \
         { "mag_x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_adum_magnetometer_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_adum_magnetometer_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_adum_magnetometer_t, mag_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADUM_MAGNETOMETER { \
    "ADUM_MAGNETOMETER", \
    5, \
    {  { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_adum_magnetometer_t, instance) }, \
         { "time_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adum_magnetometer_t, time_ms) }, \
         { "mag_x", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_adum_magnetometer_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_adum_magnetometer_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_adum_magnetometer_t, mag_z) }, \
         } \
}
#endif

/**
 * @brief Pack a adum_magnetometer message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param instance  Sensor instance index, 0 for the first magnetometer.
 * @param time_ms [ms] Timestamp of the measurement (milliseconds, in the sender's own time base).
 * @param mag_x [mgauss] Magnetic field, X axis, pointing to the front of the vehicle.
 * @param mag_y [mgauss] Magnetic field, Y axis, pointing to the right of the vehicle.
 * @param mag_z [mgauss] Magnetic field, Z axis, pointing down.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_magnetometer_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t instance, uint32_t time_ms, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN];
    _mav_put_uint32_t(buf, 0, time_ms);
    _mav_put_int16_t(buf, 4, mag_x);
    _mav_put_int16_t(buf, 6, mag_y);
    _mav_put_int16_t(buf, 8, mag_z);
    _mav_put_uint8_t(buf, 10, instance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
#else
    mavlink_adum_magnetometer_t packet;
    packet.time_ms = time_ms;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.instance = instance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_MAGNETOMETER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
}

/**
 * @brief Pack a adum_magnetometer message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param instance  Sensor instance index, 0 for the first magnetometer.
 * @param time_ms [ms] Timestamp of the measurement (milliseconds, in the sender's own time base).
 * @param mag_x [mgauss] Magnetic field, X axis, pointing to the front of the vehicle.
 * @param mag_y [mgauss] Magnetic field, Y axis, pointing to the right of the vehicle.
 * @param mag_z [mgauss] Magnetic field, Z axis, pointing down.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_magnetometer_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t instance, uint32_t time_ms, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN];
    _mav_put_uint32_t(buf, 0, time_ms);
    _mav_put_int16_t(buf, 4, mag_x);
    _mav_put_int16_t(buf, 6, mag_y);
    _mav_put_int16_t(buf, 8, mag_z);
    _mav_put_uint8_t(buf, 10, instance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
#else
    mavlink_adum_magnetometer_t packet;
    packet.time_ms = time_ms;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.instance = instance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_MAGNETOMETER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
#endif
}

/**
 * @brief Pack a adum_magnetometer message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param instance  Sensor instance index, 0 for the first magnetometer.
 * @param time_ms [ms] Timestamp of the measurement (milliseconds, in the sender's own time base).
 * @param mag_x [mgauss] Magnetic field, X axis, pointing to the front of the vehicle.
 * @param mag_y [mgauss] Magnetic field, Y axis, pointing to the right of the vehicle.
 * @param mag_z [mgauss] Magnetic field, Z axis, pointing down.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_magnetometer_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t instance,uint32_t time_ms,int16_t mag_x,int16_t mag_y,int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN];
    _mav_put_uint32_t(buf, 0, time_ms);
    _mav_put_int16_t(buf, 4, mag_x);
    _mav_put_int16_t(buf, 6, mag_y);
    _mav_put_int16_t(buf, 8, mag_z);
    _mav_put_uint8_t(buf, 10, instance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
#else
    mavlink_adum_magnetometer_t packet;
    packet.time_ms = time_ms;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.instance = instance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_MAGNETOMETER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
}

/**
 * @brief Encode a adum_magnetometer struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adum_magnetometer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_magnetometer_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adum_magnetometer_t* adum_magnetometer)
{
    return mavlink_msg_adum_magnetometer_pack(system_id, component_id, msg, adum_magnetometer->instance, adum_magnetometer->time_ms, adum_magnetometer->mag_x, adum_magnetometer->mag_y, adum_magnetometer->mag_z);
}

/**
 * @brief Encode a adum_magnetometer struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adum_magnetometer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_magnetometer_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adum_magnetometer_t* adum_magnetometer)
{
    return mavlink_msg_adum_magnetometer_pack_chan(system_id, component_id, chan, msg, adum_magnetometer->instance, adum_magnetometer->time_ms, adum_magnetometer->mag_x, adum_magnetometer->mag_y, adum_magnetometer->mag_z);
}

/**
 * @brief Encode a adum_magnetometer struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param adum_magnetometer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_magnetometer_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_adum_magnetometer_t* adum_magnetometer)
{
    return mavlink_msg_adum_magnetometer_pack_status(system_id, component_id, _status, msg,  adum_magnetometer->instance, adum_magnetometer->time_ms, adum_magnetometer->mag_x, adum_magnetometer->mag_y, adum_magnetometer->mag_z);
}

/**
 * @brief Send a adum_magnetometer message
 * @param chan MAVLink channel to send the message
 *
 * @param instance  Sensor instance index, 0 for the first magnetometer.
 * @param time_ms [ms] Timestamp of the measurement (milliseconds, in the sender's own time base).
 * @param mag_x [mgauss] Magnetic field, X axis, pointing to the front of the vehicle.
 * @param mag_y [mgauss] Magnetic field, Y axis, pointing to the right of the vehicle.
 * @param mag_z [mgauss] Magnetic field, Z axis, pointing down.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adum_magnetometer_send(mavlink_channel_t chan, uint8_t instance, uint32_t time_ms, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN];
    _mav_put_uint32_t(buf, 0, time_ms);
    _mav_put_int16_t(buf, 4, mag_x);
    _mav_put_int16_t(buf, 6, mag_y);
    _mav_put_int16_t(buf, 8, mag_z);
    _mav_put_uint8_t(buf, 10, instance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_MAGNETOMETER, buf, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
#else
    mavlink_adum_magnetometer_t packet;
    packet.time_ms = time_ms;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.instance = instance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_MAGNETOMETER, (const char *)&packet, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
#endif
}

/**
 * @brief Send a adum_magnetometer message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adum_magnetometer_send_struct(mavlink_channel_t chan, const mavlink_adum_magnetometer_t* adum_magnetometer)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adum_magnetometer_send(chan, adum_magnetometer->instance, adum_magnetometer->time_ms, adum_magnetometer->mag_x, adum_magnetometer->mag_y, adum_magnetometer->mag_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_MAGNETOMETER, (const char *)adum_magnetometer, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adum_magnetometer_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t instance, uint32_t time_ms, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_ms);
    _mav_put_int16_t(buf, 4, mag_x);
    _mav_put_int16_t(buf, 6, mag_y);
    _mav_put_int16_t(buf, 8, mag_z);
    _mav_put_uint8_t(buf, 10, instance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_MAGNETOMETER, buf, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
#else
    mavlink_adum_magnetometer_t *packet = (mavlink_adum_magnetometer_t *)msgbuf;
    packet->time_ms = time_ms;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;
    packet->instance = instance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_MAGNETOMETER, (const char *)packet, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_CRC);
#endif
}
#endif

#endif

// MESSAGE ADUM_MAGNETOMETER UNPACKING


/**
 * @brief Get field instance from adum_magnetometer message
 *
 * @return  Sensor instance index, 0 for the first magnetometer.
 */
static inline uint8_t mavlink_msg_adum_magnetometer_get_instance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field time_ms from adum_magnetometer message
 *
 * @return [ms] Timestamp of the measurement (milliseconds, in the sender's own time base).
 */
static inline uint32_t mavlink_msg_adum_magnetometer_get_time_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field mag_x from adum_magnetometer message
 *
 * @return [mgauss] Magnetic field, X axis, pointing to the front of the vehicle.
 */
static inline int16_t mavlink_msg_adum_magnetometer_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field mag_y from adum_magnetometer message
 *
 * @return [mgauss] Magnetic field, Y axis, pointing to the right of the vehicle.
 */
static inline int16_t mavlink_msg_adum_magnetometer_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field mag_z from adum_magnetometer message
 *
 * @return [mgauss] Magnetic field, Z axis, pointing down.
 */
static inline int16_t mavlink_msg_adum_magnetometer_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Decode a adum_magnetometer message into a struct
 *
 * @param msg The message to decode
 * @param adum_magnetometer C-struct to decode the message contents into
 */
static inline void mavlink_msg_adum_magnetometer_decode(const mavlink_message_t* msg, mavlink_adum_magnetometer_t* adum_magnetometer)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    adum_magnetometer->time_ms = mavlink_msg_adum_magnetometer_get_time_ms(msg);
    adum_magnetometer->mag_x = mavlink_msg_adum_magnetometer_get_mag_x(msg);
    adum_magnetometer->mag_y = mavlink_msg_adum_magnetometer_get_mag_y(msg);
    adum_magnetometer->mag_z = mavlink_msg_adum_magnetometer_get_mag_z(msg);
    adum_magnetometer->instance = mavlink_msg_adum_magnetometer_get_instance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN? msg->len : MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN;
        memset(adum_magnetometer, 0, MAVLINK_MSG_ID_ADUM_MAGNETOMETER_LEN);
    memcpy(adum_magnetometer, _MAV_PAYLOAD(msg), len);
#endif
}
