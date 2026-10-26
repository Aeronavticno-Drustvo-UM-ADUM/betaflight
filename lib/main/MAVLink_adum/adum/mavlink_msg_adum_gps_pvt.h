#pragma once
// MESSAGE ADUM_GPS_PVT PACKING

#define MAVLINK_MSG_ID_ADUM_GPS_PVT 13000


typedef struct __mavlink_adum_gps_pvt_t {
 uint64_t time_usec;           // [us] UTC time since the UNIX epoch (1970-01-01 00:00:00 UTC) of this solution.
                               // Valid only if ADUM_GPS_PVT_FLAGS_TIME_VALID is set, otherwise 0.
 int32_t lat;                  // [degE7] Latitude (WGS84), positive north.
                               // Degrees multiplied by 1e7.
 int32_t lon;                  // [degE7] Longitude (WGS84), positive east.
                               // Degrees multiplied by 1e7.
 int32_t alt_msl;              // [mm] Altitude above mean sea level (geoid), positive up.
                               // Equals alt_ellipsoid minus the geoid separation at this position.
 int32_t alt_ellipsoid;        // [mm] Altitude above the WGS84 ellipsoid, positive up.
 int32_t vel_n;                // [mm/s] Velocity over ground, North component (positive north).
 int32_t vel_e;                // [mm/s] Velocity over ground, East component (positive east).
 int32_t vel_d;                // [mm/s] Velocity over ground, Down component (positive down, so a climb is negative).
 int32_t heading_motion;       // [degE5] Heading of motion (course over ground): direction of the velocity vector,
                               // 0..359.99999 degrees clockwise from true North, multiplied by 1e5.
                               // This is NOT the vehicle's attitude heading.
                               // Valid only if ADUM_GPS_PVT_FLAGS_HEADING_VALID is set.
 uint32_t h_acc;               // [mm] Horizontal position accuracy estimate, as reported by the GPS receiver.
 uint32_t v_acc;               // [mm] Vertical position accuracy estimate, as reported by the GPS receiver.
 uint32_t s_acc;               // [mm/s] Speed accuracy estimate, as reported by the GPS receiver.
 uint16_t pdop;                // Position dilution of precision, scaled by 100 (1.23 is sent as 123).
                               // UINT16_MAX if unknown.
 uint8_t fix_type;             // Fix type (GPS_FIX_TYPE). Values:
                               //    0 NO_GPS    No GPS connected.
                               //    1 NO_FIX    GPS connected, but no position information.
                               //    2 2D_FIX    2D position (no usable altitude).
                               //    3 3D_FIX    3D position, standalone.
                               //    4 DGPS      3D position aided by DGPS / SBAS.
                               //    5 RTK_FLOAT RTK solution with float carrier-phase ambiguities (decimetre-level).
                               //    6 RTK_FIXED RTK solution with fixed carrier-phase ambiguities (centimetre-level).
                               //    7 STATIC    Static fixed position, typically used on base stations.
                               //    8 PPP       Precise Point Positioning, 3D position.
                               // Only 3 and above give a usable 3D position. ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS
                               // is set for 4, 5 and 6.
 uint8_t satellites_visible;   // Number of satellites used in the solution.
                               // UINT8_MAX if unknown.
 uint8_t flags;                // Status flags (ADUM_GPS_PVT_FLAGS bitmask):
                               //    1 TIME_VALID          time_usec is valid.
                               //    2 DIFF_CORRECTIONS    differential (RTK) corrections applied.
                               //    4 HEADING_VALID       heading_motion is valid.
} mavlink_adum_gps_pvt_t;

#define MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN 57
#define MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN 57
#define MAVLINK_MSG_ID_13000_LEN 57
#define MAVLINK_MSG_ID_13000_MIN_LEN 57

#define MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC 116
#define MAVLINK_MSG_ID_13000_CRC 116



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADUM_GPS_PVT { \
    13000, \
    "ADUM_GPS_PVT", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adum_gps_pvt_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_adum_gps_pvt_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_adum_gps_pvt_t, lon) }, \
         { "alt_msl", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_adum_gps_pvt_t, alt_msl) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_adum_gps_pvt_t, alt_ellipsoid) }, \
         { "vel_n", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_adum_gps_pvt_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_adum_gps_pvt_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_adum_gps_pvt_t, vel_d) }, \
         { "heading_motion", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_adum_gps_pvt_t, heading_motion) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_adum_gps_pvt_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 44, offsetof(mavlink_adum_gps_pvt_t, v_acc) }, \
         { "s_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 48, offsetof(mavlink_adum_gps_pvt_t, s_acc) }, \
         { "pdop", NULL, MAVLINK_TYPE_UINT16_T, 0, 52, offsetof(mavlink_adum_gps_pvt_t, pdop) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_adum_gps_pvt_t, fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_adum_gps_pvt_t, satellites_visible) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_adum_gps_pvt_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADUM_GPS_PVT { \
    "ADUM_GPS_PVT", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_adum_gps_pvt_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_adum_gps_pvt_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_adum_gps_pvt_t, lon) }, \
         { "alt_msl", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_adum_gps_pvt_t, alt_msl) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_adum_gps_pvt_t, alt_ellipsoid) }, \
         { "vel_n", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_adum_gps_pvt_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_adum_gps_pvt_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_adum_gps_pvt_t, vel_d) }, \
         { "heading_motion", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_adum_gps_pvt_t, heading_motion) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_adum_gps_pvt_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 44, offsetof(mavlink_adum_gps_pvt_t, v_acc) }, \
         { "s_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 48, offsetof(mavlink_adum_gps_pvt_t, s_acc) }, \
         { "pdop", NULL, MAVLINK_TYPE_UINT16_T, 0, 52, offsetof(mavlink_adum_gps_pvt_t, pdop) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_adum_gps_pvt_t, fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_adum_gps_pvt_t, satellites_visible) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_adum_gps_pvt_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a adum_gps_pvt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] UTC time since the UNIX epoch (1970-01-01 00:00:00 UTC) of this solution.
 *   Valid only if ADUM_GPS_PVT_FLAGS_TIME_VALID is set, otherwise 0.
 * @param lat [degE7] Latitude (WGS84), positive north.
 *   Degrees multiplied by 1e7.
 * @param lon [degE7] Longitude (WGS84), positive east.
 *   Degrees multiplied by 1e7.
 * @param alt_msl [mm] Altitude above mean sea level (geoid), positive up.
 *   Equals alt_ellipsoid minus the geoid separation at this position.
 * @param alt_ellipsoid [mm] Altitude above the WGS84 ellipsoid, positive up.
 * @param vel_n [mm/s] Velocity over ground, North component (positive north).
 * @param vel_e [mm/s] Velocity over ground, East component (positive east).
 * @param vel_d [mm/s] Velocity over ground, Down component (positive down, so a climb is negative).
 * @param heading_motion [degE5] Heading of motion (course over ground): direction of the velocity vector,
 *   0..359.99999 degrees clockwise from true North, multiplied by 1e5.
 *   This is NOT the vehicle's attitude heading.
 *   Valid only if ADUM_GPS_PVT_FLAGS_HEADING_VALID is set.
 * @param h_acc [mm] Horizontal position accuracy estimate, as reported by the GPS receiver.
 * @param v_acc [mm] Vertical position accuracy estimate, as reported by the GPS receiver.
 * @param s_acc [mm/s] Speed accuracy estimate, as reported by the GPS receiver.
 * @param pdop  Position dilution of precision, scaled by 100 (1.23 is sent as 123).
 *   UINT16_MAX if unknown.
 * @param fix_type  Fix type (GPS_FIX_TYPE). Values:
 *       0 NO_GPS    No GPS connected.
 *       1 NO_FIX    GPS connected, but no position information.
 *       2 2D_FIX    2D position (no usable altitude).
 *       3 3D_FIX    3D position, standalone.
 *       4 DGPS      3D position aided by DGPS / SBAS.
 *       5 RTK_FLOAT RTK solution with float carrier-phase ambiguities (decimetre-level).
 *       6 RTK_FIXED RTK solution with fixed carrier-phase ambiguities (centimetre-level).
 *       7 STATIC    Static fixed position, typically used on base stations.
 *       8 PPP       Precise Point Positioning, 3D position.
 *   Only 3 and above give a usable 3D position. ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS
 *   is set for 4, 5 and 6.
 * @param satellites_visible  Number of satellites used in the solution.
 *   UINT8_MAX if unknown.
 * @param flags  Status flags (ADUM_GPS_PVT_FLAGS bitmask):
 *       1 TIME_VALID          time_usec is valid.
 *       2 DIFF_CORRECTIONS    differential (RTK) corrections applied.
 *       4 HEADING_VALID       heading_motion is valid.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_gps_pvt_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt_msl, int32_t alt_ellipsoid, int32_t vel_n, int32_t vel_e, int32_t vel_d, int32_t heading_motion, uint32_t h_acc, uint32_t v_acc, uint32_t s_acc, uint16_t pdop, uint8_t fix_type, uint8_t satellites_visible, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt_msl);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_int32_t(buf, 24, vel_n);
    _mav_put_int32_t(buf, 28, vel_e);
    _mav_put_int32_t(buf, 32, vel_d);
    _mav_put_int32_t(buf, 36, heading_motion);
    _mav_put_uint32_t(buf, 40, h_acc);
    _mav_put_uint32_t(buf, 44, v_acc);
    _mav_put_uint32_t(buf, 48, s_acc);
    _mav_put_uint16_t(buf, 52, pdop);
    _mav_put_uint8_t(buf, 54, fix_type);
    _mav_put_uint8_t(buf, 55, satellites_visible);
    _mav_put_uint8_t(buf, 56, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
#else
    mavlink_adum_gps_pvt_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.heading_motion = heading_motion;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.pdop = pdop;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_GPS_PVT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
}

/**
 * @brief Pack a adum_gps_pvt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] UTC time since the UNIX epoch (1970-01-01 00:00:00 UTC) of this solution.
 *   Valid only if ADUM_GPS_PVT_FLAGS_TIME_VALID is set, otherwise 0.
 * @param lat [degE7] Latitude (WGS84), positive north.
 *   Degrees multiplied by 1e7.
 * @param lon [degE7] Longitude (WGS84), positive east.
 *   Degrees multiplied by 1e7.
 * @param alt_msl [mm] Altitude above mean sea level (geoid), positive up.
 *   Equals alt_ellipsoid minus the geoid separation at this position.
 * @param alt_ellipsoid [mm] Altitude above the WGS84 ellipsoid, positive up.
 * @param vel_n [mm/s] Velocity over ground, North component (positive north).
 * @param vel_e [mm/s] Velocity over ground, East component (positive east).
 * @param vel_d [mm/s] Velocity over ground, Down component (positive down, so a climb is negative).
 * @param heading_motion [degE5] Heading of motion (course over ground): direction of the velocity vector,
 *   0..359.99999 degrees clockwise from true North, multiplied by 1e5.
 *   This is NOT the vehicle's attitude heading.
 *   Valid only if ADUM_GPS_PVT_FLAGS_HEADING_VALID is set.
 * @param h_acc [mm] Horizontal position accuracy estimate, as reported by the GPS receiver.
 * @param v_acc [mm] Vertical position accuracy estimate, as reported by the GPS receiver.
 * @param s_acc [mm/s] Speed accuracy estimate, as reported by the GPS receiver.
 * @param pdop  Position dilution of precision, scaled by 100 (1.23 is sent as 123).
 *   UINT16_MAX if unknown.
 * @param fix_type  Fix type (GPS_FIX_TYPE). Values:
 *       0 NO_GPS    No GPS connected.
 *       1 NO_FIX    GPS connected, but no position information.
 *       2 2D_FIX    2D position (no usable altitude).
 *       3 3D_FIX    3D position, standalone.
 *       4 DGPS      3D position aided by DGPS / SBAS.
 *       5 RTK_FLOAT RTK solution with float carrier-phase ambiguities (decimetre-level).
 *       6 RTK_FIXED RTK solution with fixed carrier-phase ambiguities (centimetre-level).
 *       7 STATIC    Static fixed position, typically used on base stations.
 *       8 PPP       Precise Point Positioning, 3D position.
 *   Only 3 and above give a usable 3D position. ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS
 *   is set for 4, 5 and 6.
 * @param satellites_visible  Number of satellites used in the solution.
 *   UINT8_MAX if unknown.
 * @param flags  Status flags (ADUM_GPS_PVT_FLAGS bitmask):
 *       1 TIME_VALID          time_usec is valid.
 *       2 DIFF_CORRECTIONS    differential (RTK) corrections applied.
 *       4 HEADING_VALID       heading_motion is valid.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_gps_pvt_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt_msl, int32_t alt_ellipsoid, int32_t vel_n, int32_t vel_e, int32_t vel_d, int32_t heading_motion, uint32_t h_acc, uint32_t v_acc, uint32_t s_acc, uint16_t pdop, uint8_t fix_type, uint8_t satellites_visible, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt_msl);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_int32_t(buf, 24, vel_n);
    _mav_put_int32_t(buf, 28, vel_e);
    _mav_put_int32_t(buf, 32, vel_d);
    _mav_put_int32_t(buf, 36, heading_motion);
    _mav_put_uint32_t(buf, 40, h_acc);
    _mav_put_uint32_t(buf, 44, v_acc);
    _mav_put_uint32_t(buf, 48, s_acc);
    _mav_put_uint16_t(buf, 52, pdop);
    _mav_put_uint8_t(buf, 54, fix_type);
    _mav_put_uint8_t(buf, 55, satellites_visible);
    _mav_put_uint8_t(buf, 56, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
#else
    mavlink_adum_gps_pvt_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.heading_motion = heading_motion;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.pdop = pdop;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_GPS_PVT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
#endif
}

/**
 * @brief Pack a adum_gps_pvt message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] UTC time since the UNIX epoch (1970-01-01 00:00:00 UTC) of this solution.
 *   Valid only if ADUM_GPS_PVT_FLAGS_TIME_VALID is set, otherwise 0.
 * @param lat [degE7] Latitude (WGS84), positive north.
 *   Degrees multiplied by 1e7.
 * @param lon [degE7] Longitude (WGS84), positive east.
 *   Degrees multiplied by 1e7.
 * @param alt_msl [mm] Altitude above mean sea level (geoid), positive up.
 *   Equals alt_ellipsoid minus the geoid separation at this position.
 * @param alt_ellipsoid [mm] Altitude above the WGS84 ellipsoid, positive up.
 * @param vel_n [mm/s] Velocity over ground, North component (positive north).
 * @param vel_e [mm/s] Velocity over ground, East component (positive east).
 * @param vel_d [mm/s] Velocity over ground, Down component (positive down, so a climb is negative).
 * @param heading_motion [degE5] Heading of motion (course over ground): direction of the velocity vector,
 *   0..359.99999 degrees clockwise from true North, multiplied by 1e5.
 *   This is NOT the vehicle's attitude heading.
 *   Valid only if ADUM_GPS_PVT_FLAGS_HEADING_VALID is set.
 * @param h_acc [mm] Horizontal position accuracy estimate, as reported by the GPS receiver.
 * @param v_acc [mm] Vertical position accuracy estimate, as reported by the GPS receiver.
 * @param s_acc [mm/s] Speed accuracy estimate, as reported by the GPS receiver.
 * @param pdop  Position dilution of precision, scaled by 100 (1.23 is sent as 123).
 *   UINT16_MAX if unknown.
 * @param fix_type  Fix type (GPS_FIX_TYPE). Values:
 *       0 NO_GPS    No GPS connected.
 *       1 NO_FIX    GPS connected, but no position information.
 *       2 2D_FIX    2D position (no usable altitude).
 *       3 3D_FIX    3D position, standalone.
 *       4 DGPS      3D position aided by DGPS / SBAS.
 *       5 RTK_FLOAT RTK solution with float carrier-phase ambiguities (decimetre-level).
 *       6 RTK_FIXED RTK solution with fixed carrier-phase ambiguities (centimetre-level).
 *       7 STATIC    Static fixed position, typically used on base stations.
 *       8 PPP       Precise Point Positioning, 3D position.
 *   Only 3 and above give a usable 3D position. ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS
 *   is set for 4, 5 and 6.
 * @param satellites_visible  Number of satellites used in the solution.
 *   UINT8_MAX if unknown.
 * @param flags  Status flags (ADUM_GPS_PVT_FLAGS bitmask):
 *       1 TIME_VALID          time_usec is valid.
 *       2 DIFF_CORRECTIONS    differential (RTK) corrections applied.
 *       4 HEADING_VALID       heading_motion is valid.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adum_gps_pvt_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,int32_t lat,int32_t lon,int32_t alt_msl,int32_t alt_ellipsoid,int32_t vel_n,int32_t vel_e,int32_t vel_d,int32_t heading_motion,uint32_t h_acc,uint32_t v_acc,uint32_t s_acc,uint16_t pdop,uint8_t fix_type,uint8_t satellites_visible,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt_msl);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_int32_t(buf, 24, vel_n);
    _mav_put_int32_t(buf, 28, vel_e);
    _mav_put_int32_t(buf, 32, vel_d);
    _mav_put_int32_t(buf, 36, heading_motion);
    _mav_put_uint32_t(buf, 40, h_acc);
    _mav_put_uint32_t(buf, 44, v_acc);
    _mav_put_uint32_t(buf, 48, s_acc);
    _mav_put_uint16_t(buf, 52, pdop);
    _mav_put_uint8_t(buf, 54, fix_type);
    _mav_put_uint8_t(buf, 55, satellites_visible);
    _mav_put_uint8_t(buf, 56, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
#else
    mavlink_adum_gps_pvt_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.heading_motion = heading_motion;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.pdop = pdop;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADUM_GPS_PVT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
}

/**
 * @brief Encode a adum_gps_pvt struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adum_gps_pvt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_gps_pvt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adum_gps_pvt_t* adum_gps_pvt)
{
    return mavlink_msg_adum_gps_pvt_pack(system_id, component_id, msg, adum_gps_pvt->time_usec, adum_gps_pvt->lat, adum_gps_pvt->lon, adum_gps_pvt->alt_msl, adum_gps_pvt->alt_ellipsoid, adum_gps_pvt->vel_n, adum_gps_pvt->vel_e, adum_gps_pvt->vel_d, adum_gps_pvt->heading_motion, adum_gps_pvt->h_acc, adum_gps_pvt->v_acc, adum_gps_pvt->s_acc, adum_gps_pvt->pdop, adum_gps_pvt->fix_type, adum_gps_pvt->satellites_visible, adum_gps_pvt->flags);
}

/**
 * @brief Encode a adum_gps_pvt struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adum_gps_pvt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_gps_pvt_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adum_gps_pvt_t* adum_gps_pvt)
{
    return mavlink_msg_adum_gps_pvt_pack_chan(system_id, component_id, chan, msg, adum_gps_pvt->time_usec, adum_gps_pvt->lat, adum_gps_pvt->lon, adum_gps_pvt->alt_msl, adum_gps_pvt->alt_ellipsoid, adum_gps_pvt->vel_n, adum_gps_pvt->vel_e, adum_gps_pvt->vel_d, adum_gps_pvt->heading_motion, adum_gps_pvt->h_acc, adum_gps_pvt->v_acc, adum_gps_pvt->s_acc, adum_gps_pvt->pdop, adum_gps_pvt->fix_type, adum_gps_pvt->satellites_visible, adum_gps_pvt->flags);
}

/**
 * @brief Encode a adum_gps_pvt struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param adum_gps_pvt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adum_gps_pvt_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_adum_gps_pvt_t* adum_gps_pvt)
{
    return mavlink_msg_adum_gps_pvt_pack_status(system_id, component_id, _status, msg,  adum_gps_pvt->time_usec, adum_gps_pvt->lat, adum_gps_pvt->lon, adum_gps_pvt->alt_msl, adum_gps_pvt->alt_ellipsoid, adum_gps_pvt->vel_n, adum_gps_pvt->vel_e, adum_gps_pvt->vel_d, adum_gps_pvt->heading_motion, adum_gps_pvt->h_acc, adum_gps_pvt->v_acc, adum_gps_pvt->s_acc, adum_gps_pvt->pdop, adum_gps_pvt->fix_type, adum_gps_pvt->satellites_visible, adum_gps_pvt->flags);
}

/**
 * @brief Send a adum_gps_pvt message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] UTC time since the UNIX epoch (1970-01-01 00:00:00 UTC) of this solution.
 *   Valid only if ADUM_GPS_PVT_FLAGS_TIME_VALID is set, otherwise 0.
 * @param lat [degE7] Latitude (WGS84), positive north.
 *   Degrees multiplied by 1e7.
 * @param lon [degE7] Longitude (WGS84), positive east.
 *   Degrees multiplied by 1e7.
 * @param alt_msl [mm] Altitude above mean sea level (geoid), positive up.
 *   Equals alt_ellipsoid minus the geoid separation at this position.
 * @param alt_ellipsoid [mm] Altitude above the WGS84 ellipsoid, positive up.
 * @param vel_n [mm/s] Velocity over ground, North component (positive north).
 * @param vel_e [mm/s] Velocity over ground, East component (positive east).
 * @param vel_d [mm/s] Velocity over ground, Down component (positive down, so a climb is negative).
 * @param heading_motion [degE5] Heading of motion (course over ground): direction of the velocity vector,
 *   0..359.99999 degrees clockwise from true North, multiplied by 1e5.
 *   This is NOT the vehicle's attitude heading.
 *   Valid only if ADUM_GPS_PVT_FLAGS_HEADING_VALID is set.
 * @param h_acc [mm] Horizontal position accuracy estimate, as reported by the GPS receiver.
 * @param v_acc [mm] Vertical position accuracy estimate, as reported by the GPS receiver.
 * @param s_acc [mm/s] Speed accuracy estimate, as reported by the GPS receiver.
 * @param pdop  Position dilution of precision, scaled by 100 (1.23 is sent as 123).
 *   UINT16_MAX if unknown.
 * @param fix_type  Fix type (GPS_FIX_TYPE). Values:
 *       0 NO_GPS    No GPS connected.
 *       1 NO_FIX    GPS connected, but no position information.
 *       2 2D_FIX    2D position (no usable altitude).
 *       3 3D_FIX    3D position, standalone.
 *       4 DGPS      3D position aided by DGPS / SBAS.
 *       5 RTK_FLOAT RTK solution with float carrier-phase ambiguities (decimetre-level).
 *       6 RTK_FIXED RTK solution with fixed carrier-phase ambiguities (centimetre-level).
 *       7 STATIC    Static fixed position, typically used on base stations.
 *       8 PPP       Precise Point Positioning, 3D position.
 *   Only 3 and above give a usable 3D position. ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS
 *   is set for 4, 5 and 6.
 * @param satellites_visible  Number of satellites used in the solution.
 *   UINT8_MAX if unknown.
 * @param flags  Status flags (ADUM_GPS_PVT_FLAGS bitmask):
 *       1 TIME_VALID          time_usec is valid.
 *       2 DIFF_CORRECTIONS    differential (RTK) corrections applied.
 *       4 HEADING_VALID       heading_motion is valid.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adum_gps_pvt_send(mavlink_channel_t chan, uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt_msl, int32_t alt_ellipsoid, int32_t vel_n, int32_t vel_e, int32_t vel_d, int32_t heading_motion, uint32_t h_acc, uint32_t v_acc, uint32_t s_acc, uint16_t pdop, uint8_t fix_type, uint8_t satellites_visible, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt_msl);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_int32_t(buf, 24, vel_n);
    _mav_put_int32_t(buf, 28, vel_e);
    _mav_put_int32_t(buf, 32, vel_d);
    _mav_put_int32_t(buf, 36, heading_motion);
    _mav_put_uint32_t(buf, 40, h_acc);
    _mav_put_uint32_t(buf, 44, v_acc);
    _mav_put_uint32_t(buf, 48, s_acc);
    _mav_put_uint16_t(buf, 52, pdop);
    _mav_put_uint8_t(buf, 54, fix_type);
    _mav_put_uint8_t(buf, 55, satellites_visible);
    _mav_put_uint8_t(buf, 56, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_GPS_PVT, buf, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
#else
    mavlink_adum_gps_pvt_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt_msl = alt_msl;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.heading_motion = heading_motion;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.pdop = pdop;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_GPS_PVT, (const char *)&packet, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
#endif
}

/**
 * @brief Send a adum_gps_pvt message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adum_gps_pvt_send_struct(mavlink_channel_t chan, const mavlink_adum_gps_pvt_t* adum_gps_pvt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adum_gps_pvt_send(chan, adum_gps_pvt->time_usec, adum_gps_pvt->lat, adum_gps_pvt->lon, adum_gps_pvt->alt_msl, adum_gps_pvt->alt_ellipsoid, adum_gps_pvt->vel_n, adum_gps_pvt->vel_e, adum_gps_pvt->vel_d, adum_gps_pvt->heading_motion, adum_gps_pvt->h_acc, adum_gps_pvt->v_acc, adum_gps_pvt->s_acc, adum_gps_pvt->pdop, adum_gps_pvt->fix_type, adum_gps_pvt->satellites_visible, adum_gps_pvt->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_GPS_PVT, (const char *)adum_gps_pvt, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adum_gps_pvt_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int32_t lat, int32_t lon, int32_t alt_msl, int32_t alt_ellipsoid, int32_t vel_n, int32_t vel_e, int32_t vel_d, int32_t heading_motion, uint32_t h_acc, uint32_t v_acc, uint32_t s_acc, uint16_t pdop, uint8_t fix_type, uint8_t satellites_visible, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt_msl);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_int32_t(buf, 24, vel_n);
    _mav_put_int32_t(buf, 28, vel_e);
    _mav_put_int32_t(buf, 32, vel_d);
    _mav_put_int32_t(buf, 36, heading_motion);
    _mav_put_uint32_t(buf, 40, h_acc);
    _mav_put_uint32_t(buf, 44, v_acc);
    _mav_put_uint32_t(buf, 48, s_acc);
    _mav_put_uint16_t(buf, 52, pdop);
    _mav_put_uint8_t(buf, 54, fix_type);
    _mav_put_uint8_t(buf, 55, satellites_visible);
    _mav_put_uint8_t(buf, 56, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_GPS_PVT, buf, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
#else
    mavlink_adum_gps_pvt_t *packet = (mavlink_adum_gps_pvt_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt_msl = alt_msl;
    packet->alt_ellipsoid = alt_ellipsoid;
    packet->vel_n = vel_n;
    packet->vel_e = vel_e;
    packet->vel_d = vel_d;
    packet->heading_motion = heading_motion;
    packet->h_acc = h_acc;
    packet->v_acc = v_acc;
    packet->s_acc = s_acc;
    packet->pdop = pdop;
    packet->fix_type = fix_type;
    packet->satellites_visible = satellites_visible;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADUM_GPS_PVT, (const char *)packet, MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN, MAVLINK_MSG_ID_ADUM_GPS_PVT_CRC);
#endif
}
#endif

#endif

// MESSAGE ADUM_GPS_PVT UNPACKING


/**
 * @brief Get field time_usec from adum_gps_pvt message
 *
 * @return [us] UTC time since the UNIX epoch (1970-01-01 00:00:00 UTC) of this solution.
 *   Valid only if ADUM_GPS_PVT_FLAGS_TIME_VALID is set, otherwise 0.
 */
static inline uint64_t mavlink_msg_adum_gps_pvt_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from adum_gps_pvt message
 *
 * @return [degE7] Latitude (WGS84), positive north.
 *   Degrees multiplied by 1e7.
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from adum_gps_pvt message
 *
 * @return [degE7] Longitude (WGS84), positive east.
 *   Degrees multiplied by 1e7.
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt_msl from adum_gps_pvt message
 *
 * @return [mm] Altitude above mean sea level (geoid), positive up.
 *   Equals alt_ellipsoid minus the geoid separation at this position.
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_alt_msl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt_ellipsoid from adum_gps_pvt message
 *
 * @return [mm] Altitude above the WGS84 ellipsoid, positive up.
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_alt_ellipsoid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field vel_n from adum_gps_pvt message
 *
 * @return [mm/s] Velocity over ground, North component (positive north).
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_vel_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field vel_e from adum_gps_pvt message
 *
 * @return [mm/s] Velocity over ground, East component (positive east).
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_vel_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field vel_d from adum_gps_pvt message
 *
 * @return [mm/s] Velocity over ground, Down component (positive down, so a climb is negative).
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_vel_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field heading_motion from adum_gps_pvt message
 *
 * @return [degE5] Heading of motion (course over ground): direction of the velocity vector,
 *   0..359.99999 degrees clockwise from true North, multiplied by 1e5.
 *   This is NOT the vehicle's attitude heading.
 *   Valid only if ADUM_GPS_PVT_FLAGS_HEADING_VALID is set.
 */
static inline int32_t mavlink_msg_adum_gps_pvt_get_heading_motion(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field h_acc from adum_gps_pvt message
 *
 * @return [mm] Horizontal position accuracy estimate, as reported by the GPS receiver.
 */
static inline uint32_t mavlink_msg_adum_gps_pvt_get_h_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Get field v_acc from adum_gps_pvt message
 *
 * @return [mm] Vertical position accuracy estimate, as reported by the GPS receiver.
 */
static inline uint32_t mavlink_msg_adum_gps_pvt_get_v_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  44);
}

/**
 * @brief Get field s_acc from adum_gps_pvt message
 *
 * @return [mm/s] Speed accuracy estimate, as reported by the GPS receiver.
 */
static inline uint32_t mavlink_msg_adum_gps_pvt_get_s_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  48);
}

/**
 * @brief Get field pdop from adum_gps_pvt message
 *
 * @return  Position dilution of precision, scaled by 100 (1.23 is sent as 123).
 *   UINT16_MAX if unknown.
 */
static inline uint16_t mavlink_msg_adum_gps_pvt_get_pdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  52);
}

/**
 * @brief Get field fix_type from adum_gps_pvt message
 *
 * @return  Fix type (GPS_FIX_TYPE). Values:
 *       0 NO_GPS    No GPS connected.
 *       1 NO_FIX    GPS connected, but no position information.
 *       2 2D_FIX    2D position (no usable altitude).
 *       3 3D_FIX    3D position, standalone.
 *       4 DGPS      3D position aided by DGPS / SBAS.
 *       5 RTK_FLOAT RTK solution with float carrier-phase ambiguities (decimetre-level).
 *       6 RTK_FIXED RTK solution with fixed carrier-phase ambiguities (centimetre-level).
 *       7 STATIC    Static fixed position, typically used on base stations.
 *       8 PPP       Precise Point Positioning, 3D position.
 *   Only 3 and above give a usable 3D position. ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS
 *   is set for 4, 5 and 6.
 */
static inline uint8_t mavlink_msg_adum_gps_pvt_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field satellites_visible from adum_gps_pvt message
 *
 * @return  Number of satellites used in the solution.
 *   UINT8_MAX if unknown.
 */
static inline uint8_t mavlink_msg_adum_gps_pvt_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Get field flags from adum_gps_pvt message
 *
 * @return  Status flags (ADUM_GPS_PVT_FLAGS bitmask):
 *       1 TIME_VALID          time_usec is valid.
 *       2 DIFF_CORRECTIONS    differential (RTK) corrections applied.
 *       4 HEADING_VALID       heading_motion is valid.
 */
static inline uint8_t mavlink_msg_adum_gps_pvt_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Decode a adum_gps_pvt message into a struct
 *
 * @param msg The message to decode
 * @param adum_gps_pvt C-struct to decode the message contents into
 */
static inline void mavlink_msg_adum_gps_pvt_decode(const mavlink_message_t* msg, mavlink_adum_gps_pvt_t* adum_gps_pvt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    adum_gps_pvt->time_usec = mavlink_msg_adum_gps_pvt_get_time_usec(msg);
    adum_gps_pvt->lat = mavlink_msg_adum_gps_pvt_get_lat(msg);
    adum_gps_pvt->lon = mavlink_msg_adum_gps_pvt_get_lon(msg);
    adum_gps_pvt->alt_msl = mavlink_msg_adum_gps_pvt_get_alt_msl(msg);
    adum_gps_pvt->alt_ellipsoid = mavlink_msg_adum_gps_pvt_get_alt_ellipsoid(msg);
    adum_gps_pvt->vel_n = mavlink_msg_adum_gps_pvt_get_vel_n(msg);
    adum_gps_pvt->vel_e = mavlink_msg_adum_gps_pvt_get_vel_e(msg);
    adum_gps_pvt->vel_d = mavlink_msg_adum_gps_pvt_get_vel_d(msg);
    adum_gps_pvt->heading_motion = mavlink_msg_adum_gps_pvt_get_heading_motion(msg);
    adum_gps_pvt->h_acc = mavlink_msg_adum_gps_pvt_get_h_acc(msg);
    adum_gps_pvt->v_acc = mavlink_msg_adum_gps_pvt_get_v_acc(msg);
    adum_gps_pvt->s_acc = mavlink_msg_adum_gps_pvt_get_s_acc(msg);
    adum_gps_pvt->pdop = mavlink_msg_adum_gps_pvt_get_pdop(msg);
    adum_gps_pvt->fix_type = mavlink_msg_adum_gps_pvt_get_fix_type(msg);
    adum_gps_pvt->satellites_visible = mavlink_msg_adum_gps_pvt_get_satellites_visible(msg);
    adum_gps_pvt->flags = mavlink_msg_adum_gps_pvt_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN? msg->len : MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN;
        memset(adum_gps_pvt, 0, MAVLINK_MSG_ID_ADUM_GPS_PVT_LEN);
    memcpy(adum_gps_pvt, _MAV_PAYLOAD(msg), len);
#endif
}
