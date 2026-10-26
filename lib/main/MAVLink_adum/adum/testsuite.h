/** @file
 *    @brief MAVLink comm protocol testsuite generated from adum.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ADUM_TESTSUITE_H
#define ADUM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_storm32(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_adum(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_storm32(system_id, component_id, last_msg);
    mavlink_test_adum(system_id, component_id, last_msg);
}
#endif

#include "../storm32/testsuite.h"


static void mavlink_test_adum_gps_pvt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADUM_GPS_PVT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adum_gps_pvt_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,963498920,963499128,963499336,963499544,963499752,963499960,19939,39,106,173
    };
    mavlink_adum_gps_pvt_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt_msl = packet_in.alt_msl;
        packet1.alt_ellipsoid = packet_in.alt_ellipsoid;
        packet1.vel_n = packet_in.vel_n;
        packet1.vel_e = packet_in.vel_e;
        packet1.vel_d = packet_in.vel_d;
        packet1.heading_motion = packet_in.heading_motion;
        packet1.h_acc = packet_in.h_acc;
        packet1.v_acc = packet_in.v_acc;
        packet1.s_acc = packet_in.s_acc;
        packet1.pdop = packet_in.pdop;
        packet1.fix_type = packet_in.fix_type;
        packet1.satellites_visible = packet_in.satellites_visible;
        packet1.flags = packet_in.flags;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADUM_GPS_PVT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_gps_pvt_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adum_gps_pvt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_gps_pvt_pack(system_id, component_id, &msg , packet1.time_usec , packet1.lat , packet1.lon , packet1.alt_msl , packet1.alt_ellipsoid , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.heading_motion , packet1.h_acc , packet1.v_acc , packet1.s_acc , packet1.pdop , packet1.fix_type , packet1.satellites_visible , packet1.flags );
    mavlink_msg_adum_gps_pvt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_gps_pvt_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.lat , packet1.lon , packet1.alt_msl , packet1.alt_ellipsoid , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.heading_motion , packet1.h_acc , packet1.v_acc , packet1.s_acc , packet1.pdop , packet1.fix_type , packet1.satellites_visible , packet1.flags );
    mavlink_msg_adum_gps_pvt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adum_gps_pvt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_gps_pvt_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.lat , packet1.lon , packet1.alt_msl , packet1.alt_ellipsoid , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.heading_motion , packet1.h_acc , packet1.v_acc , packet1.s_acc , packet1.pdop , packet1.fix_type , packet1.satellites_visible , packet1.flags );
    mavlink_msg_adum_gps_pvt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ADUM_GPS_PVT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ADUM_GPS_PVT) != NULL);
#endif
}

static void mavlink_test_adum_magnetometer(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADUM_MAGNETOMETER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adum_magnetometer_t packet_in = {
        963497464,17443,17547,17651,163
    };
    mavlink_adum_magnetometer_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_ms = packet_in.time_ms;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        packet1.instance = packet_in.instance;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADUM_MAGNETOMETER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_magnetometer_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adum_magnetometer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_magnetometer_pack(system_id, component_id, &msg , packet1.instance , packet1.time_ms , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_adum_magnetometer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_magnetometer_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.instance , packet1.time_ms , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_adum_magnetometer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adum_magnetometer_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_magnetometer_send(MAVLINK_COMM_1 , packet1.instance , packet1.time_ms , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_adum_magnetometer_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ADUM_MAGNETOMETER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ADUM_MAGNETOMETER) != NULL);
#endif
}

static void mavlink_test_adum_aux_channels(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADUM_AUX_CHANNELS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adum_aux_channels_t packet_in = {
        { 17235, 17236, 17237 },{ 17547, 17548, 17549, 17550, 17551, 17552, 17553, 17554 },199
    };
    mavlink_adum_aux_channels_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.arming_status = packet_in.arming_status;

        mav_array_memcpy(packet1.motor, packet_in.motor, sizeof(uint16_t)*3);
        mav_array_memcpy(packet1.servo, packet_in.servo, sizeof(uint16_t)*8);

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_aux_channels_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adum_aux_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_aux_channels_pack(system_id, component_id, &msg , packet1.arming_status , packet1.motor , packet1.servo );
    mavlink_msg_adum_aux_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_aux_channels_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.arming_status , packet1.motor , packet1.servo );
    mavlink_msg_adum_aux_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adum_aux_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adum_aux_channels_send(MAVLINK_COMM_1 , packet1.arming_status , packet1.motor , packet1.servo );
    mavlink_msg_adum_aux_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ADUM_AUX_CHANNELS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ADUM_AUX_CHANNELS) != NULL);
#endif
}

static void mavlink_test_adum(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_adum_gps_pvt(system_id, component_id, last_msg);
    mavlink_test_adum_magnetometer(system_id, component_id, last_msg);
    mavlink_test_adum_aux_channels(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ADUM_TESTSUITE_H
