/** @file
 *	@brief MAVLink comm protocol testsuite generated from ualberta.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef UALBERTA_TESTSUITE_H
#define UALBERTA_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ualberta(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_ualberta(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_nav_filter_bias(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_nav_filter_bias_t packet_in = {
		93372036854775807ULL,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	}185.0,
	}213.0,
	};
	mavlink_nav_filter_bias_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.accel_0 = packet_in.accel_0;
        	packet1.accel_1 = packet_in.accel_1;
        	packet1.accel_2 = packet_in.accel_2;
        	packet1.gyro_0 = packet_in.gyro_0;
        	packet1.gyro_1 = packet_in.gyro_1;
        	packet1.gyro_2 = packet_in.gyro_2;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_nav_filter_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_pack(system_id, component_id, &msg , packet1.usec , packet1.accel_0 , packet1.accel_1 , packet1.accel_2 , packet1.gyro_0 , packet1.gyro_1 , packet1.gyro_2 );
	mavlink_msg_nav_filter_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.accel_0 , packet1.accel_1 , packet1.accel_2 , packet1.gyro_0 , packet1.gyro_1 , packet1.gyro_2 );
	mavlink_msg_nav_filter_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_nav_filter_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_filter_bias_send(MAVLINK_COMM_1 , packet1.usec , packet1.accel_0 , packet1.accel_1 , packet1.accel_2 , packet1.gyro_0 , packet1.gyro_1 , packet1.gyro_2 );
	mavlink_msg_nav_filter_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radio_calibration(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radio_calibration_t packet_in = {
		{ 17235, 17236, 17237 },
	}{ 17547, 17548, 17549 },
	}{ 17859, 17860, 17861 },
	}{ 18171, 18172 },
	}{ 18379, 18380, 18381, 18382, 18383 },
	}{ 18899, 18900, 18901, 18902, 18903 },
	};
	mavlink_radio_calibration_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.aileron, packet_in.aileron, sizeof(uint16_t)*3);
        	mav_array_memcpy(packet1.elevator, packet_in.elevator, sizeof(uint16_t)*3);
        	mav_array_memcpy(packet1.rudder, packet_in.rudder, sizeof(uint16_t)*3);
        	mav_array_memcpy(packet1.gyro, packet_in.gyro, sizeof(uint16_t)*2);
        	mav_array_memcpy(packet1.pitch, packet_in.pitch, sizeof(uint16_t)*5);
        	mav_array_memcpy(packet1.throttle, packet_in.throttle, sizeof(uint16_t)*5);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radio_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_pack(system_id, component_id, &msg , packet1.aileron , packet1.elevator , packet1.rudder , packet1.gyro , packet1.pitch , packet1.throttle );
	mavlink_msg_radio_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.aileron , packet1.elevator , packet1.rudder , packet1.gyro , packet1.pitch , packet1.throttle );
	mavlink_msg_radio_calibration_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radio_calibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_calibration_send(MAVLINK_COMM_1 , packet1.aileron , packet1.elevator , packet1.rudder , packet1.gyro , packet1.pitch , packet1.throttle );
	mavlink_msg_radio_calibration_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_sys_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_sys_status_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}17859,
	}17963,
	}53,
	}120,
	}187,
	}254,
	}65,
	}132,
	};
	mavlink_ualberta_sys_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.collective = packet_in.collective;
        	packet1.receiver_voltage = packet_in.receiver_voltage;
        	packet1.avionics_voltage = packet_in.avionics_voltage;
        	packet1.engine_rpm = packet_in.engine_rpm;
        	packet1.rotor_rpm = packet_in.rotor_rpm;
        	packet1.mode = packet_in.mode;
        	packet1.gx3_mode = packet_in.gx3_mode;
        	packet1.pilot_mode = packet_in.pilot_mode;
        	packet1.control_mode = packet_in.control_mode;
        	packet1.attitude_source = packet_in.attitude_source;
        	packet1.trajectory = packet_in.trajectory;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_pack(system_id, component_id, &msg , packet1.mode , packet1.gx3_mode , packet1.pilot_mode , packet1.control_mode , packet1.attitude_source , packet1.engine_rpm , packet1.rotor_rpm , packet1.collective , packet1.receiver_voltage , packet1.avionics_voltage , packet1.trajectory );
	mavlink_msg_ualberta_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode , packet1.gx3_mode , packet1.pilot_mode , packet1.control_mode , packet1.attitude_source , packet1.engine_rpm , packet1.rotor_rpm , packet1.collective , packet1.receiver_voltage , packet1.avionics_voltage , packet1.trajectory );
	mavlink_msg_ualberta_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_sys_status_send(MAVLINK_COMM_1 , packet1.mode , packet1.gx3_mode , packet1.pilot_mode , packet1.control_mode , packet1.attitude_source , packet1.engine_rpm , packet1.rotor_rpm , packet1.collective , packet1.receiver_voltage , packet1.avionics_voltage , packet1.trajectory );
	mavlink_msg_ualberta_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_novatel_gps_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_novatel_gps_raw_t packet_in = {
		{ 17.0, 18.0, 19.0 },
	}{ 101.0, 102.0, 103.0 },
	}963498712,
	}89,
	}156,
	}223,
	}34,
	};
	mavlink_novatel_gps_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.pos_type = packet_in.pos_type;
        	packet1.pos_status = packet_in.pos_status;
        	packet1.num_sats = packet_in.num_sats;
        	packet1.vel_type = packet_in.vel_type;
        
        	mav_array_memcpy(packet1.pos_error, packet_in.pos_error, sizeof(float)*3);
        	mav_array_memcpy(packet1.vel_error, packet_in.vel_error, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_novatel_gps_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_novatel_gps_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_novatel_gps_raw_pack(system_id, component_id, &msg , packet1.pos_type , packet1.pos_status , packet1.num_sats , packet1.pos_error , packet1.vel_type , packet1.vel_error , packet1.time_boot_ms );
	mavlink_msg_novatel_gps_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_novatel_gps_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_type , packet1.pos_status , packet1.num_sats , packet1.pos_error , packet1.vel_type , packet1.vel_error , packet1.time_boot_ms );
	mavlink_msg_novatel_gps_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_novatel_gps_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_novatel_gps_raw_send(MAVLINK_COMM_1 , packet1.pos_type , packet1.pos_status , packet1.num_sats , packet1.pos_error , packet1.vel_type , packet1.vel_error , packet1.time_boot_ms );
	mavlink_msg_novatel_gps_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_position_t packet_in = {
		{ 17.0, 18.0, 19.0 },
	}{ 101.0, 102.0, 103.0 },
	}{ 185.0, 186.0, 187.0 },
	}{ 269.0, 270.0, 271.0 },
	}{ 353.0, 354.0, 355.0 },
	}{ 437.0, 438.0, 439.0 },
	}{ 521.0, 522.0, 523.0 },
	}963501832,
	};
	mavlink_ualberta_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        
        	mav_array_memcpy(packet1.llh_pos, packet_in.llh_pos, sizeof(float)*3);
        	mav_array_memcpy(packet1.ned_pos, packet_in.ned_pos, sizeof(float)*3);
        	mav_array_memcpy(packet1.ned_vel, packet_in.ned_vel, sizeof(float)*3);
        	mav_array_memcpy(packet1.ned_origin, packet_in.ned_origin, sizeof(float)*3);
        	mav_array_memcpy(packet1.reference_position, packet_in.reference_position, sizeof(float)*3);
        	mav_array_memcpy(packet1.position_error_body, packet_in.position_error_body, sizeof(float)*3);
        	mav_array_memcpy(packet1.position_error_ned, packet_in.position_error_ned, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_position_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_position_pack(system_id, component_id, &msg , packet1.llh_pos , packet1.ned_pos , packet1.ned_vel , packet1.ned_origin , packet1.reference_position , packet1.position_error_body , packet1.position_error_ned , packet1.time_boot_ms );
	mavlink_msg_ualberta_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.llh_pos , packet1.ned_pos , packet1.ned_vel , packet1.ned_origin , packet1.reference_position , packet1.position_error_body , packet1.position_error_ned , packet1.time_boot_ms );
	mavlink_msg_ualberta_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_position_send(MAVLINK_COMM_1 , packet1.llh_pos , packet1.ned_pos , packet1.ned_vel , packet1.ned_origin , packet1.reference_position , packet1.position_error_body , packet1.position_error_ned , packet1.time_boot_ms );
	mavlink_msg_ualberta_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_gx3_message(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_gx3_message_t packet_in = {
		"ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW",
	};
	mavlink_ualberta_gx3_message_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.message, packet_in.message, sizeof(char)*50);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_gx3_message_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_gx3_message_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_gx3_message_pack(system_id, component_id, &msg , packet1.message );
	mavlink_msg_ualberta_gx3_message_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_gx3_message_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.message );
	mavlink_msg_ualberta_gx3_message_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_gx3_message_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_gx3_message_send(MAVLINK_COMM_1 , packet1.message );
	mavlink_msg_ualberta_gx3_message_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_action(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_action_t packet_in = {
		5,
	}72,
	};
	mavlink_ualberta_action_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.action = packet_in.action;
        	packet1.param = packet_in.param;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_action_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_action_pack(system_id, component_id, &msg , packet1.action , packet1.param );
	mavlink_msg_ualberta_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_action_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.action , packet1.param );
	mavlink_msg_ualberta_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_action_send(MAVLINK_COMM_1 , packet1.action , packet1.param );
	mavlink_msg_ualberta_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_attitude_t packet_in = {
		{ 17.0, 18.0, 19.0 },
	}{ 101.0, 102.0, 103.0 },
	}{ 185.0, 186.0, 187.0 },
	}{ 269.0, 270.0, 271.0 },
	}{ 353.0, 354.0 },
	}963500376,
	};
	mavlink_ualberta_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        
        	mav_array_memcpy(packet1.nav_euler, packet_in.nav_euler, sizeof(float)*3);
        	mav_array_memcpy(packet1.nav_euler_rate, packet_in.nav_euler_rate, sizeof(float)*3);
        	mav_array_memcpy(packet1.ahrs_euler, packet_in.ahrs_euler, sizeof(float)*3);
        	mav_array_memcpy(packet1.ahrs_euler_rate, packet_in.ahrs_euler_rate, sizeof(float)*3);
        	mav_array_memcpy(packet1.attitude_reference, packet_in.attitude_reference, sizeof(float)*2);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_attitude_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_attitude_pack(system_id, component_id, &msg , packet1.nav_euler , packet1.nav_euler_rate , packet1.ahrs_euler , packet1.ahrs_euler_rate , packet1.attitude_reference , packet1.time_boot_ms );
	mavlink_msg_ualberta_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nav_euler , packet1.nav_euler_rate , packet1.ahrs_euler , packet1.ahrs_euler_rate , packet1.attitude_reference , packet1.time_boot_ms );
	mavlink_msg_ualberta_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_attitude_send(MAVLINK_COMM_1 , packet1.nav_euler , packet1.nav_euler_rate , packet1.ahrs_euler , packet1.ahrs_euler_rate , packet1.attitude_reference , packet1.time_boot_ms );
	mavlink_msg_ualberta_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_control_effort(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_control_effort_t packet_in = {
		{ 17.0, 18.0, 19.0, 20.0, 21.0, 22.0 },
	};
	mavlink_ualberta_control_effort_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.normalized_control_effort, packet_in.normalized_control_effort, sizeof(float)*6);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_control_effort_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_control_effort_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_control_effort_pack(system_id, component_id, &msg , packet1.normalized_control_effort );
	mavlink_msg_ualberta_control_effort_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_control_effort_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.normalized_control_effort );
	mavlink_msg_ualberta_control_effort_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_control_effort_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_control_effort_send(MAVLINK_COMM_1 , packet1.normalized_control_effort );
	mavlink_msg_ualberta_control_effort_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta_altimiter(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ualberta_altimiter_t packet_in = {
		17.0,
	};
	mavlink_ualberta_altimiter_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.dist = packet_in.dist;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_altimiter_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ualberta_altimiter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_altimiter_pack(system_id, component_id, &msg , packet1.dist );
	mavlink_msg_ualberta_altimiter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_altimiter_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dist );
	mavlink_msg_ualberta_altimiter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ualberta_altimiter_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ualberta_altimiter_send(MAVLINK_COMM_1 , packet1.dist );
	mavlink_msg_ualberta_altimiter_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_udenver_cpu_usage(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_udenver_cpu_usage_t packet_in = {
		17.0,
	};
	mavlink_udenver_cpu_usage_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.cpu_usage = packet_in.cpu_usage;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_udenver_cpu_usage_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_udenver_cpu_usage_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_udenver_cpu_usage_pack(system_id, component_id, &msg , packet1.cpu_usage );
	mavlink_msg_udenver_cpu_usage_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_udenver_cpu_usage_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cpu_usage );
	mavlink_msg_udenver_cpu_usage_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_udenver_cpu_usage_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_udenver_cpu_usage_send(MAVLINK_COMM_1 , packet1.cpu_usage );
	mavlink_msg_udenver_cpu_usage_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ualberta(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_nav_filter_bias(system_id, component_id, last_msg);
	mavlink_test_radio_calibration(system_id, component_id, last_msg);
	mavlink_test_ualberta_sys_status(system_id, component_id, last_msg);
	mavlink_test_novatel_gps_raw(system_id, component_id, last_msg);
	mavlink_test_ualberta_position(system_id, component_id, last_msg);
	mavlink_test_ualberta_gx3_message(system_id, component_id, last_msg);
	mavlink_test_ualberta_action(system_id, component_id, last_msg);
	mavlink_test_ualberta_attitude(system_id, component_id, last_msg);
	mavlink_test_ualberta_control_effort(system_id, component_id, last_msg);
	mavlink_test_ualberta_altimiter(system_id, component_id, last_msg);
	mavlink_test_udenver_cpu_usage(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UALBERTA_TESTSUITE_H
