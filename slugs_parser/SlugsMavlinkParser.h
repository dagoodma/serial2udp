/*
* @file     SlugsMavlinkParser.h
* @author   dagoodman@soe.ucsc.edu
*
* @brief
* Interface for SLUGS MAVLink parser
*
* @details
* This is the interface for SLUGS MAVlink parser, which parses Simulink UDP streams
* into MAVlink messages and visa versa.
*
* @date July 26, 2013   -- Created
*/
#ifndef SLUGS_MAVLINK_PARSER_H
#define SLUGS_MAVLINK_PARSER_H

#include <boost/tuple/tuple.hpp>
#include "common\mavlink.h"
#include "slugs\slugs.h"

/******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

class SlugsMavlinkParser {
		int autopilot_system_id, autopilot_comp_id, gs_system_id, gs_comp_id;
		mavlink_servo_output_raw_t mlPwmCommands;
		mavlink_attitude_t mlAttitudeData;
	public:
		SlugsMavlinkParser (int, int, int, int);
		boost::tuple<uint8_t*, size_t> parse_udp2serial(uint8_t*, size_t);
		boost::tuple<uint8_t*, size_t> parse_serial2udp(uint8_t*, size_t);
		bool isOk();
	private: 
		enum HilMessageType {HIL_GPS, HIL_GPS_DATE_TIME, HIL_AIR, HIL_RAW, HIL_RAW_AIR,
            HIL_ATTITUDE, HIL_XYZ, HIL_MSG_COUNT} round_robin_type;
		bool send_attitude;
		boost::tuple<uint8_t*, size_t> assemble_udp_pwm_command();
		boost::tuple<uint8_t*, size_t> assemble_mavlink_message(uint8_t*, HilMessageType);
		bool is_ok;
};


#endif
