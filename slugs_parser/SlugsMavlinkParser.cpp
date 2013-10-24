/*
* File:   SlugsMavlinkParser.cpp
* Author: dagoodman@soe.ucsc.edu
*
* The SLUGS MAVlink parser parses Simulink UDP streams into MAVlink messages and
* visa versa.  This tool is used for HIL simulation with simulink.
*
* Created on July 26, 2013 
*/
//#define DEBUG
//#ifdef DEBUG
#include <iostream>
#include <string>
using namespace std;
//#endif

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <vector>
#include "Packing.h"
#include "SlugsMavlinkParser.h"

// Outgoing buffer sizes determined by the longest message.
#define PWM_COMMAND_BUFFER_SIZE		20
#define MAVLINK_SERIAL_BUFFER_SIZE	100

// Byte offset values for starting location of data in Simulink UDP packets
#define HIL_GPS_START			6 // skips date time fields
#define HIL_GPS_DATE_TIME_START	0
#define HIL_AIR_START			28
#define HIL_RAW_START			38
#define HIL_RAW_AIR_START		56
#define HIL_ATTITUDE_START      62
#define HIL_XYZ_START			90	

// List of offsets indexed by HilMessageType
unsigned char udp_data_offset[] = {HIL_GPS_START, HIL_GPS_DATE_TIME_START, HIL_AIR_START, HIL_RAW_START, HIL_RAW_AIR_START,
	HIL_ATTITUDE_START, HIL_XYZ_START};

// Order to send HIL messages for round robin (attitude or position every other are packed with each)
static const HilMessageType hil_message_order[] = {HIL_GPS, HIL_GPS_DATE_TIME, HIL_AIR, HIL_RAW, HIL_RAW_AIR};

unsigned int hil_list_length =  sizeof(hil_message_order) / sizeof(hil_message_order[0]);


/******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * @brief Constructor for SLUGS MAVLink parser.
 */
SlugsMavlinkParser::SlugsMavlinkParser(int autopilot_system_id, int autopilot_comp_id, int gs_system_id, int gs_comp_id) {
	this->autopilot_system_id = autopilot_system_id;
	this->autopilot_comp_id = autopilot_comp_id;
	this->gs_system_id = gs_system_id;
	this->gs_comp_id = gs_comp_id;
	is_ok = true;
	round_robin_index = 0;

	mlAttitudeData.time_boot_ms = 0;
#ifdef DEBUG
	cout << "Slugs parser initialized in DEBUG mode!" << endl;
#endif
}

/**
 * @return True or false whether the parser is working.
 * @brief Used to check if the parser is working.
 */
bool SlugsMavlinkParser::isOk() {
	return is_ok;
}

/**
 * @param Buffer with raw UDP data.
 * @param Number of bytes in buffer.
 * @return A tuple, were the first element is the new buffer, and second is the new size.
 * @brief Processes a datagram packet from Simulink by parsing it into
 *  MAVLink messages to be sent to the Autopilot via serial.
 * @note This does not read all data from the udp packet every time. Instead, a round
 *	robin approach is used to read, parse, and return a different message each time.
 * @note The assemble_mavlink_message() helper function will append either a position
 *	or an attitude message (toggles between the two on each call) to the round robin message.
 *  
 */
boost::tuple<uint8_t*, size_t> SlugsMavlinkParser::parse_udp2serial(uint8_t *buf, size_t bytes) {
	// Persistent variable to toggle sending of either attitude or position along with message
	static bool sent_attitude_last = false;

	// Choose message type to send and increment index
	HilMessageType message_type = hil_message_order[round_robin_index++];
	if (round_robin_index >= hil_list_length) round_robin_index = 0;

	// Create a new buffer for two messages
	uint8_t *newBuf = new uint8_t[MAVLINK_SERIAL_BUFFER_SIZE * 2];
	size_t newLen = 0;

	// Returns a new buffer with the specified mavlink message filled and packed into it
	switch(message_type) {
		case HIL_GPS:
			newLen = assemble_mavlink_message(buf, newBuf, HIL_GPS);
			break;
		case HIL_GPS_DATE_TIME:
			newLen = assemble_mavlink_message(buf, newBuf, HIL_GPS_DATE_TIME);
			break;
		case HIL_AIR:
			newLen = assemble_mavlink_message(buf, newBuf, HIL_AIR);
			break;
		case HIL_RAW:
			newLen = assemble_mavlink_message(buf, newBuf, HIL_RAW);
			break;
		case HIL_RAW_AIR:
			newLen = assemble_mavlink_message(buf, newBuf, HIL_RAW_AIR);
			break;
		default:
			throw "Bad round robin type!";
			break;
	}

	// Pack attitude of position onto the end
	if (sent_attitude_last)  {
		newLen += assemble_mavlink_message(buf, &newBuf[newLen], HIL_ATTITUDE);
	}
	else {
		newLen += assemble_mavlink_message(buf, &newBuf[newLen], HIL_XYZ);
	}

	sent_attitude_last ^= 1;

    return boost::tuple<uint8_t*, size_t>(newBuf, newLen);
}

/**
 * @param Buffer with raw serial data.
 * @param Size of data in buffer.
 * @return A tuple, were the first element is the new buffer, and second is the new size.
 * @brief Processes mavlink serial messages from the Autopilot by parsing it into
 *  a datagram sequence to be sent to Simulink over UDP.
 */
boost::tuple<uint8_t*, size_t> SlugsMavlinkParser::parse_serial2udp(uint8_t *buf, size_t bytes) {
    mavlink_message_t msg;
    mavlink_status_t status;

    for (uint32_t i =0; i < bytes; i++) {
        if (mavlink_parse_char(0, buf[i], &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                    mavlink_msg_servo_output_raw_decode(&msg, &mlPwmCommands);
                    return this->assemble_udp_pwm_command();
            }
        }
    }

	// Unhandled message, return as is ?
    return boost::tuple<uint8_t*, size_t>(buf, bytes);
}

/******************************************************************************
 * PRIVATE FUNCTIONS                                                          *
 ******************************************************************************/


/*
 * @return A tuple, were the first element is the new buffer with PWM commands packed into it,
 *	 and second is the size of the buffer.
 * @return A buffer sequence with PWM commands packed into UDP.
 * @brief Packs PWM commands into a UDP packet.
 * @note Endianness is currently not checked. Little endian is used below,
 *	but the packing library also has big endian support (BE prefix).
 */
boost::tuple<uint8_t*, size_t> SlugsMavlinkParser::assemble_udp_pwm_command() {
    uint8_t *send_buffer = new uint8_t[PWM_COMMAND_BUFFER_SIZE];

    // Find timestamp in microseconds
	uint32_t timeStampUsec = mlAttitudeData.time_boot_ms;
    
    mavlink_servo_output_raw_t pwmSampleLocal = mlPwmCommands;
    uint16_t rawServoData[8] = { pwmSampleLocal.servo1_raw, pwmSampleLocal.servo2_raw,
        pwmSampleLocal.servo3_raw, pwmSampleLocal.servo4_raw, pwmSampleLocal.servo5_raw,
        pwmSampleLocal.servo6_raw, pwmSampleLocal.servo7_raw, pwmSampleLocal.servo8_raw };
    
	// Iterate over PWM servo data and add these uint16_t types to the send buffer
    for (uint8_t i = 0; i < 8; i++) {
		LEPackUint16(&send_buffer[i*2], rawServoData[i]);
    }    
	
    // Add the timestamp to the end (send_buffer[0] to send_buffer[15] already filled)
	LEPackUint32(&send_buffer[16], timeStampUsec);

#ifdef DEBUG
	cout << "Unpacked PWM command UDP packet: " << pwmSampleLocal.servo1_raw << ", " <<  pwmSampleLocal.servo2_raw << ", " <<  pwmSampleLocal.servo3_raw << ", " <<  pwmSampleLocal.servo4_raw << ", " <<  pwmSampleLocal.servo5_raw << ", " <<  pwmSampleLocal.servo6_raw << ", " <<  pwmSampleLocal.servo7_raw << ", " <<  pwmSampleLocal.servo8_raw << endl;
#endif

    return boost::tuple<uint8_t*, size_t>(send_buffer, static_cast<size_t>(PWM_COMMAND_BUFFER_SIZE));
}


/*
 * @param Raw UDP message from simulink.
 * @param Type of MAVLink HIL message to send (see hil_message_order[] for ordered list.
 * @return The size of the messages that was packed into the buffer.
 * @brief Packs a mavlink message with the specified type into a buffer using raw UDP data.
 */
size_t SlugsMavlinkParser::assemble_mavlink_message(uint8_t* rawUdpData, uint8_t* buf, HilMessageType type) { 
	//uint8_t *msgBuf = new uint8_t[MAVLINK_SERIAL_BUFFER_SIZE];
	mavlink_message_t msg = {};
	size_t len = 0;

	// Offset index into the raw data buffer
	uint16_t i = udp_data_offset[type];

	switch (type) {
		case HIL_GPS:
		{
			mavlink_gps_raw_int_t mlGpsData = {};
			// Parse UDP into MAVLink
			mlGpsData.fix_type = rawUdpData[i];
			LEUnpackInt32(&mlGpsData.lat, &rawUdpData[i+1]); 
			LEUnpackInt32(&mlGpsData.lon, &rawUdpData[i+5]);
			LEUnpackInt32(&mlGpsData.alt, &rawUdpData[i+9]);
			LEUnpackUint16(&mlGpsData.cog, &rawUdpData[i+13]);
			LEUnpackUint16(&mlGpsData.vel, &rawUdpData[i+15]);
			LEUnpackUint16(&mlGpsData.eph, &rawUdpData[i+17]);
#ifdef DEBUG
			cout << "Packed GPS message: " << mlGpsData.lat << ", " <<  mlGpsData.lon <<", " <<  mlGpsData.alt <<", cog:" <<  mlGpsData.cog << ", vel:" <<  mlGpsData.vel << "." << endl; 
#endif
			// Encode MAVLink message
			mavlink_msg_gps_raw_int_encode(autopilot_system_id, autopilot_comp_id, &msg, &mlGpsData);
			break;
		}
		case HIL_GPS_DATE_TIME:
		{
			mavlink_gps_date_time_t mlGpsDateTime = {};
			// Parse UDP into MAVLink
			mlGpsDateTime.year = rawUdpData[i];
			mlGpsDateTime.month = rawUdpData[i+1];
			mlGpsDateTime.day = rawUdpData[i+2];
			mlGpsDateTime.hour = rawUdpData[i+3];
			mlGpsDateTime.min = rawUdpData[i+4];
			mlGpsDateTime.sec = rawUdpData[i+5];
			mlGpsDateTime.visSat = rawUdpData[i+24]; // skips ahead
			// Encode MAVLink message
#ifdef DEBUG
			printf("Packed GPS Date Time message: Date: %d,%d,%d, Time: %d,%d,%d\r\n", mlGpsDateTime.year, mlGpsDateTime.month, mlGpsDateTime.day, mlGpsDateTime.hour, mlGpsDateTime.min, mlGpsDateTime.sec);
#endif
			mavlink_msg_gps_date_time_encode(autopilot_system_id, autopilot_comp_id, &msg, &mlGpsDateTime);
			break;
		}
		case HIL_AIR:
		{
			mavlink_scaled_pressure_t mlAirData = {};
			// Parse UDP into MAVLink
			LEUnpackReal32(&mlAirData.press_diff, &rawUdpData[i]);
			LEUnpackReal32(&mlAirData.press_abs, &rawUdpData[i+4]);
			LEUnpackInt16(&mlAirData.temperature, &rawUdpData[i+8]);
			// Encode MAVLink message
			#ifdef DEBUG
			cout << "Packed Scaled Air Data message: " << mlAirData.press_diff << ", " <<  mlAirData.press_abs <<", " << mlAirData.temperature << "." << endl; 
			#endif
			mavlink_msg_scaled_pressure_encode(autopilot_system_id, autopilot_comp_id, &msg, &mlAirData);
			break;
		}
		case HIL_RAW:
		{
			mavlink_raw_imu_t mlRawData = {};
			// Parse UDP into MAVLink
			LEUnpackInt16(&mlRawData.xgyro, &rawUdpData[i]);
			LEUnpackInt16(&mlRawData.ygyro, &rawUdpData[i+2]);
			LEUnpackInt16(&mlRawData.zgyro, &rawUdpData[i+4]);
			LEUnpackInt16(&mlRawData.xacc, &rawUdpData[i+6]);
			LEUnpackInt16(&mlRawData.yacc, &rawUdpData[i+8]);
			LEUnpackInt16(&mlRawData.zacc, &rawUdpData[i+10]);
			LEUnpackInt16(&mlRawData.xmag, &rawUdpData[i+12]);
			LEUnpackInt16(&mlRawData.ymag, &rawUdpData[i+14]);
			LEUnpackInt16(&mlRawData.zmag, &rawUdpData[i+16]);
			// Encode MAVLink message
			#ifdef DEBUG
			cout << "Packed Raw IMU message: Gryo" << mlRawData.xgyro << "," <<  mlRawData.ygyro <<"," << mlRawData.zgyro << "." << endl; 
			#endif
			mavlink_msg_raw_imu_encode(autopilot_system_id, autopilot_comp_id, &msg, &mlRawData);
			break;
		}
		case HIL_RAW_AIR:
		{
			mavlink_raw_pressure_t mlRawPressure = {};
			// Parse UDP into MAVLink
			LEUnpackInt16(&mlRawPressure.press_diff1, &rawUdpData[i]);
			LEUnpackInt16(&mlRawPressure.press_abs, &rawUdpData[i+2]);
			LEUnpackInt16(&mlRawPressure.temperature, &rawUdpData[i+4]);
			// Encode MAVLink message
			#ifdef DEBUG
			cout << "Packed Raw Air Data message: " << mlRawPressure.press_diff1 << ", " <<  mlRawPressure.press_abs <<", " << mlRawPressure.temperature << "." << endl; 
			#endif

			mavlink_msg_raw_pressure_encode(autopilot_system_id, autopilot_comp_id, &msg, &mlRawPressure);
			
			break;
		}
		case HIL_ATTITUDE:
		{
			mavlink_attitude_t mlAttitude = {};
			// Parse UDP into MAVLink
			LEUnpackReal32(&mlAttitude.roll, &rawUdpData[i]);
			LEUnpackReal32(&mlAttitude.pitch, &rawUdpData[i+4]);
			LEUnpackReal32(&mlAttitude.yaw, &rawUdpData[i+8]);
	
			LEUnpackReal32(&mlAttitude.rollspeed, &rawUdpData[i+12]);
			LEUnpackReal32(&mlAttitude.pitchspeed, &rawUdpData[i+16]);
			LEUnpackReal32(&mlAttitude.yawspeed, &rawUdpData[i+20]);

			LEUnpackUint32(&mlAttitude.time_boot_ms, &rawUdpData[i+24]);


			// Encode MAVLink message
			#ifdef DEBUG
			cout << "Packed Attitude message: time=" << mlAttitude.time_boot_ms << "." << endl;
			#endif
			mavlink_msg_attitude_encode(autopilot_system_id, autopilot_comp_id, &msg, &mlAttitude);
			break;
		}
		case HIL_XYZ:
		{
			mavlink_local_position_ned_t mlLocalPosition = {};
			// Parse UDP into MAVLink
			LEUnpackReal32(&mlLocalPosition.x, &rawUdpData[i]);
			LEUnpackReal32(&mlLocalPosition.y, &rawUdpData[i+4]);
			LEUnpackReal32(&mlLocalPosition.z, &rawUdpData[i+8]);
			
			LEUnpackReal32(&mlLocalPosition.vx, &rawUdpData[i+12]);
			LEUnpackReal32(&mlLocalPosition.vy, &rawUdpData[i+16]);
			LEUnpackReal32(&mlLocalPosition.vz, &rawUdpData[i+20]);
			// Encode MAVLink message
			#ifdef DEBUG
			cout << "Packed XYZ message: " << mlLocalPosition.x << ", " <<  mlLocalPosition.y <<", " << mlLocalPosition.z << "." << endl; 
			#endif
			mavlink_msg_local_position_ned_encode(autopilot_system_id, autopilot_comp_id, &msg, &mlLocalPosition);
			break;

		}
	} // switch(type))
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	return len;
}
