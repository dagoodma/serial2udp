/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

// Required by Windows compilation to specify the version of Windows being targeted. In this case Windows XP.
#ifdef WIN32
#define _WIN32_WINNT 0x0501
#endif

#define PARSE_SLUGS_MAVLINK

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#ifdef PARSE_SLUGS_MAVLINK
#include <boost/tuple/tuple.hpp>
#include "slugsMavlinkParser.h"
#endif

// Set up some shorthand for namespaces
namespace po = boost::program_options;

// Default to the standard namespace
using namespace std;

// Initialized internal variables
boost::asio::ip::udp::socket *udpSocket;
boost::asio::serial_port *serialPort;

// Intermediate internal variables
boost::asio::ip::udp::endpoint remote_endpoint_tx;
boost::asio::ip::udp::endpoint remote_endpoint_rx;
char *udp_receive_buffer;
char *serial_receive_buffer;

// Function prototypes
void start_udp_receive();
void start_serial_receive();
void handle_udp_receive(const boost::system::error_code &error, size_t bytes_transferred);
void handle_udp_send(const boost::system::error_code &error, size_t bytes_transferred);
void handle_serial_receive(const boost::system::error_code &error, size_t bytes_transferred);
void handle_serial_send(const boost::system::error_code &error, size_t bytes_transferred);
void print_error(const string message);

// Keep track of all of the command-line options
int opt_baud_rate;
string opt_port;
int opt_remote_socket;
string opt_remote_rx_addr;
string opt_remote_tx_addr;
int opt_local_socket;
int opt_udp_rx_packet_size;
int opt_udp_tx_packet_size;
int opt_serial_packet_size;
// SLUGS parser specific options
#ifdef PARSE_SLUGS_MAVLINK
int opt_autopilot_system_id;
int opt_autopilot_comp_id;
int opt_gs_system_id;
int opt_gs_comp_id;
SlugsMavlinkParser *parser;
#endif

int main(int ac, char* av[])
{
	// Manage program startup.
	try {

		// Declare the supported options.
		po::options_description desc("Allowed options");
		desc.add_options()
			("help", "produce help message")
			("port", po::value<string>(&opt_port)->default_value("COM1"), "serial port to use")
			("baud_rate", po::value<int>(&opt_baud_rate)->default_value(115200), "set the serial port baud rate")
			("local_socket", po::value<int>(&opt_local_socket)->default_value(5679), "local UDP socket to receive on")
			("remote_socket", po::value<int>(&opt_remote_socket)->default_value(5678), "remote UDP socket to trasmit on")
			("remote_tx_addr", po::value<string>(&opt_remote_tx_addr)->default_value("255.255.255.255"), "remote IP address to receive from")
			("remote_rx_addr", po::value<string>(&opt_remote_rx_addr)->default_value("any"), "remote IP address to receive from")
			("udp_rx_packet_size", po::value<int>(&opt_udp_rx_packet_size)->default_value(113), "size of received UDP datagram in bytes (<= 128)")
			("udp_tx_packet_size", po::value<int>(&opt_udp_tx_packet_size)->default_value(20), "size of transmitted UDP datagram in bytes (<= 128)")
			("serial_packet_size", po::value<int>(&opt_serial_packet_size)->default_value(100), "size of the serial packets to be expected (<= 128)")
			#ifdef PARSE_SLUGS_MAVLINK
			("autopilot_sys_id", po::value<int>(&opt_autopilot_system_id)->default_value(100), "set autopilot system id for MAVLink (<=255)")
			("autopilot_comp_id", po::value<int>(&opt_autopilot_comp_id)->default_value(1), "set autopilot component id for MAVLink (<=255)")
			("gs_sys_id", po::value<int>(&opt_gs_system_id)->default_value(127), "set ground station system id for MAVLink (<=255)")
			("gs_comp_id", po::value<int>(&opt_gs_comp_id)->default_value(0), "set ground station component id for MAVLink (<=255)")
			#endif
		;

		// Now parse the user-specified options
		po::variables_map vm;
		po::store(po::parse_command_line(ac, av, desc), vm);
		po::notify(vm);

		// If the help menu was specified, spit out the help text and quit.
		if (vm.count("help")) {
			cout << desc << "\n";
			return 0;
		}

		udp_receive_buffer = new char[opt_udp_rx_packet_size];
		if (udp_receive_buffer == NULL) {
			cerr << "ERROR: Could not allocate enough space for a UDP receive buffer of that size." << endl;
			return 3;
		}
		
		serial_receive_buffer = new char[opt_serial_packet_size];
		if (serial_receive_buffer == NULL) {
			cerr << "ERROR: Could not allocate enough space for a serial receive buffer of that size." << endl;
			return 2;
		}

	} catch (exception& e) {
		cerr << e.what() << endl;
		return 1;
	}

	// Create a parser
	#ifdef PARSE_SLUGS_MAVLINK
	try {
		// Initialize SLUGS parser
		parser = new SlugsMavlinkParser(opt_autopilot_system_id, opt_autopilot_comp_id, opt_gs_system_id, opt_gs_comp_id);
		if (!parser->isOk()) {
			cerr << "ERROR: Failed to initialize the SLUGS parser." << endl;
			return 4;
		}
		cout << "Initialized SLUGS MAVLink parser." << endl;
	} catch (exception& e) {
		cerr << "Failed to construct parser: " << e.what() << endl;
		return 4;
	}
	#endif
	
	// Create and start the UDP socket.
	boost::asio::io_service io_service;
	try {
		// Parse addresses
		bool udp_receive_any = (opt_remote_rx_addr.compare("any") == 0)? true : false; 
		bool udp_transmit_broadcast = (opt_remote_tx_addr.compare("255.255.255.255") == 0)? true : false;
		boost::asio::ip::address_v4 remote_rx_addr = (udp_receive_any)?
			boost::asio::ip::address_v4::any() : 
			boost::asio::ip::address_v4::from_string(opt_remote_rx_addr);

		boost::asio::ip::address_v4 remote_tx_addr = (udp_transmit_broadcast)?
			boost::asio::ip::address_v4::broadcast() : 
			boost::asio::ip::address_v4::from_string(opt_remote_tx_addr);

		// Set up the endpoints. These are the local and remote server definitions (IP + socket)
		remote_endpoint_tx = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::broadcast(), opt_remote_socket);
		remote_endpoint_rx = boost::asio::ip::udp::endpoint(remote_rx_addr, opt_local_socket);

		// Start a Boost IO service and attach a new UDP socket
		udpSocket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), opt_local_socket));
		
		// Allow broadcasting if desired
		if (udp_transmit_broadcast) {
			boost::asio::socket_base::broadcast option(true);
			udpSocket->set_option(option);
		}

		start_udp_receive();

	} catch (exception& e) {
		cerr << "Failed to start UDP socket on local port " << opt_local_socket << "." << endl;
		return 1;
	}
	
	cout << "Listening at localhost:" << opt_local_socket << "." << endl;
	cout << "Listening for messages from " << opt_remote_rx_addr << ":" << opt_local_socket << "." << endl;
	cout << "Transmitting to " << opt_remote_tx_addr << ":" << opt_remote_socket << "." << endl; 

	// Create and start the serial port.
	try {
		serialPort = new boost::asio::serial_port(io_service, opt_port);

		// Configure the options for the serial port. I was having problems with the default options for one
		// or more of these options, so I just declare them all to what I want.
		serialPort->set_option(boost::asio::serial_port::baud_rate(opt_baud_rate));
		serialPort->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
		serialPort->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
		serialPort->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
		serialPort->set_option(boost::asio::serial_port::character_size(8));

		// Set up event handlers for receiving data on both ends
		start_serial_receive();

	} catch (exception& e) {
		cerr << "Failed to open serial port: " << opt_port << ". Does it exist?" << endl;
		return 1;
	}

	try {
		// Then start everything running
		io_service.run();
	} catch (exception& e) {
		cerr << e.what() << endl;
		return 1;
	}

	return 0;
}

/**
 * This is a convenience function for starting an asynchronous receive operation over UDP.
 */
void start_udp_receive()
{
	udpSocket->async_receive_from(
		boost::asio::buffer(udp_receive_buffer, opt_udp_rx_packet_size), 
		remote_endpoint_rx,
		&handle_udp_receive
	);
}

/**
 * This is a convenience function for starting an asynchronous receive operation over serial.
 */
void start_serial_receive()
{
	boost::asio::async_read(
		*serialPort,
		boost::asio::buffer(serial_receive_buffer, opt_serial_packet_size),
		&handle_serial_receive
	);
}



void print_hex(const char *s)
{
  while(*s)
    printf("%02x", (unsigned int) *s++);
  printf("\n");
}

/**
 * This function is called asynchronously after a UDP datagram is received. It just packages up that
 * same data for an asynchronous transmit out the serial port. Any errors are printed to the screen.
 */
void handle_udp_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
	// Once a UDP datagram is received, send it out over the serial port. If the reception was unsuccessful,	
	// print an error message.
	if (!error || error == boost::asio::error::message_size) {

        // Simulink to Autopilot (udp to mavlink serial)
        #ifdef PARSE_SLUGS_MAVLINK
		size_t old_size = bytes_transferred;
		boost::tuple<uint8_t*, size_t> t = parser->parse_udp2serial(
			reinterpret_cast<uint8_t*>(udp_receive_buffer), bytes_transferred);
		char *udp_receive_buffer = reinterpret_cast<char*>(t.get<0>());
		size_t bytes_transferred = t.get<1>();
        #endif

		boost::asio::async_write(
			*serialPort,
            boost::asio::buffer(udp_receive_buffer, bytes_transferred),
			&handle_serial_send
		);

	} else if (error != boost::asio::error::connection_refused) {
		print_error(error.message());
	}

	// Resume the waiting for UDP datagrams
	start_udp_receive();
}

/**
 * This function is called asynchronously after each UDP packet is sent. Since the packet has already been sent or errored out,
 * the only thing to do is print any errors.
 */
void handle_udp_send(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (error) {
		print_error(error.message());
	}
}


/**
 * This function deals with a buffer full of serial data. More specifically it sends the buffer off in a UDP
 * datagram if there isn't an error and resumes waiting.
 */
void handle_serial_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
	// If there isn't an error or the error is just one about message size
	if (!error || error == boost::asio::error::message_size) {
    
        // Autopilot to UDP (serial mavlink to simulink udp)
        #ifdef PARSE_SLUGS_MAVLINK
		boost::tuple<uint8_t*, size_t> t = parser->parse_serial2udp(
			reinterpret_cast<uint8_t*>(serial_receive_buffer), bytes_transferred);
		char *serial_receive_buffer = reinterpret_cast<char*>(t.get<0>());
		size_t bytes_transferred = t.get<1>();
        #endif

		udpSocket->async_send_to(
			boost::asio::buffer(serial_receive_buffer, bytes_transferred),
			remote_endpoint_tx,
			&handle_udp_send
		);
	} else if (error != boost::asio::error::connection_refused) {
		print_error(error.message());
	}

	// Resume waiting for more serial data
	start_serial_receive();
}

/**
 * This function is called asynchronously after each serial message buffer is transmit. Since the buffer has already been sent or errored out,
 * the only thing to do is print any errors.
 */
void handle_serial_send(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (error) {
		print_error(error.message());
	}
}

/**
 * This is a convenience function for printing an error with a timestamp at the start of it.
 */
void print_error(const string message)
{
	ostringstream msg;
	const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
	cerr << boost::posix_time::to_simple_string(now) << " - " << message << endl;
}
