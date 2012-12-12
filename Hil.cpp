/*
The MIT License

Copyright (c) 2010 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// ==============================================================
// This code provides a protocol decoder for the binary communications
// protocol used between the groundstation/HIL and the dsPIC in
// the Autoboat project. As most programming relies on Simulink and
// the Real-Time Workshop, retrieval functions here return arrays of
// data to be compatible (instead of much-nicer structs).
// A complete structure is passed byte-by-byte and assembled in an
// internal buffer. This is then verified by its checksum and the
//data pushed into the appropriate struct. This data can then be
// retrieved via an accessor function.
//
// While this code was written specifically for the Autoboat and its
// protocol, it has been kept as modular as possible to be useful
// in other situations with the most minimal alterations.
//
// Code by: Bryant W. Mairs
// First Revision: Aug 25 2010
// ==============================================================

#include "Hil.h"

#include <stdint.h>
#include <string.h>

uint8_t txWrapper[22] = {
    '%',
    '&',
    0,
    16,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    '^',
    '&'
};
uint16_t receivedTimestamp = 0;
uint8_t newHilData = false;

// Keep track of how many messages were successfully received.
static uint32_t receivedMessageCount = 0;
// Keep track of how many fails we've run into
static uint32_t failedMessageCount = 0;
static uint8_t sameFailedMessageFlag = 0;

// Track whether HIL is currently active.
static uint8_t hilStatus = false;

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 * sensorMode is a boolean that is true when generated actuator sensor data
 * should be overridden by real-world actuator sensor data.
 */
void HilBuildMessage(uint8_t data)
{
	static uint8_t message[64] = {0};
	static uint8_t messageIndex = 0;
	static uint8_t messageState = 0;

	// This contains the function's state of whether
	// it is currently building a message.
	// 0 - Awaiting header byte 0 (%)
	// 1 - Awaiting header byte 1 (&)
	// 2 - Building message
	// 3 - Awaiting header byte 0 (^)
	// 4 - Awaiting header byte 1 (&)
	// 5 - Reading checksum character

	// We start recording a new message if we see the header
	if (messageState == 0) {
		if (data == '%') {
			message[0] = data;
			messageIndex++;
			messageState = 1;
		} else {
			messageIndex = 0;
			messageState = 0;

			// Here we've failed parsing a message so count another failure.
			if (!sameFailedMessageFlag) {
				failedMessageCount++;
				sameFailedMessageFlag = 1;
			}
		}
	} else if (messageState == 1) {
		// If we don't find the necessary ampersand we start over
		// waiting for a new sentence
		if (data == '&') {
			message[messageIndex] = data;
			messageIndex++;
			messageState = 2;

		} else if (data != '%'){
			messageIndex = 0;
			messageState = 0;

			// Here we've failed parsing a message so count another failure.
			if (!sameFailedMessageFlag) {
				failedMessageCount++;
				sameFailedMessageFlag = 1;
			}
		}
	} else if (messageState == 2) {
		// Record every character that comes in now that we're building a sentence.
		// Stop scanning once we've reached the message length of characters.
		message[messageIndex++] = data;
		if (messageIndex > 3 && messageIndex == message[3] + 5) {
			if (data == '^') {
				messageState = 3;
			} else {
				messageState = 0;
				messageIndex = 0;

				// Here we've failed parsing a message.
				failedMessageCount++;
				sameFailedMessageFlag = 1;
			}
		} else if (messageIndex == sizeof(message) - 3) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			messageState = 0;
			messageIndex = 0;

			// Here we've failed parsing a message.
			failedMessageCount++;
			sameFailedMessageFlag = 1;
		}
	} else if (messageState == 3) {
            // If we don't find the necessary ampersand we continue
            // recording data as we haven't found the footer yet until
            // we've filled up the entire message (ends at 124 characters
            // as we need room for the 2 footer chars).
            message[messageIndex++] = data;
            if (data == '&') {
                message[messageIndex] = data;

                // The checksum is now verified and if successful the message
                // is stored in the appropriate struct.
				int g = HilCalculateChecksum(&message[4], 22);
                if (message[2] == g) {
                    // We now memcpy all the data into our data store.
                    receivedMessageCount++;
					memcpy(&txWrapper[18], &message[24], 2);
					txWrapper[2] = txWrapper[18] ^ txWrapper[19];
					newHilData = true;

                    // Now that we've successfully parsed a message, clear the flag.
                    sameFailedMessageFlag = 0;
                } else {
                    // Here we've failed parsing a message.
                    failedMessageCount++;
                    sameFailedMessageFlag = 1;
                }

                // We clear all state variables here regardless of success.
                messageIndex = 0;
                messageState = 0;
                int b;
                for (b = 0; b < sizeof(message); b++) {
                    message[b] = 0;
                }
            } else {
                messageIndex = 0;
                messageState = 0;
            }
	}
}

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
uint8_t HilCalculateChecksum(const uint8_t *sentence, uint8_t size)
{

	uint8_t checkSum = 0;
	uint8_t i;
	for (i = 0; i < size; i++) {
		checkSum ^= sentence[i];
	}

	return checkSum;
}
