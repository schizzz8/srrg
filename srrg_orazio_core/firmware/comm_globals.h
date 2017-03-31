#pragma once
#include "packet_encoder.h"
#include "packet_decoder.h"

//! global packet encoder
extern PacketDecoder packet_decoder;
//! global packed decoder
extern PacketEncoder packet_encoder;

//! opens the serial port and instantiates the decoder and encoder
void initCommunication();

//! handles the incoming data from the serial line
//! each received byte is sent to the decoder
//! when a good packet is received
//! the appropriate management routine is called
//! to be called in the main loop out of ISR
bool handleCommunication();

//! empties the output buffer
//! to be called in the main loop, out of ISR
void flushCommunication();

