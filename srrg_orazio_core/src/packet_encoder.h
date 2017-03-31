#pragma once
#include <stdint.h>

namespace srrg_orazio_core {

  //! base class that formats a binary blob into a packet woth the checksum
  //! and implements a nice buffer (to be used by a serial ISR)
  //! given a blob of x bytes, the packet is 
  //! AA 55 SIZE <blob> CHECKSUM
  //! AA55 are the header
  //! size is a uint8 telling the size
  //! checksum is the xor of all bytes minus the header (AA55)

  class PacketEncoder {
  public:
    PacketEncoder();
    int bytesToSend(); //< how many bytes are in buffer?
    unsigned char getChar(); //< returns the oldest char in buffer. 
    //<If buffer empty returns undefined values 
  
    int putData(unsigned char* raw_packet, uint8_t size); //< makes a packet with the data passed as input and puts it in the buffer
 
    //< makes a packet with the struct passed as input and puts it in the buffer
    template <typename T>
    int putPacket(T& packet){
      return putData((unsigned char*) &packet, sizeof(T));
    }
  protected:
    uint8_t _tx_buffer_start_idx;
    uint8_t _tx_buffer_end_idx;
    unsigned char _tx_buffer[256];
  };


}
