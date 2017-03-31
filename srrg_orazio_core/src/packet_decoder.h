#pragma once
#include <stdint.h>

namespace srrg_orazio_core {

  //! packet parser
  //! it  recognized well formed packets AA 55 size <blob> checksum
  //! used to parse consistent packets
  //! you typically instantiate a PacketDecoder per stream
  //! and pass it all chars of a stream as they come in (using putChar());
  //! when a packet is complete you have a look and do something in response
  class PacketDecoder{
  public:
    PacketDecoder();
  
    bool putChar(unsigned char c);  //< put a char in the decoder.returns true if packet complete
  
    //! beginning of the <blob> payload. 
    inline unsigned char* readBufferStart() {return _read_buffer_start;}
    //! end of gthe <blob> payload. Makes sense only if a packet is complete
    inline unsigned char* readBufferEnd() {return _read_buffer_end;}
  protected:
    enum Status {Unsync=0x0, Sync=0x1, Length=0x2, Payload=0x3};
    unsigned char _read_buffer_start[256];
    unsigned char *_read_buffer_end;
    uint8_t _length;
    uint8_t _checksum;
    Status  _status;
  };


}
