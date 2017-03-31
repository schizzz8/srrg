#include "packet_decoder.h"
#include <string.h>
#include <stdio.h>

namespace srrg_orazio_core {

  PacketDecoder::PacketDecoder() {
    _read_buffer_end=_read_buffer_start;
    _length=0;
    _checksum=0;
    _status=Unsync;
  }


  bool PacketDecoder::putChar(unsigned char c){
    //printf ("char: %x, status: %d\n", (int) c, decoder->status);
    switch (_status) {
    case Unsync:
      _read_buffer_end=_read_buffer_start;
      _checksum = 0;
      if (c==0xAA)
	_status = Sync;
      break;
    case Sync:
      if (c==0x55)
	_status = Length;
      else
	_status = Unsync;
      break;
    case Length:
      _length=(uint8_t)c;
      _status=Payload;
      break;
    case Payload:
      if (_length==0) {
	_status = Unsync;
	_checksum^=c;
	if (!_checksum) {
	  return 1;
	} else {
	  return 0;
	}
      } else {
	_checksum^=c;
	*_read_buffer_end=c;
	_read_buffer_end++;
	_length--;
      }
      break;
    }
    return 0;
  }


}
