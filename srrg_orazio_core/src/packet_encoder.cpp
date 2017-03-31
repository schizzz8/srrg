#include "packet_encoder.h"
namespace srrg_orazio_core{

  PacketEncoder::PacketEncoder() {
    _tx_buffer_start_idx = _tx_buffer_end_idx = 0;
  }

  int PacketEncoder::bytesToSend(){
    return (uint8_t) (_tx_buffer_end_idx - _tx_buffer_start_idx);
  }

  unsigned char PacketEncoder::getChar(){
    if (_tx_buffer_end_idx==_tx_buffer_start_idx)
      return 0;
    unsigned char c = _tx_buffer[_tx_buffer_start_idx];
    _tx_buffer_start_idx++;
    return c;
  }

  int PacketEncoder::putData(unsigned char* raw_packet, uint8_t size){
    uint8_t bytes_to_send = bytesToSend();
    uint8_t free_bytes = 255-bytes_to_send;
    if (free_bytes<(size+4))
      return 0;
    _tx_buffer[_tx_buffer_end_idx++]=0xAA;
    _tx_buffer[_tx_buffer_end_idx++]=0x55;
    _tx_buffer[_tx_buffer_end_idx++]=size;
    unsigned char checksum=0;
    while(size>0){
      uint8_t c=*raw_packet;
      raw_packet++;
      checksum^=c;
      _tx_buffer[_tx_buffer_end_idx++]=c;
      size--;
    }
    _tx_buffer[_tx_buffer_end_idx++]=checksum;
    return 1;
  }


} // end namespace
