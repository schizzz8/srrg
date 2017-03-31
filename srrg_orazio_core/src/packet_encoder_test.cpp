#include "packet_encoder.h"
#include "packet_decoder.h"
#include "packets.h"
#include <iostream>

using namespace srrg_orazio_core;
using namespace std;

PacketEncoder encoder;
PacketDecoder decoder;

SystemStatusPacket status_sent, status_received;
ResponsePacket response_sent, response_received;

int main(int argc, char**argv){
  int max_trials=10;
  int packets_in_trial=20;
  for (int i=0; i<max_trials; i++){
    for (int j=0; j<packets_in_trial; j++) {
      status_sent.seq=i*packets_in_trial+j;
      encoder.putPacket(status_sent);
      //response_sent.seq=i*packets_in_trial+j;
      //encoder.putPacket(response_sent);
    }
    while (encoder.bytesToSend()){
      unsigned char c=encoder.getChar();
      bool packet_complete=decoder.putChar(c);
      if (packet_complete){
	Packet* p=reinterpret_cast<Packet*>(decoder.readBufferStart());
	switch(p->id){
	case (SystemStatusPacket::default_id):
	  status_received=*reinterpret_cast<SystemStatusPacket*>(decoder.readBufferStart());
	  cerr << packet_complete << " STATUS " << status_received.seq << endl;
	  break;
	case (ResponsePacket::default_id):
	  response_received=*reinterpret_cast<ResponsePacket*>(decoder.readBufferStart());
	  cerr << packet_complete << " RESPONSE " << response_received.seq << endl;
	  break;
	default:;
	}

      }
    }
    
  }
  
}

