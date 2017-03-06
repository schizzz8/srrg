#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "srrg_txt_io/sensor_message_sorter.h"
#include "srrg_txt_io/spherical_image_message.h"

using namespace std;
using namespace srrg_core;

// Help objects to force linking
PinholeImageMessage im;
SphericalImageMessage sim;
LaserMessage las;

const char* banner[] = {
  "srrg_txt_io_merger_app: merges multiple txt_io files, based on the timestamp",
  "",
  "usage: srrg_txt_io_merger_app -o <output_file> <input 1> ... <input n>",
  0
};

struct MessageFileBeingRead {
  void open(const std::string& filename){
    reader.open(filename);
    _is_good=reader.good();
    next();
 }

  inline BaseSensorMessage* currentMessage() { return _current_sensor_message; }
  inline bool isGood() {return _is_good;}
  void next() {
    _current_sensor_message=0;
    while (_is_good) {
      BaseMessage* msg =reader.readMessage();
      if (!msg) {
	_is_good=false;
	return;
      }
      _current_sensor_message=dynamic_cast<BaseSensorMessage*>(msg);
      if (_current_sensor_message) {
      	return;
      }
      _is_good=false;
      delete msg;
    }
  }

  MessageReader reader;
  BaseSensorMessage* _current_sensor_message;
  bool _is_good;
};

struct FileMerger {
  bool addStream(const std::string& filename){
    MessageFileBeingRead* stream=new MessageFileBeingRead;
    stream->open(filename);
    if (! stream->isGood()) {
      delete stream;
      return false;
    }
    streams.push_back(stream);
    return true;
  }

  BaseSensorMessage* getYoungestMessage(){
    double earliest_time=std::numeric_limits<double>::max();
    BaseSensorMessage* earliest_message=0;
    MessageFileBeingRead* earliest_stream=0;
    for( MessageFileBeingRead* stream: streams){
      if (stream->isGood() && 
	  stream->currentMessage() && 
	  stream->currentMessage()->timestamp()<earliest_time) {
	earliest_stream=stream;
	earliest_message=stream->currentMessage();
	earliest_time=earliest_message->timestamp();
      } 
    }
    if (earliest_stream) {
      earliest_stream->next();
    }
    return earliest_message;
  }

  std::list<MessageFileBeingRead*> streams;
};

int main(int argc, char ** argv) {

  FileMerger file_merger;
  std::string outfilename="";
  if (argc < 2 || !strcmp(argv[1], "-h")) {
    printBanner(banner);
    return 0;
  }
  int c=1;
  while (c<argc) {
    if (! strcmp(argv[c],"-o") ){
      c++;
      outfilename=argv[c];
    } else {
      std::string fname=argv[c];
      if (!file_merger.addStream(fname)) {
	cerr << "opening file [" << fname <<  "]: ERROR" << endl << "aborting" << endl;
	return -1;
      } else {
	cerr << "opening file [" << fname << "]: OK" << endl;
      }
    }
    c++;
  }

  
  if (! outfilename.size()) {
    cerr << "no outfilename provided, aborting" << endl;
    return -1;
  }

  MessageWriter writer;
  writer.open(outfilename);
 
  BaseSensorMessage* msg=0;
  while ((msg=file_merger.getYoungestMessage())) {
    msg->untaint();
    writer.writeMessage(*msg);
    cerr << "msg: " << msg << endl;
    delete (msg);
    
  }

  writer.close();

  cerr << "done" << endl;
}
