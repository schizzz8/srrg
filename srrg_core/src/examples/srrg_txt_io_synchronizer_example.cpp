#include <fstream>

#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_timestamp_synchronizer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "srrg_txt_io/sensor_message_sorter.h"

using namespace std;
using namespace srrg_core;

// Help objects to force linking
PinholeImageMessage i;
LaserMessage l;
MessageTimestampSynchronizer synchronizer;

const char* banner[] = {
  "srrg_txt_io_synchronizer_example: synchronizer example that generates associations between depth and rgb images contained on a file written with txt_io",
  "",
  "usage: srrg_txt_io_synchronizer_example <dump_file>",
  0
};

int main(int argc, char ** argv) {
  if (argc < 2 || !strcmp(argv[1], "-h")) {
    printBanner(banner);
    return 0;
  }

  std::string depth_topic = "/camera/depth/image_raw";
  std::string rgb_topic = "/camera/rgb/image_raw";
  cerr << "Synchronizing topics: " << endl;
  cerr << "  " << depth_topic << endl;
  cerr << "  " << rgb_topic << endl;

  MessageReader reader;
  reader.open(argv[1]);
  
  std::vector<std::string> sync_topics;
  sync_topics.push_back(rgb_topic);
  sync_topics.push_back(depth_topic);
  synchronizer.setTopics(sync_topics);
  BaseMessage* msg = 0;
  ofstream os("associations.txt");
  while ((msg = reader.readMessage())) {
    msg->untaint();
    BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(msg);
    if (sensor_msg->topic() == rgb_topic) {
      synchronizer.putMessage(sensor_msg);
    } else if (sensor_msg->topic() == depth_topic) {
      synchronizer.putMessage(sensor_msg);
    } else {
      delete sensor_msg;
    }
    if (synchronizer.messagesReady()) {
      char buf[1024];
      sprintf(buf, "%.5f %s %.5f %s", synchronizer.messages()[0]->timestamp(), synchronizer.messages()[0]->binaryFullFilename().c_str(), synchronizer.messages()[1]->timestamp(), synchronizer.messages()[1]->binaryFullFilename().c_str());
      os << buf << endl;
      synchronizer.reset();
      cerr << ".";
    }
  }
  cerr << "done" << endl;
}
