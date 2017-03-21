#include <srrg_system_utils/system_utils.h>
#include "srrg_ros_wrappers/odom_tf_publisher.h"
#include <srrg_txt_io/message_reader.h>

using namespace std;
using namespace srrg_core;
using namespace srrg_core_ros;

const char* banner[] = {
  "publishes the transform tree each time it gets the odometry",
  "usage:",
  " rosrun srrg_core_ros srrg_state_publisher_node [options] <transforms>",
  " options:",
  " -odom_topic <string>         :   the topic where the odometry os published",
  " -base_link_frame_id <string> :   the frame id of the robot",
  " -odom_frame_id <string> :   the frame id of the odometry",
  0
};

int main(int argc, char **argv) {
  std::string odom_topic = "/odom";
  std::string base_link_frame_id = "/base_link";
  std::string odom_frame_id = "/odom";
  std::string config_file="";
  int c = 1;
  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      printBanner(banner);
      return 0;
    }
    if (! strcmp(argv[c], "-odom_topic")){
      c++;
      odom_topic = argv[c];
    } else if (! strcmp(argv[c], "-base_link_frame_id")){
      c++;
      base_link_frame_id = argv[c];
    } else if (! strcmp(argv[c], "-odom_frame_id")){
      c++;
      odom_frame_id = argv[c];
    } else if (! strcmp(argv[c], "-odom_topic")){
      c++;
      odom_topic = argv[c];
    } else {
      config_file = argv[c];
    }
    c++;
  }

  ros::init(argc, argv, "srrg_state_publisher_node");
  ros::NodeHandle nh;
  tf::TransformBroadcaster broadcaster;
  
  StaticTransformTree tree;
  MessageReader reader;
  reader.open(config_file);
  while(reader.good()){
    BaseMessage* msg=reader.readMessage();
    if (! msg)
      continue;
    StaticTransformMessage* tf=dynamic_cast<StaticTransformMessage*>(msg);
    if (tf) {
      tree.addMessage(tf);
    }
  }
  cerr << "tree root: " << tree.rootFrameId() << endl;
  tree.isWellFormed();

  OdomTfPublisher * odom_publisher = new OdomTfPublisher(&nh, 
							 &broadcaster,
							 &tree);

  odom_publisher->setBaseLinkFrameId(base_link_frame_id);
  odom_publisher->setOdomFrameId(odom_frame_id);
  cerr << "odom topic: " << odom_topic << endl;
  odom_publisher->subscribe(odom_topic);

  if (! config_file.length()){
    printBanner(banner);				
    return 0;
  }

  ros::spin();

  return 0;
}
