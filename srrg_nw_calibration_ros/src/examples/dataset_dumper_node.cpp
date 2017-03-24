#include "../utils/utils.h"

using namespace new_world_calibration;

/// VARS
int so3_sensor_num = 0;
int so2_sensor_num = 0;

tf::TransformListener* tf_listener;
TfFrameVector so3_tf_frames;
TfFrameVector so2_tf_frames;
TfTransformVector curr_so2;
TfTransformVector curr_so3;
Eigen::Vector2f curr_ticks;
std::ofstream* writer;
bool tf_found;

/// TAGS
std::string JOINT_STATE = "JOINT_STATE";
std::string SENSOR3_POSE = "SENSOR3_POSE";
std::string SENSOR2_POSE = "SENSOR2_POSE";




void jointCallback(const sensor_msgs::JointStateConstPtr &joint){

    /// LISTEN DATA
    /// JOINT
    float tl = (float)joint->position[0];
    float tr = (float)joint->position[1];
    //curr_ticks = Eigen::Vector2f(tl, tr);

    float current_stamp = (float)(joint->header.stamp).toSec();

    /// TFs
    tf::StampedTransform current_T;
    size_t curr_so2_dataset = 0;
    tf_found = true;
    ///seek for tf(s) of the SO2 sensors
    for(TfFrameVector::iterator so2it = so2_tf_frames.begin(); so2it != so2_tf_frames.end(); ++so2it, ++curr_so2_dataset){
        try{
            tf_listener->lookupTransform((*so2it).frame_id_, (*so2it).child_frame_id_, ros::Time(0), current_T);
            curr_so2.at(curr_so2_dataset) = current_T;
        }
        catch (tf::TransformException &ex){
            tf_found = false;
            ROS_ERROR("%s", ex.what());
            continue;
        }
    }

    ///seek for the tf(s) of the SO3 sensors
    size_t curr_so3_dataset = 0;
    for(TfFrameVector::iterator so3it = so3_tf_frames.begin(); so3it != so3_tf_frames.end(); ++so3it, ++curr_so3_dataset){
        try{
            tf_listener->lookupTransform((*so3it).frame_id_, (*so3it).child_frame_id_, ros::Time(0), current_T);
            curr_so3.at(curr_so3_dataset) = current_T;
        }
        catch (tf::TransformException &ex){
            tf_found = false;
            ROS_ERROR("%s", ex.what());
            continue;
        }
    }


    if (tf_found)
    {
        tf_found = false;

        /// Store ticks
        *writer << JOINT_STATE << " "
               << current_stamp << " "
               << tl << " "
               << tr << std::endl;

        Eigen::Isometry2f sensor2_pose;
        sensor2_pose.setIdentity();
        /// Store 2d sensors transforms
        for(size_t i = 0; i < so2_sensor_num; ++i)
        {
            transformTFToEigenImpl(curr_so2.at(i), sensor2_pose);
            Eigen::Vector3f sensor_pose = t2v(sensor2_pose);

            /// Store current 2d pose
            *writer << SENSOR2_POSE << " "
                    << current_stamp << " "
                    << so2_tf_frames.at(i).frame_id_ << " "
                    << so2_tf_frames.at(i).child_frame_id_ << " "
                    << sensor_pose(0) << " "
                    << sensor_pose(1) << " "
                    << sensor_pose(2) << std::endl;

        }

        Eigen::Isometry3f sensor3_pose;
        sensor3_pose.setIdentity();
        /// Store 3d sensors transforms
        for(size_t i = 0; i < so3_sensor_num; ++i)
        {
            transformTFToEigenImpl(curr_so3.at(i), sensor3_pose);
            Vector6f sensor_pose = t2v(sensor3_pose);

            /// Store current 3d pose
            *writer << SENSOR3_POSE << " "
                    << current_stamp << " "
                    << so3_tf_frames.at(i).frame_id_ << " "
                    << so3_tf_frames.at(i).child_frame_id_ << " "
                    << sensor_pose(0) << " "
                    << sensor_pose(1) << " "
                    << sensor_pose(2) << " "
                    << sensor_pose(3) << " "
                    << sensor_pose(4) << " "
                    << sensor_pose(5) << std::endl;

        }
    } //end else if (tf_found)

} //end of callback





void printBanner(const char** banner) {
    const char** b = banner;
    while(*b) {
        std::cerr << *b << std::endl;
        b++;
    }
}

const char* banner[]={
        "\n\nUsage:  dataset_dumper_node -joint-topic <joint_topic> -o <dataset.txt> -sensor2-frame <frame> <child_frame> -sensor3-frame <frame> <child_frame> [Options]\n",
        "Example:  dataset_dumper_node -joint-topic /joint_state -o dataset.txt -sensor2-frame /base_link /laser_frame -sensor3-frame /base_link /camera_left_frame\n\n",
        "Options:\n",
        "------------------------------------------\n",
        "-joint-topic <string>               topic name of joints of the platform",
        "-sensor2-frame <string> <string>    base frame and child frame of tf attached to 2d sensor tracker",
        "-sensor3-frame <string> <string>    base frame and child frame of tf attached to 3d sensor tracker",
        "-rate <float>                       log rate expressed in Hz, default 2",
        "-h                                  this help\n",
        0
};


int main(int argc, char ** argv){

  ros::init(argc, argv, "new_world_calibration_node");
  ros::NodeHandle nh("~");
  ROS_INFO("new_world_calibration_node started...");

  if(argc < 2){
      printBanner(banner);
      return 1;
    }

    std::string joint_topic = "/joint_state";
    std::string out_file = "";
  int iterations = 5;
  int block = 50;
  double rate = 10;

  int c=1;

  while(c<argc){
      if (! strcmp(argv[c],"-h")){
          printBanner(banner);
          return 1;
        }
      else if(! strcmp(argv[c],"-joint-topic")){
          c++;
          joint_topic = argv[c];
        }
      else if (! strcmp(argv[c],"-sensor3-frame")){
          tf_frame frame;
          c++;
          frame.frame_id_ = argv[c];
          c++;
          frame.child_frame_id_ = argv[c];
          so3_tf_frames.push_back(frame);
        }
      else if(! strcmp(argv[c],"-sensor2-frame")){
          tf_frame frame;
          c++;
          frame.frame_id_ = argv[c];
          c++;
          frame.child_frame_id_ = argv[c];
          so2_tf_frames.push_back(frame);
      }
      else if(! strcmp(argv[c],"-rate")){
          c++;
          rate = std::atof(argv[c]);
        }
      else if(! strcmp(argv[c],"-o")){
          c++;
          out_file = argv[c];
      }
      c++;
    }

  for(TfFrameVector::iterator it = so2_tf_frames.begin(); it != so2_tf_frames.end(); ++it )
    std::cerr<<"#i-th so2-frame -> "<<(*it)<<std::endl;
  for(TfFrameVector::iterator it = so3_tf_frames.begin(); it != so3_tf_frames.end(); ++it )
    std::cerr<<"#i-th so3-frame -> "<<(*it)<<std::endl;
    std::cerr<<"out_file:    "<<iterations<<std::endl;
    std::cerr<<"joint-topic: "<<joint_topic<<std::endl;
    std::cerr<<"rate:        "<<rate<<std::endl;

    writer = new std::ofstream();
    writer->open(out_file, std::ios::out | std::ios::app | std::ios::binary);

  ros::Subscriber joint_sub = nh.subscribe(joint_topic, 1, jointCallback);
  tf_listener = new tf::TransformListener;

    ros::Rate r(rate);

    for(; ros::ok() ;){
    //while(ros::ok()){
        r.sleep();
        ros::spinOnce(); //queue size of jointCallback is 1
    }
    if(ros::ok()){
        ros::shutdown();
    }
    else{
        //closing file
        writer->close();
        std::cerr<<"closing file "<<std::endl;
        fflush(stdout);
    }

        return 0;
}


