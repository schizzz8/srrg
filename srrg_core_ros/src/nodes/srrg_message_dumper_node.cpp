#include <iostream>
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

#include <srrg_system_utils/system_utils.h>
#include "srrg_ros_wrappers/imu_interpolator.h"
#include "srrg_ros_wrappers/image_message_listener.h"
#include "srrg_ros_wrappers/laser_message_listener.h"
#include "srrg_ros_wrappers/multiecholaser_message_listener.h"
#include "srrg_ros_wrappers/spherical_image_message_listener.h"
#include "srrg_ros_wrappers/joint_state_message_listener.h"
#include <srrg_txt_io/message_dumper_trigger.h>

using namespace std;
using namespace Eigen;
using namespace srrg_core;
using namespace srrg_core_ros;

const char* banner[] = {
    "srrg_message_dumper_node",
    "simple message dumper",
    "usage:",
    " start the kinect/xtion node or play a bag that contains depth images",
    " in a shell type",
    " rosrun srrg_core_ros srrg_message_dumper_node [options]",
    " where [options]: ",
    "  -t:  [string] ros topics of images",
    "  -j:  [string] ros topics of joints",
    "  -laser:  [string] ros topics of laser",
    "  -base_link_frame_id <base frame id> (default unset, if provided it will consider the odometry)",
    "  -odom_frame_id <base frame id> (default odom)",
    "  -o:       [string] output filename where to write the local maps. Default: out.txt",
    0
};


int main(int argc, char **argv) {
    std::string outputFilename="out.txt";
    std::string outputFileDir;
    int c = 1;
    std::list<string> image_topics;
    std::list<string> laser_topics;
    std::list<string> multiecholaser_topics;
    std::list<string> joint_topics;
    std::list<string> spherical_topics;
    std::list<SphericalImageMessageListener*> spherical_listeners;
    std::list<ImageMessageListener*> camera_listeners;
    std::list<LaserMessageListener*> laser_listeners;
    std::list<MultiEchoLaserMessageListener*> multiecholaser_listeners;
    std::list<JointStateMessageListener*> joint_listeners;
    std::string imu_topic = "";
    tf::TransformListener * listener = 0;
    std::string base_link_frame_id = "";
    std::string odom_frame_id = "/odom";

    while (c<argc){
        if (! strcmp(argv[c], "-h")){
            printBanner(banner);
            return 0;
        }
        if (! strcmp(argv[c], "-t")){
            c++;
            image_topics.push_back(argv[c]);
        }
        if (! strcmp(argv[c], "-base_link_frame_id")){
            c++;
            base_link_frame_id=argv[c];
        }
        if (! strcmp(argv[c], "-odom_frame_id")){
            c++;
            odom_frame_id=argv[c];
        }
        if (! strcmp(argv[c], "-imu")){
            c++;
            imu_topic=argv[c];
        }
        if (! strcmp(argv[c], "-laser")){
            c++;
            laser_topics.push_back(argv[c]);
        }
        if (! strcmp(argv[c], "-multiecholaser")){
            c++;
            multiecholaser_topics.push_back(argv[c]);
        }
        if (! strcmp(argv[c], "-j")){
            c++;
            joint_topics.push_back(argv[c]);
        }
        if (! strcmp(argv[c], "-spherical")){
            c++;
            spherical_topics.push_back(argv[c]);
        }
        if (! strcmp(argv[c], "-o")){
            c++;
            outputFilename = argv[c];
        }
        c++;
    }

    ros::init(argc, argv, "srrg_message_dumper_node");
    if (base_link_frame_id.length()>0){
        cerr << "making listener" << endl;
        listener = new tf::TransformListener();
    }
    ros::NodeHandle nh;
    image_transport::ImageTransport itr(nh);

    ImuInterpolator* interpolator = 0;
    if (imu_topic.length()){
        interpolator = new ImuInterpolator(&nh);
        interpolator->subscribe(imu_topic);
    }

    SensorMessageSorter sorter;
    sorter.setTimeWindow(1.0);
    MessageWriter writer;
    writer.open(outputFilename);
    MessageDumperTrigger* dumper = new MessageDumperTrigger(&sorter, 0, &writer, outputFileDir+"/");
    for (std::list<std::string>::iterator it = image_topics.begin(); it!=image_topics.end(); it++) {
        std::string image_topic = *it;
        ImageMessageListener* camera_listener =
                new ImageMessageListener (&nh, &itr, &sorter, listener, odom_frame_id, base_link_frame_id);
        if (interpolator)
            camera_listener->setImuInterpolator(interpolator);
        camera_listener->subscribe(image_topic);
        camera_listener->setVerbose(true);
        cerr << "subscribing for image topic: " << image_topic << endl;
        camera_listeners.push_back(camera_listener);
    }

    for (std::list<std::string>::iterator it = laser_topics.begin(); it!=laser_topics.end(); it++) {
        std::string laser_topic = *it;
        LaserMessageListener* laser_listener =
                new LaserMessageListener (&nh, &sorter, listener, odom_frame_id, base_link_frame_id);
        if (interpolator)
            laser_listener->setImuInterpolator(interpolator);
        laser_listener->subscribe(laser_topic);
        laser_listener->setVerbose(true);
        cerr << "subscribing for laser topic: " << laser_topic << endl;
        laser_listeners.push_back(laser_listener);
    }


    for (std::list<std::string>::iterator it = multiecholaser_topics.begin(); it!=multiecholaser_topics.end(); it++) {
        std::string multiecholaser_topic = *it;
        MultiEchoLaserMessageListener* multiecholaser_listener =
                new MultiEchoLaserMessageListener (&nh, &sorter, listener, odom_frame_id, base_link_frame_id);
        if (interpolator)
            multiecholaser_listener->setImuInterpolator(interpolator);
        multiecholaser_listener->subscribe(multiecholaser_topic);
        multiecholaser_listener->setVerbose(true);
        cerr << "subscribing for multiecholaser topic: " << multiecholaser_topic << endl;
        multiecholaser_listeners.push_back(multiecholaser_listener);
    }

    for (std::list<std::string>::iterator it = spherical_topics.begin(); it!=spherical_topics.end(); it++) {
        std::string topic = *it;
        SphericalImageMessageListener* spherical_listener =
                new SphericalImageMessageListener (&nh, &sorter, listener, odom_frame_id, base_link_frame_id);
        if (interpolator)
            spherical_listener->setImuInterpolator(interpolator);
        spherical_listener->subscribe(topic);
        spherical_listener->setVerbose(true);
        cerr << "subscribing for topic: " << topic << endl;
        spherical_listeners.push_back(spherical_listener);
    }

    for (std::list<std::string>::iterator it = joint_topics.begin(); it!=joint_topics.end(); it++) {
        std::string topic = *it;
        JointStateMessageListener* joint_listener =
                new JointStateMessageListener (&nh, &sorter);
        joint_listener->subscribe(topic);
        cerr << "subscribing for topic: " << topic << endl;
        joint_listeners.push_back(joint_listener);
    }

    ros::spin();

    return 0;
}
