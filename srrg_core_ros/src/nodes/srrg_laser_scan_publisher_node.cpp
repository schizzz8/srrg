#include <fstream>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/sensor_message_sorter.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace srrg_core;

// Help objects to force linking
LaserMessage l;

const char* banner[] = {
    "srrg_open_file_example_app: example on how to open a txt tio file and read the stuff",
    "",
    "usage: srrg_open_file_example_app <dump_file>",
    0
};

tf::Transform eigen2tfTransform(const Eigen::Isometry3f& T){
  Eigen::Quaternionf q(T.linear());
  Eigen::Vector3f t=T.translation();
  tf::Transform tft;
  tft.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
  tft.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  return tft;
}

int main(int argc, char ** argv) {
    if (argc < 2 || !strcmp(argv[1], "-h")) {
        printBanner(banner);
        return 0;
    }

    ros::init(argc, argv, "laser_publisher");
    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    static tf::TransformBroadcaster br;
    ros::Rate r(3.0);

    MessageReader reader;
    reader.open(argv[1]);

    BaseMessage* msg = 0;
    while ((msg = reader.readMessage()) && nh.ok()) {
        msg->untaint();
        LaserMessage* laser_msg = dynamic_cast<LaserMessage*>(msg);
        if (laser_msg) {
            tf::Transform transform = eigen2tfTransform(laser_msg->odometry());
            br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/odom","/base_link"));
            //r.sleep();

            sensor_msgs::LaserScan scan;
            scan.header.stamp = ros::Time::now();
            scan.header.frame_id = "laser_frame";
            scan.angle_min = laser_msg->minAngle();
            scan.angle_max = laser_msg->maxAngle();
            scan.angle_increment = laser_msg->angleIncrement();
            scan.time_increment = laser_msg->timeIncrement();
            scan.range_min = laser_msg->minRange();
            scan.range_max = laser_msg->maxRange();

            scan.ranges.resize(laser_msg->ranges().size());
            for(size_t i=0; i < laser_msg->ranges().size(); i++)
                scan.ranges[i] = laser_msg->ranges().at(i);

            scan_pub.publish(scan);
            r.sleep();
        }

    }
    cerr << "done" << endl;
}
