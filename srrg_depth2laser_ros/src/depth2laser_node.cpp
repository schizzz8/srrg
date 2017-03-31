#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <string>
//GLOBALS
//=====================================================================

//Topics and messages
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ros::Subscriber info_sub;
ros::Subscriber frame_sub;
ros::Publisher cloud_pub;
ros::Publisher laser_pub;
sensor_msgs::CameraInfo camerainfo;
sensor_msgs::PointCloud cloud;
sensor_msgs::LaserScan scan;
float inverse_angle_increment;
//EIGEN
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Eigen::Matrix3f K;
Eigen::Matrix3f inv_K;
Eigen::Isometry3f camera_transform;
Eigen::Isometry3f laser_transform;
Eigen::Isometry3f camera2laser_transform;

//TF
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
tf::TransformListener* listener;

using namespace std;

//Other
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool gotInfo=false;
//Configuration structure
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct Configuration{
  //TF STUFF
  string laser_frame_id;
  string base_frame_id;
  // laser_parameters
  double angle_min;
  double angle_max;
  int num_ranges;
  double range_min;
  double range_max;
  double laser_plane_thickness;
  //TOPICS
  string pointcloud_topic;
  string camera_image_topic;
  string camera_info_topic;
  string laser_topic;
  //OPTIONS
  int publish_pointcloud;
}; 

Configuration c;

void clearScan(){
  scan.ranges.resize(c.num_ranges);
  scan.angle_min=c.angle_min;
  scan.angle_max=c.angle_max;
  scan.range_min=c.range_min;
  scan.range_max=c.range_max;
  scan.angle_increment=(c.angle_max-c.angle_min)/c.num_ranges;
  scan.header.frame_id=c.laser_frame_id;
  scan.scan_time=0;
  scan.time_increment=0;
  inverse_angle_increment=1./scan.angle_increment;
  for (size_t i=0; i<scan.ranges.size(); i++){
    scan.ranges[i]=c.range_max+0.1;
  }
}

  
//=====================================================================


Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p){
  Eigen::Isometry3f iso;
  iso.translation().x()=p.getOrigin().x();
  iso.translation().y()=p.getOrigin().y();
  iso.translation().z()=p.getOrigin().z();
  Eigen::Quaternionf q;
  tf::Quaternion tq = p.getRotation();
  q.x()= tq.x();
  q.y()= tq.y();
  q.z()= tq.z();
  q.w()= tq.w();
  iso.linear()=q.toRotationMatrix();
  return iso;
}

tf::Transform eigen2tfTransform(const Eigen::Isometry3f& T){
  Eigen::Quaternionf q(T.linear());
  Eigen::Vector3f t=T.translation();
  tf::Transform tft;
  tft.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
  tft.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  return tft;
}


void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info){
    //K matrix
    //================================================================================
    camerainfo.K = info->K;
    ROS_INFO("Got camera info!");
    K(0,0) = camerainfo.K.c_array()[0];
    K(0,1) = camerainfo.K.c_array()[1];
    K(0,2) = camerainfo.K.c_array()[2];
    K(1,0) = camerainfo.K.c_array()[3];
    K(1,1) = camerainfo.K.c_array()[4];
    K(1,2) = camerainfo.K.c_array()[5];
    K(2,0) = camerainfo.K.c_array()[6];
    K(2,1) = camerainfo.K.c_array()[7];
    K(2,2) = camerainfo.K.c_array()[8];

    K = Eigen::Matrix3f::Zero();
    K(0,0) = 273.457;
    K(1,1) = 273.457;
    K(0,2) = 159.5;
    K(1,2) = 119.5;
    K(2,2) = 1;

    inv_K=K.inverse();
    cerr << "got camera matrix" << endl;
    cerr << K << endl;

    //TF
    //================================================================================
    ROS_INFO("waif for tf...");
    tf::StampedTransform camera_tf;
    listener->waitForTransform(c.base_frame_id,info->header.frame_id,ros::Time(0),ros::Duration(30));
    listener->lookupTransform(c.base_frame_id, info->header.frame_id,ros::Time(0), camera_tf);
    camera_transform=tfTransform2eigen(camera_tf);
    ROS_INFO("got camera transform");

    tf::StampedTransform laser_tf;
    listener->waitForTransform(c.base_frame_id,c.laser_frame_id,ros::Time(0),ros::Duration(30));
    listener->lookupTransform(c.base_frame_id, c.laser_frame_id,ros::Time(0), laser_tf);
    
    laser_transform=tfTransform2eigen(laser_tf);
    camera2laser_transform=laser_transform.inverse()*camera_transform;
    //Don't want to receive any camera info stuff, basically this callback is used a init function
    info_sub.shutdown();
    //set flag. Bye.
    gotInfo=true;
}

void frameCallback(const sensor_msgs::Image::ConstPtr& frame)
{
  if(!gotInfo) return;
  ros::Time current_time = ros::Time(0);
  cloud.points.clear();
  clearScan();
  scan.header.stamp=frame->header.stamp;
  scan.header.frame_id=c.laser_frame_id;
  cloud.header.frame_id=frame->header.frame_id;
  cloud.header.stamp= frame->header.stamp;
  
  cv_bridge::CvImageConstPtr image =  cv_bridge::toCvShare(frame);
  float  squared_max_norm=scan.range_max*scan.range_max;
  float  squared_min_norm=scan.range_min*scan.range_min;
  int good_points=0;
  for(int i =0;i<image->image.rows;i++){
    const ushort* row_ptr = image->image.ptr<ushort>(i);
    for(int j=0;j<image->image.cols;j++){
      ushort id=row_ptr[j];
      if(id!=0){
	float d=1e-3*id;
	Eigen::Vector3f image_point(j*d,i*d,d);
	Eigen::Vector3f camera_point=inv_K*image_point;
	Eigen::Vector3f laser_point=camera2laser_transform*camera_point;
	geometry_msgs::Point32 point;
	point.x=camera_point.x();
	point.y=camera_point.y();
	point.z=camera_point.z();
	cloud.points.push_back(point);
	if (fabs(laser_point.z())<c.laser_plane_thickness){
	  float theta=atan2(laser_point.y(),laser_point.x());
	  /*
	  if(theta<scan.angle_min)
	    continue;
	  if(theta>scan.angle_max)
	    continue;
	  */
	  float range=laser_point.head<2>().squaredNorm();
	  
	  if (range<squared_min_norm)
	    continue;
	  if (range>squared_max_norm)
	    continue;
	  range=sqrt(range);
	  int bin=(int)((theta-scan.angle_min)*inverse_angle_increment);
	  if (bin<0||bin>=scan.ranges.size())
	    continue;
	  if(scan.ranges[bin]>range) {
	    scan.ranges[bin]=range;
	    good_points++;
	  }
	}
      }
    }
  }
  cerr << good_points << " ";
  if(c.publish_pointcloud){
    cloud_pub.publish(cloud);
  }
  laser_pub.publish(scan);
}

//Params echo
//================================================================================
void EchoParameters(){
  cerr << "_angle_min: " <<  c.angle_min << endl;
  cerr << "_angle_max: " << c.angle_max << endl;
  cerr << "_num_ranges: " << c.num_ranges << endl;
  cerr << "_range_min: " << c.range_min << endl;
  cerr << "_range_max: " << c.range_max << endl;
  cerr << "_laser_plane_thickness: " << c.laser_plane_thickness << endl;
  cerr << "_base_frame_id: " << c.base_frame_id << endl;
  cerr << "_laser_frame_id: " << c.laser_frame_id << endl;
  cerr << "_depth_topic: " << c.camera_image_topic << endl;
  cerr << "_camera_image_topic: " << c.camera_image_topic << endl;
  cerr << "_pointcloud_topic: " << c.pointcloud_topic << endl;
  cerr << "_laser_topic: " <<  c.laser_topic << endl;
  cerr << "_publish_pointcloud: " << c.publish_pointcloud << endl;
  cerr << "_num_ranges" <<  c.num_ranges << endl;

}

//================================================================================
//MAIN PROGRAM
//================================================================================
int main(int argc, char **argv){
    ros::init(argc, argv, "depth2laser");
    ros::NodeHandle n("~");
    listener= new tf::TransformListener();
    //Getting and setting parameters
    n.param("angle_min", c.angle_min, -M_PI/2);
    n.param("angle_max", c.angle_max, M_PI/2);
    n.param("num_ranges", c.num_ranges, 1024);
    n.param("range_min", c.range_min, 0.1);
    n.param("range_max", c.range_max, 10.0);
    n.param("laser_plane_thickness", c.laser_plane_thickness, 0.05);
    n.param<string>("base_frame_id", c.base_frame_id, "/base_link");
    n.param<string>("laser_frame_id", c.laser_frame_id, "/laser_frame");
    n.param<string>("camera_image_topic", c.camera_image_topic, "/camera/depth/image_raw");
    n.param<string>("camera_info_topic", c.camera_info_topic, "/camera/depth/camera_info");
    n.param<string>("pointcloud_topic", c.pointcloud_topic, "/pointcloud");
    n.param<string>("laser_topic", c.laser_topic, "/scan");
    n.param("publish_pointcloud", c.publish_pointcloud, 0);
    n.param("num_ranges", c.num_ranges, 1024);
    EchoParameters();
    //Messages headers for TF
    //================================================================================
    clearScan();

    //Subscribers
    //================================================================================
    frame_sub = n.subscribe(c.camera_image_topic, 1, frameCallback);
    info_sub = n.subscribe(c.camera_info_topic, 1, infoCallback);
    //Publishers
    //================================================================================
    cloud_pub = n.advertise<sensor_msgs::PointCloud>(c.pointcloud_topic, 1);
    laser_pub = n.advertise<sensor_msgs::LaserScan>(c.laser_topic, 1);
    ros::spin();

    return 0;
}
