#include "ros_localizer.h"

using namespace std;
using namespace srrg_localizer2d_ros;



void startLocalizer(ROSLocalizer* localizer, ros::NodeHandle& private_nh) {
  double tmp;

  cerr << "Thin Localizer Parameters: " << endl;
  int particles;
  private_nh.param("particles", particles, 2000);
  cerr << "[int] _particles: " << particles << endl;

  double max_range;
  private_nh.param("max_range", max_range, 10.0);
  cerr << "[float] _max_range: " << max_range << endl;

  double min_weight;
  private_nh.param("min_weight", min_weight, 200.0);
  cerr << "[float] _min_weight: " << min_weight << endl;

  double distance_threshold;
  private_nh.param("distance_threshold", distance_threshold, 1.0);
  cerr << "[float] _distance_threshold: " << distance_threshold << endl;

  double endpoint_distance;
  private_nh.param("endpoint_distance", endpoint_distance, 0.1);
  cerr << "[float] _endpoint_distance: " << endpoint_distance << endl;
  
  bool dynamic_restart;
  private_nh.param("dynamic_restart", dynamic_restart, false);
  cerr << "[bool] _dynamic_restart: " << dynamic_restart << endl;

  bool use_gui;
  private_nh.param("use_gui", use_gui, false);
  cerr << "[bool] _use_gui: " << use_gui << endl;

  std::string odom_frame_id;
  private_nh.param("odom_frame_id", odom_frame_id, std::string("odom"));
  localizer->setOdomFrameId(odom_frame_id);
  cerr << "[string] _odom_frame_id: " << odom_frame_id << endl;

  std::string base_frame_id;
  private_nh.param("base_frame_id", base_frame_id, std::string("base_link"));
  localizer->setBaseFrameId(base_frame_id);
  cerr << "[string] _base_frame_id: " << base_frame_id << endl;

  std::string global_frame_id;
  private_nh.param("global_frame_id", global_frame_id, std::string("map"));
  localizer->setGlobalFrameId(global_frame_id);
  cerr << "[string] _global_frame_id: " << global_frame_id << endl;

  std::string laser_topic;
  private_nh.param("laser_topic", laser_topic, std::string("base_scan"));
  cerr << "[string] _laser_topic: " << laser_topic << endl;


  Eigen::Vector3d initial_pose(0,0,0);
  bool has_initial_pose=private_nh.hasParam("initial_pose_x");
  if (has_initial_pose) {
    private_nh.param("initial_pose_x", initial_pose.x(), 0.0);
    private_nh.param("initial_pose_y", initial_pose.y(), 0.0);
    private_nh.param("initial_pose_theta", initial_pose.z(), 0.0);
    cerr << "[float] _initial_pose_x: " << initial_pose.x() << endl;
    cerr << "[float] _initial_pose_y: " << initial_pose.y() << endl;
    cerr << "[float] _initial_pose_theta: " << initial_pose.z() << endl;
  } else {
    cerr << "[float] _initial pose_{x,y,a}: not set, starting global localization"<< endl;
  }
      

  localizer->init(particles, distance_threshold, 0.2, min_weight,10);
  localizer->setSquaredEndpointDistance(endpoint_distance*endpoint_distance);
  if (use_gui)
    localizer->initGUI();

  localizer->setParticleResetting(dynamic_restart);
  localizer->subscribeCallbacks(laser_topic);

  if  (has_initial_pose){
    localizer->setInitialPose(float(initial_pose[0]),float(initial_pose[1]),float(initial_pose[2]));
  } else 
    localizer->startGlobal();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "thin_localizer_node");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");


  //constructs the localizer
  ROSLocalizer* localizer=new ROSLocalizer(n);

  //requests the map
  localizer->requestMap();

  
  if (ros::ok())
    startLocalizer(localizer, private_nh);


  //run baby run
  ros::spin();
  return 0;
}
