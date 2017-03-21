#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <srrg_txt_io/message_writer.h>
#include <srrg_txt_io/base_message.h>
#include <srrg_txt_io/joint_state_message.h>
#include <srrg_txt_io/pinhole_image_message.h>
#include <srrg_txt_io/laser_message.h>
#include <srrg_txt_io/imu_message.h>
#include <srrg_txt_io/spherical_image_message.h>

//ds ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

//ds opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace srrg_core;

//ds desired precision
typedef float real;
typedef Eigen::Matrix<real, 3, 3> Matrix3;
typedef Eigen::Matrix<real, 3, 1> Vector3;
typedef cv::Vec<real, 5> Vector5;
typedef Eigen::Quaternion<real> Quaternion;
typedef Eigen::Matrix<real, 3, 4> ProjectionMatrix;
typedef Eigen::Transform<real, 3, Eigen::Isometry> TransformMatrix3D;

//ds entry point
int32_t main(int32_t argc, char** argv) {

  //ds arguments at least 2 - optional topic names
  if (argc < 3) {
    std::cerr << "ERROR: invalid call - use ./srrg_txt_io_converter_visensor_app -f <file_name_ros_bag>.bag (in the respective visensor rosbag folder)" << std::endl;
    return 0;
  }

  //ds obtain configuration
  std::string file_name_ros_bag              = "";
  std::string topic_name_camera_info_left    = "/thin_visensor_node/camera_left/camera_info";
  std::string topic_name_camera_left         = "/thin_visensor_node/camera_left/image_raw";
  std::string topic_name_camera_left_to_imu  = "/thin_visensor_node/camera_left/pose_to_imu_adis16448";
  std::string topic_name_camera_info_right   = "/thin_visensor_node/camera_right/camera_info";
  std::string topic_name_camera_right        = "/thin_visensor_node/camera_right/image_raw";
  std::string topic_name_camera_right_to_imu = "/thin_visensor_node/camera_right/pose_to_imu_adis16448";
  std::string topic_name_imu                 = "/thin_visensor_node/imu_adis16448";
  std::string topic_name_magnetic_field      = "/thin_visensor_node/magnetic_field";
  std::string topic_name_pressure            = "/thin_visensor_node/pressure";
  int32_t count_added_arguments = 1;
  while(count_added_arguments < argc){
    if (! strcmp(argv[count_added_arguments],"-f")){
      count_added_arguments++;
      file_name_ros_bag      = argv[count_added_arguments];
    }
    count_added_arguments++;
  }

  //ds if elementary input is not set
  if (file_name_ros_bag == "") {
    std::cerr << "ERROR: parameter -f <file_name_ros_bag>.bag not set" << std::endl;
    return 0;
  }

  //ds attempt to open the ROS bag - may throw BagException
  rosbag::Bag bag(file_name_ros_bag);
  const std::string file_name_ros_bag_clean = file_name_ros_bag.substr(0, file_name_ros_bag.length()-4);

  //ds outfile configuration
  const std::string filename_txt_io(file_name_ros_bag_clean+".txt");
  const std::string folder_txt_io_images(filename_txt_io+".d/");

  //ds check file input
  std::cerr << "ROS bag: " << file_name_ros_bag << std::endl;
  std::cerr << "topic name camera info left: " << topic_name_camera_info_left << std::endl;
  std::cerr << "                camera left: " << topic_name_camera_left << std::endl;
  std::cerr << "    pose camera left to imu: " << topic_name_camera_left_to_imu << std::endl;
  std::cerr << "          camera info right: " << topic_name_camera_info_right << std::endl;
  std::cerr << "               camera right: " << topic_name_camera_right << std::endl;
  std::cerr << "   pose camera right to imu: " << topic_name_camera_right_to_imu << std::endl;
  std::cerr << "                        imu: " << topic_name_imu << std::endl;
  std::cerr << "             magnetic field: " << topic_name_magnetic_field << std::endl;
  std::cerr << "                   pressure: " << topic_name_pressure << std::endl;
  std::cerr << "txt_io output messages: " << filename_txt_io << std::endl;
  std::cerr << "                images: " << folder_txt_io_images << std::endl;

  //ds attempt to create the image directory - and check for failure
  if (mkdir(folder_txt_io_images.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
    std::cerr << "ERROR: unable to create image directory: " << folder_txt_io_images << " (maybe already existing?)" << std::endl;
    return 0;
  }

  //ds obtain a bag viewer
  rosbag::View bag_viewer(bag);

  //ds open a message writer in the current directory
  MessageWriter txt_io_message_writer;
  txt_io_message_writer.open(filename_txt_io);

  //ds info
  uint64_t number_of_messages = 0;

  //ds buffered components - set once and assumed to be constant over the whole dataset
  cv::Mat_<real> camera_matrix_left(3, 3);   camera_matrix_left = 0;
  cv::Mat_<real> camera_matrix_right(3, 3); camera_matrix_right = 0;
  TransformMatrix3D camera_left_to_imu(TransformMatrix3D::Identity());
  TransformMatrix3D camera_right_to_imu(TransformMatrix3D::Identity());
  cv::Mat_<real> projection_matrix_left(3, 4);  projection_matrix_left  = 0;
  cv::Mat_<real> projection_matrix_right(3, 4); projection_matrix_right = 0;
  Vector5 distortion_coefficients_left;   distortion_coefficients_left = 0;
  Vector5 distortion_coefficients_right; distortion_coefficients_right = 0;
  cv::Mat_<real> rectification_matrix_left(3, 3);  rectification_matrix_left  = 0;
  cv::Mat_<real> rectification_matrix_right(3, 3); rectification_matrix_right = 0;
  uint32_t image_width_pixels  = 0;
  uint32_t image_height_pixels = 0;

  //ds loop over the bag looking for the components to buffer
  for (rosbag::View::iterator itMessage = bag_viewer.begin(); itMessage != bag_viewer.end(); ++itMessage) {

    const ros::Time timestamp = itMessage->getTime();
    std::string message_topic = itMessage->getTopic();

    //ds match message to topic - UGLY UGLY TODO collapse this whole switch bomb
    if (message_topic == topic_name_camera_info_left) {
      sensor_msgs::CameraInfo::ConstPtr ros_message = itMessage->instantiate<sensor_msgs::CameraInfo>();
      image_width_pixels  = ros_message->width;
      image_height_pixels = ros_message->height;

      //ds set camera matrix
      for(uint32_t row = 0; row < 3; ++row) {
        for(uint32_t col = 0; col < 3; ++col) {
          camera_matrix_left(row,col) = ros_message->K[3*row+col];
        }
      }

      //ds set projection matrix
      for(uint32_t row = 0; row < 3; ++row) {
        for(uint32_t col = 0; col < 4; ++col) {
          projection_matrix_left(row,col) = ros_message->P[4*row+col];
        }
      }

      //ds set distortion coefficients
      for(uint32_t u = 0; u < 5; ++u) {
        distortion_coefficients_left[u] = ros_message->D[u];
      }

      //ds set rectification matrix
      for(uint32_t row = 0; row < 3; ++row) {
        for(uint32_t col = 0; col < 3; ++col) {
          rectification_matrix_left(row,col) = ros_message->R[3*row+col];
        }
      }
    } else if (message_topic == topic_name_camera_left_to_imu) {
      geometry_msgs::Pose::ConstPtr ros_message = itMessage->instantiate<geometry_msgs::Pose>();

      //ds buffer transform components
      Vector3 translation(ros_message->position.x, ros_message->position.y, ros_message->position.z);
      Quaternion rotation(ros_message->orientation.w, ros_message->orientation.x, ros_message->orientation.y, ros_message->orientation.z);

      //ds set full transform
      camera_left_to_imu.translation() = translation;
      camera_left_to_imu.linear()      = rotation.toRotationMatrix();
    } else if (message_topic == topic_name_camera_info_right) {
      sensor_msgs::CameraInfo::ConstPtr ros_message = itMessage->instantiate<sensor_msgs::CameraInfo>();
      image_width_pixels  = ros_message->width;
      image_height_pixels = ros_message->height;

      //ds set camera matrix
      for(uint32_t row = 0; row < 3; ++row) {
        for(uint32_t col = 0; col < 3; ++col) {
          camera_matrix_right(row,col) = ros_message->K[3*row+col];
        }
      }

      //ds set projection matrix
      for(uint32_t row = 0; row < 3; ++row) {
        for(uint32_t col = 0; col < 4; ++col) {
          projection_matrix_right(row,col) = ros_message->P[4*row+col];
        }
      }

      //ds set distortion coefficients
      for(uint32_t u = 0; u < 5; ++u) {
        distortion_coefficients_right[u] = ros_message->D[u];
      }

      //ds set rectification matrix
      for(uint32_t row = 0; row < 3; ++row) {
        for(uint32_t col = 0; col < 3; ++col) {
          rectification_matrix_right(row,col) = ros_message->R[3*row+col];
        }
      }
    } /*else if (message_topic == topic_name_camera_right_to_imu) {
      geometry_msgs::Pose::ConstPtr ros_message = itMessage->instantiate<geometry_msgs::Pose>();

      //ds buffer transform components
      Vector3 translation(ros_message->position.x, ros_message->position.y, ros_message->position.z);
      Quaternion rotation(ros_message->orientation.w, ros_message->orientation.x, ros_message->orientation.y, ros_message->orientation.z);

      //ds set full transform
      camera_right_to_imu.translation() = translation;
      camera_right_to_imu.linear()      = rotation.toRotationMatrix();
    }*/

    //ds as soon as we have all the essential information required
    if (cv::norm(camera_matrix_left)  != 0&&
        cv::norm(camera_matrix_right) != 0) {
      break;
    }
  }

  //ds check if a value has not been set
  if (cv::norm(camera_matrix_left) == 0) {
    std::cerr << "ERROR: camera matrix left not set" << std::endl;
    return 0;
  }
  if (cv::norm(camera_matrix_right) == 0) {
    std::cerr << "ERROR: camera matrix right not set" << std::endl;
    return 0;
  }
  if (camera_left_to_imu.translation().norm() == 0) {
    std::cerr << "WARNING: transform camera left to IMU not set - Press [ENTER] to continue" << std::endl;
    getchar();
  }
//  if (camera_right_to_imu.translation().norm() == 0) {
//    std::cerr << "ERROR: transform camera right to IMU not set" << std::endl;
//    return 0;
//  }

  //ds undistortion/rectification
  cv::Mat undistort_rectify_maps_left[2];
  cv::Mat undistort_rectify_maps_right[2];

  //ds compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(camera_matrix_left,
                              distortion_coefficients_left,
                              rectification_matrix_left,
                              projection_matrix_left,
                              cv::Size(image_width_pixels, image_height_pixels),
                              CV_16SC2,
                              undistort_rectify_maps_left[0],
                              undistort_rectify_maps_left[1]);
  cv::initUndistortRectifyMap(camera_matrix_right,
                              distortion_coefficients_right,
                              rectification_matrix_right,
                              projection_matrix_right,
                              cv::Size(image_width_pixels, image_height_pixels),
                              CV_16SC2,
                              undistort_rectify_maps_right[0],
                              undistort_rectify_maps_right[1]);

  //ds check if rectification failed
  if (cv::norm(undistort_rectify_maps_left[0]) == 0 || cv::norm(undistort_rectify_maps_left[1]) == 0) {
    std::cerr << "ERROR: unable to undistort and rectify camera left" << std::endl;
    return 0;
  }
  if (cv::norm(undistort_rectify_maps_right[0]) == 0 || cv::norm(undistort_rectify_maps_right[1]) == 0) {
    std::cerr << "ERROR: unable to undistort and rectify camera right" << std::endl;
    return 0;
  }


  //ds get camera matrices to eigen space for convenience
  Matrix3 camera_matrix_eigen(Matrix3::Zero());
  for(uint32_t row = 0; row < 3; ++row) {
    for(uint32_t col = 0; col < 3; ++col) {
      camera_matrix_eigen(row,col)  = projection_matrix_left(row,col);
    }
  }

  //ds get right projection matrix to eigen space
  ProjectionMatrix projection_matrix_right_eigen(ProjectionMatrix::Zero());
  for(uint32_t row = 0; row < 3; ++row) {
    for(uint32_t col = 0; col < 4; ++col) {
      projection_matrix_right_eigen(row,col) = projection_matrix_right(row,col);
    }
  }

  //ds compute offset for right camera in order to reconstruct projection matrix form txt_io message
  const Vector3 offset(camera_matrix_eigen.fullPivLu().solve(projection_matrix_right_eigen.block<3,1>(0,3)));
  TransformMatrix3D camera_left_to_right(TransformMatrix3D::Identity());
  camera_left_to_right.translation() = -offset;

  //ds loop over the bag - writing to disk
  for (rosbag::View::iterator itMessage = bag_viewer.begin(); itMessage != bag_viewer.end(); ++itMessage) {

    const ros::Time timestamp = itMessage->getTime();
    std::string message_topic = itMessage->getTopic();

    //ds match message to topic - UGLY UGLY TODO collapse this whole switch bomb
    if (message_topic == topic_name_camera_left || message_topic == topic_name_camera_right) {
      sensor_msgs::ImageConstPtr ros_message         = itMessage->instantiate<sensor_msgs::Image>();
      srrg_core::PinholeImageMessage* txt_io_message = new srrg_core::PinholeImageMessage();
      txt_io_message->setSeq(ros_message->header.seq);
      txt_io_message->setTimestamp(ros_message->header.stamp.toSec());
      txt_io_message->setFrameId(ros_message->header.frame_id);
      txt_io_message->setTopic(message_topic);

      //ds copy image data
      cv_bridge::CvImagePtr image_handler;
      image_handler = cv_bridge::toCvCopy(ros_message, ros_message->encoding);
      const cv::Mat image_raw(image_handler->image);
      cv::Mat image_undistorted_rectified;

      //ds if left image
      if (message_topic == topic_name_camera_left) {
        txt_io_message->setCameraMatrix(camera_matrix_eigen);
        txt_io_message->setOffset(camera_left_to_imu.cast<float>());

        //ds rectify image
        cv::remap(image_raw, image_undistorted_rectified, undistort_rectify_maps_left[0], undistort_rectify_maps_left[1], cv::INTER_LINEAR);

      } else {
        txt_io_message->setCameraMatrix(camera_matrix_eigen);
        txt_io_message->setOffset(camera_left_to_right.cast<float>());

        //ds rectify image
        cv::remap(image_raw, image_undistorted_rectified, undistort_rectify_maps_right[0], undistort_rectify_maps_right[1], cv::INTER_LINEAR);
      }

      //ds equalize histogram
      cv::equalizeHist(image_undistorted_rectified, image_undistorted_rectified);

      //ds set image to message
      txt_io_message->setImage(image_undistorted_rectified);
      txt_io_message->setBinaryFilePrefix(folder_txt_io_images);

      //ds save message to disk
      imwrite((folder_txt_io_images+txt_io_message->binaryFullFilename()).c_str(), txt_io_message->image());
      txt_io_message_writer.writeMessage(*txt_io_message);
      delete txt_io_message;
    } else if (message_topic == topic_name_imu) {
      sensor_msgs::ImuConstPtr ros_message   = itMessage->instantiate<sensor_msgs::Imu>();
      srrg_core::CIMUMessage* txt_io_message = new srrg_core::CIMUMessage();
      txt_io_message->setSeq(ros_message->header.seq);
      txt_io_message->setTimestamp(ros_message->header.stamp.toSec());
      txt_io_message->setFrameId(ros_message->header.frame_id);
      txt_io_message->setTopic(message_topic);

      //ds checked
      Eigen::Vector3d angular_velocity(ros_message->angular_velocity.x, ros_message->angular_velocity.y, ros_message->angular_velocity.z);
      txt_io_message->setAngularVelocity(angular_velocity);
      Eigen::Vector3d linear_acceleration(ros_message->linear_acceleration.x, ros_message->linear_acceleration.y, ros_message->linear_acceleration.z);
      txt_io_message->setLinearAcceleration(linear_acceleration);
      Eigen::Quaterniond orientation(ros_message->orientation.w, ros_message->orientation.x, ros_message->orientation.y, ros_message->orientation.z);
      txt_io_message->setOrientation(orientation);

      //ds save message to disk
      txt_io_message_writer.writeMessage(*txt_io_message);
      delete txt_io_message;

    } else if (message_topic == topic_name_magnetic_field) {

      //ds not implemented yet in txt_io

    } else if (message_topic == topic_name_pressure) {

      //ds not implemented yet in txt_io

    }

    //ds progress info
    std::cerr << "x";
    ++number_of_messages;
  }
  std::cerr << std::endl;
  std::cerr << "converted messages: " << number_of_messages << std::endl;
  bag.close();
  return 0;
}
