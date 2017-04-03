#pragma once
#include <opencv2/core/core.hpp>
#include <string>
#include "srrg_txt_io/laser_message.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace srrg_depth2laser{

  class Depth2Laser{
  public:
    Depth2Laser(std::string filename);
    void setParameters(int seq,double timestamp,srrg_core::LaserMessage& laser_msg);
    void compute(const cv::Mat& image,srrg_core::LaserMessage& laser_msg);

    float minRange(){return _min_range;}
    float maxRange(){return _max_range;}
    float minAngle(){return _min_angle;}
    float maxAngle(){return _max_angle;}
    float angleIncrement(){return _angle_increment;}

    inline void setK(Eigen::Matrix3f K_){_K=K_;_invK=_K.inverse();}
    inline Eigen::Matrix3f K(){return _K;}
    inline void setCameraTransform(const Eigen::Isometry3f& camera_transform_){_camera_transform=camera_transform_;}
    inline Eigen::Isometry3f cameraTransform(){return _camera_transform;}
    inline void setLaserTransform(const Eigen::Isometry3f& laser_transform_ = Eigen::Isometry3f::Identity()){
        _laser_transform = laser_transform_;
        _laser_transform.translation() = _camera_transform.translation();
//        _laser_transform.translation() = Eigen::Vector3f::Zero();
//        _laser_transform.translation().x() = 0.104013;
//        _laser_transform.translation().y() = 0.00379057;
        camera2laser_transform = _laser_transform.inverse()*_camera_transform;
    }
    inline Eigen::Isometry3f laserTransform(){return _laser_transform;}
  protected:
    std::string _topic_name;
    std::string _frame_id;
    int _num_ranges;
    float _min_range;
    float _max_range;
    float _min_angle;
    float _max_angle;
    float _angle_increment;
    float _inverse_angle_increment;
    float _laser_plane_thickness;
    float _squared_max_norm;
    float _squared_min_norm;
    Eigen::Matrix3f _K;
    Eigen::Matrix3f _invK;
    Eigen::Isometry3f _camera_transform;
    Eigen::Isometry3f _laser_transform;
    Eigen::Isometry3f camera2laser_transform;

  };
}
