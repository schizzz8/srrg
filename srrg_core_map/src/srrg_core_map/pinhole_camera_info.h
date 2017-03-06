#pragma once
#include "base_camera_info.h"

namespace srrg_core_map {
  class PinholeCameraInfo : public BaseCameraInfo {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PinholeCameraInfo(const std::string& topic_ = "none",
		      const std::string& frame_id = "",
		      const Eigen::Matrix3f& camera_matrix  = Eigen::Matrix3f::Identity(),
		      const Eigen::Isometry3f&offset_ = Eigen::Isometry3f::Identity(),
		      float depth_scale_ = 1e-3,
		      int id=-1,
		      srrg_boss::IdContext* context=0);
    inline const Eigen::Matrix3f& cameraMatrix () const {return _camera_matrix;}
    inline void  setCameraMatrix (const Eigen::Matrix3f& cm)  {_camera_matrix = cm;}
    
    virtual BaseCameraInfo* scale(float s);
    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    
  protected:
    Eigen::Matrix3f _camera_matrix;
  };

}
