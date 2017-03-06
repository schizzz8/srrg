#pragma once
#include "base_camera_info.h"

namespace srrg_core_map {

  class MultiCameraInfo : public BaseCameraInfo {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
     MultiCameraInfo(const std::string& topic_ = "none",
		     const std::string& frame_id = "",
		     const Eigen::Isometry3f&offset_ = Eigen::Isometry3f::Identity(),
		     float depth_scale_ = 1e-3,
		     int id=-1,
		     srrg_boss::IdContext* context=0);
    
     virtual BaseCameraInfo* scale(float s);
     
     virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
     virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
     virtual void deserializeComplete();

    inline std::vector<BaseCameraInfo*>& cameraInfos() {return  _camera_infos;}
    ~MultiCameraInfo();
    
  protected:
    std::vector<BaseCameraInfo*> _camera_infos; // these are owned by the object
    std::vector<srrg_boss::Identifiable*>  _pending_cameras;
  };

}
