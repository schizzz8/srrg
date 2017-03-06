#pragma once
#include "base_camera_info.h"

namespace srrg_core_map {
  /**
     Spherical projection camera parameters
     Encapsulated in a 4x1 vector
     v_0=horizontal_fov (radians)
     v_1=vertical_fov (radians)
     v_2=horizontal_resolution (pixel/radians)
     v_3=vertical_resolution (pixel/radians)
   */
  class SphericalCameraInfo : public BaseCameraInfo {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SphericalCameraInfo(const std::string& topic_ = "none",
			const std::string& frame_id = "",
			const Eigen::Vector4f& camera_matrix=Eigen::Vector4f(M_PI/2, M_PI/4, 180/M_PI, 90/M_PI),
			const Eigen::Isometry3f&offset_ = Eigen::Isometry3f::Identity(),
			float depth_scale_ = 1e-3,
			int id=-1,
			srrg_boss::IdContext* context=0);

    
    virtual BaseCameraInfo* scale(float s);
    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    inline const Eigen::Vector4f& cameraMatrix() const {return _camera_matrix;}
    inline void setCameraMatrix(const Eigen::Vector4f& camera_matrix_)  {_camera_matrix = camera_matrix_;}
  protected:
    Eigen::Vector4f _camera_matrix;
  };

}
