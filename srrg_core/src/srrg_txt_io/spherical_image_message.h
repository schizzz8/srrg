#pragma once
#include <string>
#include <iostream>
#include "base_image_message.h"
#include <Eigen/Core>

namespace srrg_core {

  class SphericalImageMessage: public BaseImageMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SphericalImageMessage(const std::string& topic="", const std::string& frame_id="", int seq=-1, double timestamp=-1);
    virtual const std::string& tag() const;
    virtual void fromStream(std::istream& is);
    virtual void toStream(std::ostream& os) const;

    //! v_0=horizontal_fov (radians)
    //! v_1=vertical_fov (radians)
    //! v_2=horizontal_resolution (pixel/radians)
    //! v_3=vertical_resolution (pixel/radians)
 
    inline const Eigen::Vector4f& cameraMatrix() const {return _camera_matrix; }
    inline void setCameraMatrix(const Eigen::Vector4f& camera_matrix) {_camera_matrix = camera_matrix;}
  protected:
    static const std::string _tag;
    Eigen::Vector4f _camera_matrix;
  };

}
