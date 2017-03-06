#pragma once
#include <string>
#include <iostream>
#include "base_image_message.h"
#include <Eigen/Core>

namespace srrg_core {

  class PinholeImageMessage: public BaseImageMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PinholeImageMessage(const std::string& topic_="", const std::string& frame_id_="", int seq_=-1, double timestamp_=-1);
    virtual const std::string& tag() const;
    virtual void fromStream(std::istream& is_);
    virtual void toStream(std::ostream& os_) const;
    inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix; }
    inline void setCameraMatrix(const Eigen::Matrix3f& camera_matrix_) {_camera_matrix = camera_matrix_;}
  protected:
    static const std::string _tag;
    Eigen::Matrix3f _camera_matrix;
  };

}
