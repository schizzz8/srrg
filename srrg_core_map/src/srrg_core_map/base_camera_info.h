#pragma once

#include <srrg_types/defs.h>

#include <srrg_boss/identifiable.h>

namespace srrg_core_map {

  class BaseCameraInfo : public srrg_boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    BaseCameraInfo(const std::string& topic_ = "none",
		   const std::string& frame_id = "",
		   const Eigen::Isometry3f&offset_ = Eigen::Isometry3f::Identity(),
		   float depth_scale_ = 1e-3,
		   int id=-1,
		   srrg_boss::IdContext* context=0);

    inline const std::string& topic () const {return _topic;}
    inline void setTopic (const std::string& topic_)  {_topic = topic_;}
    inline const std::string& frameId () const {return _frame_id;}
    inline void setFrameId (const std::string& fid)  { _frame_id = fid;}
    inline const Eigen::Isometry3f& offset () const {return _offset;}
    inline void setOffset (const Eigen::Isometry3f& offset_)  {_offset = offset_;}
    inline float depthScale () const {return _depth_scale;}
    inline void setDepthScale (float ds)  {_depth_scale = ds;}

    virtual BaseCameraInfo* scale(float s);
    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
  protected:
    std::string _frame_id;
    std::string _topic;
    Eigen::Isometry3f _offset;
    float _depth_scale;
  };

}


