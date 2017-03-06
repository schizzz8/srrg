#pragma once

#include "base_camera_info.h"
#include "map_node.h"

namespace srrg_core_map {

  class ImageMapNode : public MapNode{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageMapNode(const Eigen::Isometry3f& transform=Eigen::Isometry3f::Identity(), 
		 BaseCameraInfo* cam=0,
		 const std::string& topic = "",
		 int seq_=-1,
		 int id=-1,
		 srrg_boss::IdContext* context=0);

    inline const std::string& topic() const {return _topic; }
    inline void setTopic (const std::string topic_) {_topic = topic_;}

    inline int seq() const {return _seq; }
    inline void setSeq(int s)  { _seq = s; }

    inline BaseCameraInfo* cameraInfo() {return _camera_info;}
    inline void setCameraInfo(BaseCameraInfo* c)  { _camera_info = c;}
    virtual void draw(srrg_gl_helpers::DrawAttributesType attributes = ATTRIBUTE_SHOW, int name = -1);

    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);

  protected:
    std::string _topic;
    BaseCameraInfo* _camera_info;
    int _seq;
  };

}
