#include "base_camera_info.h"

namespace srrg_core_map {

  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_core;

  BaseCameraInfo::BaseCameraInfo(const std::string& topic_,
				 const std::string& frame_id,
				 const Eigen::Isometry3f&offset_,
				 float depth_scale_,
				 int id,
				 IdContext* context):   Identifiable(id,context) {
    _offset = offset_;
    _topic = topic_;
    _frame_id = frame_id;
    _depth_scale = depth_scale_;
  }

  BaseCameraInfo* BaseCameraInfo::scale(float ) { return 0; }

  void BaseCameraInfo::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    data.setString("topic",_topic);
    data.setFloat("depth_scale",_depth_scale);
    t2v(_offset).toBOSS(data, "offset");
  }

  void BaseCameraInfo::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    _topic = data.getString("topic");
    _depth_scale = data.getFloat("depth_scale");
    Vector6f v;
    v.fromBOSS(data, "offset");
    _offset = v2t(v);
  }

  BOSS_REGISTER_CLASS(BaseCameraInfo);

}


