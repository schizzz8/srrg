#include <iostream>
#include <GL/gl.h>

#include "multi_image_map_node.h"
#include <srrg_gl_helpers/opengl_primitives.h>

namespace srrg_core_map {

  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_gl_helpers;

  MultiImageMapNode::MultiImageMapNode(const Eigen::Isometry3f& t, 
			     MultiCameraInfo* cam,
			     const std::string& topic_,
			     int seq_,
			     int id,
			     IdContext* context) : MapNode(t, id, context){
    _topic = topic_;
    _camera_info = cam;
    _seq = seq_;
  }


  void MultiImageMapNode::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("camera_info", _camera_info);
    data.setString("topic", _topic);
    data.setInt("seq",_seq);
    ArrayData* subimages_array = new ArrayData;
    for (size_t i = 0; i<_subimage_seqs.size(); i++)
      subimages_array->add(_subimage_seqs[i]);
    data.setField("subimages_seq", subimages_array);
  }

  void MultiImageMapNode::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
    data.getReference("camera_info").bind(_camera_info);
    _topic = data.getString("topic");
    _seq = data.getInt("seq");
    ArrayData& subimages_array = data.getField("subimages_seq")->getArray();
    _subimage_seqs.resize(subimages_array.size());
    for (size_t i =0; i< subimages_array.size(); i++){
      _subimage_seqs[i] = subimages_array[i].getInt();
    }
  }

  void MultiImageMapNode::draw(DrawAttributesType attributes, int name) {
    if (! (attributes&ATTRIBUTE_SHOW))
      return;
    if (name>-1)
      glPushName(name);

    glPushMatrix();
    glMultMatrix(_transform);
    glScalef(0.1, 0.1, 0.1);
    drawReferenceSystem();    
    glPopMatrix();

    glPushMatrix();
    Eigen::Isometry3f cameraPose = _transform*_camera_info->offset();
    glMultMatrix(cameraPose);
    for (size_t i = 0; i< _camera_info->cameraInfos().size(); i++){
      glPushMatrix();
      glMultMatrix(_camera_info->cameraInfos()[i]->offset());
      glColor3f(0.56f, 0.0f, 1.0f);
      drawPyramidWireframe(0.05, 0.025);
      glPopMatrix();
    }
    glPopMatrix();
    if (name>-1)
      glPopName();
  }


  BOSS_REGISTER_CLASS(MultiImageMapNode);

}
