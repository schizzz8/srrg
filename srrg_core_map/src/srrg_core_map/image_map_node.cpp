#include <iostream>
#include <GL/gl.h>

#include "image_map_node.h"
#include <srrg_gl_helpers/opengl_primitives.h>

namespace srrg_core_map {
  
  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_gl_helpers;

  ImageMapNode::ImageMapNode(const Eigen::Isometry3f& t, 
			     BaseCameraInfo* cam,
			     const std::string& topic_,
			     int seq_,
			     int id,
			     IdContext* context) : MapNode(t, id, context){
    _topic = topic_;
    _camera_info = cam;
    _seq = seq_;
  }


  void ImageMapNode::serialize(ObjectData& data, IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("camera_info", _camera_info);
    data.setString("topic", _topic);
    data.setInt("seq",_seq);
  }

  void ImageMapNode::deserialize(ObjectData& data, IdContext& context){
    MapNode::deserialize(data,context);
    data.getReference("camera_info").bind(_camera_info);
    _topic = data.getString("topic");
    _seq = data.getInt("seq");
  }

  void ImageMapNode::draw(DrawAttributesType attributes, int name) {
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
    glColor3f(0.56f, 0.0f, 1.0f);
    drawPyramidWireframe(0.02, 0.01);
    glPopMatrix();
    if (name>-1)
      glPopName();
  }


  BOSS_REGISTER_CLASS(ImageMapNode);

}
