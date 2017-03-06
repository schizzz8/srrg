#include <iostream>
#include <GL/gl.h>

#include "binary_node_relation.h"
#include <srrg_gl_helpers/opengl_primitives.h>

namespace srrg_core_map {
  
  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_core;
  using namespace srrg_gl_helpers;
  
  BinaryNodeRelation::BinaryNodeRelation(MapNode* from_,
					 MapNode* to_,
					 const Eigen::Isometry3f& transform_, 
					 const Matrix6f& info, 
					 int id,
					 IdContext* context): Identifiable(id,context){
    _from = from_;
    _to = to_;
    _parent = 0;
    _transform = transform_;
    _information_matrix=info;
  }
  
  void BinaryNodeRelation::draw(DrawAttributesType attributes, int name){

    if (! _from || !_to)
      return;

    Eigen::Isometry3f equilibrium = _from->transform()*_transform;
    // glPushMatrix();
    // gl_helpers::glMultMatrix(equilibrium);
    // glScalef(0.05, 0.05, 0.05);
    // gl_helpers::drawReferenceSystem();
    // glPopMatrix();
    
    Eigen::Vector3f start = _from->transform().translation();
    Eigen::Vector3f middle = equilibrium.translation();
    Eigen::Vector3f end = _to->transform().translation();

    
    glPushAttrib(GL_LINE_WIDTH|GL_COLOR);
    glLineWidth(1);
    glColor3f(0,0,0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(start.x(), start.y(), start.z());
    glVertex3f(middle.x(), middle.y(), middle.z());
    glVertex3f(end.x(), end.y(), end.z());
    glEnd();
    glPopAttrib();

  }

  BinaryNodeRelation::~BinaryNodeRelation(){
  }

  void BinaryNodeRelation::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    data.setPointer("from", _from);
    data.setPointer("to", _to);
    data.setPointer("parent", _parent);
    t2v(_transform).toBOSS(data,"transform");
    _information_matrix.toBOSS(data,"information_matrix");
  }
  
  void BinaryNodeRelation::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    data.getReference("from").bind(_from);
    data.getReference("to").bind(_to);
    data.getReference("parent").bind(_parent);
    Vector6f v;
    v.fromBOSS(data,"transform");
    _transform = v2t(v);
    _information_matrix.fromBOSS(data,"information_matrix");
  }


  BOSS_REGISTER_CLASS(BinaryNodeRelation);

}
