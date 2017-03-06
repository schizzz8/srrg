#include <iostream>
#include <GL/gl.h>

#include "local_map.h"
#include <srrg_gl_helpers/opengl_primitives.h>

namespace srrg_core_map {

  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_gl_helpers;
  
  LocalMap::LocalMap(const Eigen::Isometry3f& transform, 
		     int id,
		     IdContext* context) : MapNode(transform, id, context){
  }

  LocalMap::~LocalMap(){
    _cloud_ref.set(0);
  }
  
  void LocalMap::serialize(ObjectData& data, IdContext& context) {
    MapNode::serialize(data,context);
    ObjectData * blobData=new ObjectData();
    data.setField("cloud", blobData);
    _cloud_ref.serialize(*blobData,context);

    ArrayData* nodesArray = new ArrayData;
    for (MapNodeList::iterator it = _nodes.begin(); it!=_nodes.end(); it++){
      MapNode* node=it->get();
      nodesArray->add(new PointerData(node));
    }
    data.setField("nodes", nodesArray);

    ArrayData* relationsArray = new ArrayData;
    for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++){
      BinaryNodeRelation* rel=it->get();
      relationsArray->add(new PointerData(rel));
    }
    data.setField("relations", relationsArray);

  }
  
  void LocalMap::deserialize(ObjectData& data, IdContext& context) {
    MapNode::deserialize(data,context);

    // deserialize the cloud
    ObjectData * blobData = static_cast<ObjectData *>(data.getField("cloud"));
    _cloud_ref.deserialize(*blobData,context);

    ArrayData& nodesArray=data.getField("nodes")->getArray();    
    for (size_t i =0; i< nodesArray.size(); i++){
      ValueData& v = nodesArray[i];
      Identifiable* id = v.getPointer();
      MapNode* n = dynamic_cast<MapNode*>(id);
      _nodes.addElement(n);
    }
    
    ArrayData& relationsArray=data.getField("relations")->getArray();
    for (size_t i =0; i< relationsArray.size(); i++){
      ValueData& v = relationsArray[i];
      Identifiable* id = v.getPointer();
      BinaryNodeRelation* r = dynamic_cast<BinaryNodeRelation*>(id);
    }
  }

  void LocalMap::push() { 
    MapNode::push();
    for(MapNodeList::iterator nodes_it = _nodes.begin(); nodes_it != _nodes.end(); ++nodes_it) { 
      (*nodes_it)->push(); 
    }
  }

  void LocalMap::pop() { 
    MapNode::pop();
    for(MapNodeList::iterator nodes_it = _nodes.begin(); nodes_it != _nodes.end(); ++nodes_it) { 
      (*nodes_it)->pop(); 
    }
  }

  void LocalMap::draw(DrawAttributesType attributes, int name){
    if (! (attributes&ATTRIBUTE_SHOW))
      return;
    
    if (name>-1)
      glPushName(name);

    glPushMatrix();
    glMultMatrix(_transform);
    if (attributes&ATTRIBUTE_SELECTED) {
      if(!(attributes&ATTRIBUTE_ONLY)) {
	cloud()->draw(attributes);      
      }
      _nodes.draw(attributes);
      for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++) {
	(*it)->draw();
      }
    }

    glPushMatrix();
    glScalef(0.2, 0.2, 0.2);
    glPushAttrib(GL_COLOR);
    drawReferenceSystem();

    glPopAttrib();
    glPopMatrix();

    glPopMatrix();

    if (name>-1)
      glPopName();
  }

  BOSS_REGISTER_CLASS(LocalMap);  

}
