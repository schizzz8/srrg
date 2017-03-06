#include "map_node.h"
#include <GL/gl.h>
#include <iostream>

namespace srrg_core_map {
  
  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_core;
  using namespace srrg_gl_helpers;
  
  MapNode::MapNode(const Eigen::Isometry3f& t, 
		   int id,
		   IdContext* context):
    Identifiable(id,context){
    _transform = t;
    _timestamp = 0;
  }

  void MapNode::draw(DrawAttributesType attributes, int name) { }  
  
  MapNode::~MapNode() {}

  void MapNode::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    data.setDouble("timestamp", _timestamp);
    t2v(_transform).toBOSS(data,"transform");

    ArrayData* parentsArray = new ArrayData;
    for (MapNodeSet::iterator it = _parents.begin(); it!=_parents.end(); it++){
      MapNode* node=*it;
      parentsArray->add(new PointerData(node));
    }
    data.setField("parents", parentsArray);
  }

  void MapNode::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    _timestamp = data.getDouble("timestamp");
    Vector6f v;
    v.fromBOSS(data,"transform");
    _transform = v2t(v);
    ArrayData& parentsArray=data.getField("parents")->getArray();
    _pending_parents.resize(parentsArray.size());
    for (size_t i =0; i< parentsArray.size(); i++){
      ValueData& v = parentsArray[i];
      v.getReference().bind(_pending_parents[i]);
    }
  }
  
  void MapNode::deserializeComplete(){
    _parents.clear();
    for (size_t i =0; i< _pending_parents.size(); i++){
      MapNode* n = dynamic_cast<MapNode*>(_pending_parents[i]);
      _parents.insert(n);
    }
  }
}
