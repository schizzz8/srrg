#pragma once

#include "cloud.h"
#include "binary_node_relation.h"
#include "map_node_list.h"

namespace srrg_core_map {

  class LocalMap : public MapNode{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMap(const Eigen::Isometry3f& transform=Eigen::Isometry3f::Identity(), 
	     int id=-1,
	     srrg_boss::IdContext* context=0);

    virtual ~LocalMap();
 
    virtual void draw(srrg_gl_helpers::DrawAttributesType attributes=ATTRIBUTE_SHOW, int name = -1);
    inline Cloud* cloud() { return _cloud_ref.get(); }
    inline void setCloud(Cloud* c) { _cloud_ref.set(c); }

    inline MapNodeList& nodes() {return _nodes; }
    inline const MapNodeList& nodes() const {return _nodes; }
    inline BinaryNodeRelationSet& relations() {return _relations;}

    virtual void push();
    virtual void pop();

    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    inline CloudBLOBReference& cloudReference() { return _cloud_ref;}
  protected:
    // Nodes positions are expressed with respect to the local map position 
    MapNodeList _nodes;
    BinaryNodeRelationSet _relations;
    CloudBLOBReference _cloud_ref;
  };

}
