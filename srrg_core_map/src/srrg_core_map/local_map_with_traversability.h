#pragma once
#include <srrg_boss/eigen_boss_plugin.h>
#include <srrg_core_map/local_map.h>
#include <srrg_core_map/binary_node_relation.h>
#include <srrg_core_map/cloud.h>
#include "traversability_map.h"

namespace srrg_core_map {

typedef srrg_boss::BLOBReference<srrg_core_map::Cloud> CloudBLOBReference;

class LocalMapWithTraversability : public srrg_core_map::MapNode{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapWithTraversability(const Eigen::Isometry3f& transform=Eigen::Isometry3f::Identity(),
                               int id=-1,
                               srrg_boss::IdContext* context=0);
    virtual ~LocalMapWithTraversability();

    inline double resolution() const { return _resolution; }
    inline void setResolution(double r) { _resolution = r; }
    inline Eigen::Vector3f& lower() { return _lower; }
    inline void setLower(const Eigen::Vector3f lower_) { _lower = lower_;}
    inline Eigen::Vector3f& upper() { return _upper; }
    inline void setUpper(const Eigen::Vector3f upper_) { _upper = upper_;}
    inline TraversabilityMap* traversabilityMap() { return _traversability_map_ref.get(); }
    inline void setTraversabilityMap(TraversabilityMap* t) { _traversability_map_ref.set(t); }
    inline TraversabilityMapBLOBReference& traversabilityMapReference() { return _traversability_map_ref;}
    inline CloudBLOBReference& cloudReference() { return _cloud_ref;}
    inline srrg_core_map::Cloud* cloud() { return _cloud_ref.get(); }
    inline void setCloud(srrg_core_map::Cloud* c) { _cloud_ref.set(c); }
    inline srrg_core_map::MapNodeList& nodes() {return _nodes; }
    inline const srrg_core_map::MapNodeList& nodes() const {return _nodes; }
    inline srrg_core_map::BinaryNodeRelationSet& relations() {return _relations;}

    virtual void draw(srrg_gl_helpers::DrawAttributesType attributes = ATTRIBUTE_SHOW, int name = -1);
    virtual void push();
    virtual void pop();
    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);

    bool intersects(LocalMapWithTraversability &n);
    void findMinMax(LocalMapWithTraversability& n, Eigen::Vector3f& common_min,Eigen::Vector3f& common_max);
    inline bool isTraversable(Eigen::Vector3f p){
        Eigen::Vector3f projected = (p - _lower)/_resolution;
        int r=projected.y();
        int c=projected.x();
        if (r>=_traversability_map_ref.get()->image().rows || r<0)
            return false;
        if (c>=_traversability_map_ref.get()->image().cols || r<0)
            return false;
        if (_traversability_map_ref.get()->image().at<unsigned char>(r,c)!=0)
            return false;
        return true;
    }

protected:
    // Nodes positions are expressed with respect to the local map position
    srrg_core_map::MapNodeList _nodes;
    srrg_core_map::BinaryNodeRelationSet _relations;
    CloudBLOBReference _cloud_ref;
    double _resolution;
    Eigen::Vector3f _lower;
    Eigen::Vector3f _upper;
    TraversabilityMapBLOBReference _traversability_map_ref;
};
}
