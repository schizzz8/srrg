#pragma once
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>
#include <queue>
#include <map>
#include <boost/bimap.hpp>
#include <Eigen/Core>
#include "srrg_core_map/map_node.h"
#include "srrg_core_map/map_node_list.h"
#include "srrg_core_map/local_map_with_traversability.h"
#include "srrg_octree/octree.h"
#include "srrg_structure_analyzer/structure_analyzer.h"

namespace srrg_local_maps_condenser{

typedef std::map<Eigen::Vector3f,srrg_core_map::MapNode*> Vector3fMapNodeMap;

class LocalMapsClusterizer {

public:
    LocalMapsClusterizer(int depth_ = 0, float resolution_ = 0.05, float voxel_size_ = 0.02):
        _depth(depth_),
        _resolution(resolution_),
        _voxel_size(voxel_size_),
        _merged_local_maps(new srrg_core_map::MapNodeList),
        _octree(NULL) {
        _structure_analyzer.setCellResolution(_resolution);
        _count = 0;
    }
    void setInput(srrg_core_map::MapNodeList *local_maps_);
    void execute();
    srrg_core_map::MapNodeList* clusters() {return _merged_local_maps;}
private:
    int _count;
    int _depth;
    float _resolution;
    float _voxel_size;
    //srrg_core_map::MapNodeList* _local_maps;
    Vector3fMapNodeMap _local_maps;
    std::vector<Eigen::Vector3f> _points;
    srrg_core_map::MapNodeList* _merged_local_maps;
    Eigen::Vector3f _min;
    Eigen::Vector3f _max;
    srrg_core::OctreeNode* _octree;
    srrg_structure_analyzer::StructureAnalyzer _structure_analyzer;
    void visit(srrg_core::OctreeNode* octree);
};
}
