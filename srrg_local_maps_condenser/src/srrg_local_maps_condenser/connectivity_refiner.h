#pragma once
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>
#include <queue>
#include <Eigen/Core>
#include <srrg_core_map/map_node_list.h>
#include <srrg_core_map/binary_node_relation.h>
#include "srrg_core_map/local_map_with_traversability.h"
#include "srrg_structure_analyzer/structure_analyzer.h"

namespace srrg_local_maps_condenser{

class ConnectivityRefiner{

public:
    ConnectivityRefiner(float distance_threshold_ = 5, int connected_points_ = 10, float resolution_ = 0.05):
        _distance_threshold(distance_threshold_),
        _connected_points(connected_points_),
        _resolution(resolution_),
        _local_maps(new srrg_core_map::MapNodeList),
        _relations(new srrg_core_map::BinaryNodeRelationSet){}
    inline void setInput(srrg_core_map::MapNodeList* local_maps_) {_local_maps = local_maps_;}
    void execute();
    srrg_core_map::BinaryNodeRelationSet* edges() {return _relations;}
    bool addEdge(srrg_core_map::LocalMapWithTraversability* lmap1, srrg_core_map::LocalMapWithTraversability* lmap2);
    bool same(srrg_core_map::LocalMapWithTraversability* lmap1, srrg_core_map::LocalMapWithTraversability* lmap2);
    bool closeEnough(srrg_core_map::LocalMapWithTraversability* lmap1, srrg_core_map::LocalMapWithTraversability* lmap2);
    bool alreadyConnected(srrg_core_map::LocalMapWithTraversability* lmap1, srrg_core_map::LocalMapWithTraversability* lmap2);

protected:
    float _distance_threshold;
    int _connected_points;
    float _resolution;
    srrg_core_map::MapNodeList* _local_maps;
    srrg_core_map::BinaryNodeRelationSet* _relations;
};
}
