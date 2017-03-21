#pragma once

#include <srrg_core_viewers/simple_viewer.h>
#include <srrg_core_map/map_node_list.h>
#include <srrg_core_map/binary_node_relation.h>

namespace srrg_core_map_gui {

  class TrajectoryViewer: public srrg_core_viewers::SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void draw();
    virtual void drawWithNames();

    virtual void keyPressEvent(QKeyEvent *e);

    srrg_core_map::MapNodeList nodes;
    srrg_core_map::BinaryNodeRelationSet relations;

  protected:
    std::map<int, srrg_core_map::MapNode*> _names_map;
    virtual void postSelection(const QPoint& point);
    std::set<srrg_core_map::MapNode*> _selected_objects;

  };
 
}
