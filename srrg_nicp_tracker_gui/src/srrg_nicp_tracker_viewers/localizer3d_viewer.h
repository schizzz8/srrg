#pragma once

#include "qglviewer.h"
#include "qapplication.h"
#include <srrg_nicp_tracker/tracker.h>
#include <srrg_core_viewers/simple_viewer.h>

namespace srrg_nicp_tracker_gui {

class LocalizerViewer: public srrg_core_viewers::SimpleViewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    LocalizerViewer(srrg_nicp_tracker::Tracker* _tracker);
    virtual void draw();
    inline bool followCamera() const {return _follow_camera;}
    inline void setFollowCamera(bool follow_camera) { _follow_camera = follow_camera;}
    inline void setTransform(const Eigen::Isometry3f& t) {_transform = t;}
protected:
    srrg_nicp_tracker::Tracker* _tracker;
    bool _modelTainted;
    bool _follow_camera;
    Eigen::Isometry3f _transform;
};


}
