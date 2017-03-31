#pragma once
#include <qevent.h>
#include <vector>
#include "srrg_core_viewers/simple_viewer.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_types/types.hpp"
#include "srrg_gl_helpers/opengl_primitives.h"
#include "srrg_depth2laser/depth2laser.h"

namespace srrg_depth2laser_gui {

class Depth2LaserViewer: public srrg_core_viewers::SimpleViewer {
public:
    Depth2LaserViewer(srrg_depth2laser::Depth2Laser* depth2laser_);

    virtual void draw();
    std::vector<srrg_core::LaserMessage*> scans;
protected:
    srrg_depth2laser::Depth2Laser* _depth2laser;

};


}
