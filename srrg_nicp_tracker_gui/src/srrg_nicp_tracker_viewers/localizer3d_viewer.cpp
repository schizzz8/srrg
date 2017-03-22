#include "localizer3d_viewer.h"
#include "qevent.h"
#include <cstring>
#include <srrg_gl_helpers/opengl_primitives.h>

namespace srrg_nicp_tracker_gui{
using namespace std;
using namespace Eigen;
using namespace srrg_core_viewers;
using namespace srrg_nicp_tracker;
using namespace srrg_gl_helpers;

LocalizerViewer::LocalizerViewer(Tracker* t) {
    _tracker = t;
    _modelTainted = true;
    _follow_camera = false;
}

void LocalizerViewer::draw(){
    if (!_tracker->currentModel() || !_tracker->referenceModel() )
        return;

    glPushMatrix();
    glMultMatrix(_transform);
    drawReferenceSystem();
    glPopMatrix();

    glPushMatrix();
    if (!_follow_camera) {
        glMultMatrix( _tracker->globalT() );
    } else {
        glTranslatef(0,0,1.5);
    }

    glPushMatrix();

    glScalef(0.2, 0.2, 0.2);
    drawReferenceSystem();
    glPopMatrix();

    glPushMatrix();
    glMultMatrix(_tracker->lastCamera()->offset());
    float pyrH = 0.1;
    float pyrW = 0.05;
    drawPyramidWireframe(/*pyrH = */ 0.1, /*pyrW = */ 0.05);
    glPopMatrix();

    /// draw the reference
    if (_tracker->referenceModel()){
        glColor3f(0.5, 0.5, 0.5);
        _tracker->referenceModel()->draw();
    }

    /// draw the current after applying the epsilon T from aligner
    glPushMatrix();
    glMultMatrix(_tracker->aligner().T().inverse());
    if (_tracker->currentModel()){
        glColor3f(0.2, 0.2, 1.0);
        _tracker->currentModel()->draw();
    }
    glPopMatrix();

    glPopMatrix();
}

}

