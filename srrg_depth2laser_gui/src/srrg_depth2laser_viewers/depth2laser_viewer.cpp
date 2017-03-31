#include "depth2laser_viewer.h"

namespace srrg_depth2laser_gui {
using namespace std;
using namespace Eigen;
using namespace srrg_core;
using namespace srrg_gl_helpers;

Depth2LaserViewer::Depth2LaserViewer(srrg_depth2laser::Depth2Laser *depth2laser_){
    _depth2laser = depth2laser_;
}

void Depth2LaserViewer::draw(){
    for (size_t i = 0; i<scans.size(); i++){
        vector<float> ranges = scans[i]->ranges();
        Eigen::Isometry3f odometry = scans[i]->odometry();
        Eigen::Isometry3f offset = scans[i]->offset();

        glPushMatrix();
        glMultMatrix(odometry);

        drawArrow2D(0.1,0.05,0.05);

        glPushAttrib(GL_COLOR|GL_POINT_SIZE);
        glPointSize(1);
        glBegin(GL_POINTS);
        for(int j=0; j < ranges.size(); j++){
            if(ranges[j] >= _depth2laser->maxRange())
                continue;

            float theta = _depth2laser->minAngle() + j*_depth2laser->angleIncrement();
            float rho = ranges[j];
            Eigen::Vector3f laser_point(rho*cos(theta),rho*sin(theta),0);
            Eigen::Vector3f robot_point = offset*laser_point;
            glColor3f(1,0,0);
            glNormal3f(0,0,1);
            glVertex3f(robot_point.x(), robot_point.y(), robot_point.z());
        }
        glEnd();
        glPopAttrib();
        glPopMatrix();

    }
}

}
