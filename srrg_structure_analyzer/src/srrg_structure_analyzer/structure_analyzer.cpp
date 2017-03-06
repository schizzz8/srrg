#include "structure_analyzer.h"
#include <iostream>

namespace srrg_structure_analyzer {

using namespace std;
using namespace srrg_core_map;

StructureAnalyzer::StructureAnalyzer() {
    _cell_resolution=0.005;
    _normal_cos_threshold=cos(20*M_PI/180.0);
    _upper = Eigen::Vector3f::Zero();
    _lower = Eigen::Vector3f::Zero();
}

void StructureAnalyzer::compute(Cloud* cloud_, const Eigen::Isometry3f& iso) {
    _indices.release();
    _elevations.release();
    _transformed_cloud.clear();
    if (! cloud_)
        return;
    _transformed_cloud=*cloud_;
    _transformed_cloud.transformInPlace(iso);
    float ires=1./_cell_resolution;
    _transformed_cloud.computeBoundingBox(_lower, _upper);

    Eigen::Vector3f range = _upper - _lower;
    _size = (range*ires).cast<int>();
    int cols=_size.x();
    int rows=_size.y();

    _bottom = _lower;
    _bottom.z() = 0;
    _indices.create(rows,cols);
    _elevations.create(rows,cols);
    _classified.create(rows,cols);
    _indices=-1;
    _elevations=4;
    _classified=127;
    float _robot_climb_step=0.1;
    float _robot_height=0.5;

    // compute the elevatio of the surface
    for (size_t i=0; i<_transformed_cloud.size(); i++){
        const RichPoint& p = _transformed_cloud[i];
        float z = p.point().z();
        Eigen::Vector3f projected_point = (p.point() - _bottom)*ires;
        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();
        if (r>=_indices.rows || r<0)
            continue;
        if (c>=_indices.cols || r<0)
            continue;
        float &h=_elevations.at<float>(r,c);
        int& idx=_indices.at<int>(r,c);
        if (z<h) {
            h=z;
            idx=i;
        }
    }

    // mark the cells that are obstacles
    for (size_t i=0; i<_transformed_cloud.size(); i++){
        const RichPoint& p = _transformed_cloud[i];
        Eigen::Vector3f projected_point = (p.point() - _bottom)*ires;
        float z = p.point().z();
        if (p.normal().squaredNorm()< 0.1)
            continue;
        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();
        if (r>=_indices.rows || r<0)
            continue;
        if (c>=_indices.cols || r<0)
            continue;
        float &h=_obstacles.at<float>(r,c);
        float &g=_elevations.at<float>(r,c);
        int& idx=_indices.at<int>(r,c);
        float min_obstacle_height=g+_robot_climb_step;
        float max_obstacle_height=g+_robot_height;
        if (z< min_obstacle_height) {
            continue;
        }
        if (z>max_obstacle_height)
            continue;
        idx=-2;
        g=z;
    }
    // fill in the invalid points
    for (int r=0; r<_indices.rows; r++)
        for (int c=0; c<_indices.cols; c++) {
            int idx = _indices.at<int>(r,c);
            if (idx==-1)
                continue;
            if (idx<-1){
                _classified.at<unsigned char>(r,c)=255;
                continue;
            }
            _classified.at<unsigned char>(r,c)=0;
        }


    // clean the spurious points
    for (int r=1; r<_classified.rows-1; r++)
        for (int c=1; c<_classified.cols-1; c++) {
            unsigned char & cell=_classified.at<unsigned char>(r,c);
            if (cell!=255)
                continue;
            // seek for the 8 neighbors and isolate spurious points
            bool one_big=false;
            for (int rr=-1; rr<=1; rr++)
                for (int cc=-1; cc<=1; cc++) {
                    if (rr==0 && cc==0)
                        continue;
                    one_big |= _classified.at<unsigned char>(r+rr,c+cc)==255;
                }
            if (! one_big) {
                cell=0;
            }
        }

    // color the points (visualization only)
    for (size_t i=0; i<_transformed_cloud.size(); i++){
        const RichPoint& p = _transformed_cloud[i];
        Eigen::Vector3f projected_point = (p.point() - _bottom)*ires;
        float z = p.point().z();
        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();
        if (r>=_indices.rows || r<0)
            continue;
        if (c>=_indices.cols || r<0)
            continue;
        float &g=_elevations.at<float>(r,c);
        float max_obstacle_height=g+_robot_height;
        int& idx=_indices.at<int>(r,c);
        if (idx<-1 || z>max_obstacle_height) {
            cloud_->at(i)._rgb=Eigen::Vector3f(1,0,0);
            //cloud_->at(i).setTraversable(false);
        }
        else {
            cloud_->at(i)._rgb=Eigen::Vector3f(0,1,0);
            //cloud_->at(i).setTraversable(true);
        }
    }
}
}
