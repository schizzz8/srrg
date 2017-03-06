#include "octree.h"

namespace srrg_core {

using namespace std;

AxisAlignedBox::AxisAlignedBox(Eigen::Vector3f min, Eigen::Vector3f max):_min(min),_max(max){
    _halfWidth = (_max(0) - _min(0))/2;
    _center << _min(0) + _halfWidth,_min(1) + _halfWidth,_min(2) + _halfWidth;
}

AxisAlignedBox::AxisAlignedBox(Eigen::Vector3f center, float halfWidth):_center(center),_halfWidth(halfWidth){
    _min << _center(0) - _halfWidth,_center(1) - _halfWidth,_center(2) - _halfWidth;
    _max << _center(0) + _halfWidth,_center(1) + _halfWidth,_center(2) + _halfWidth;
}

void AxisAlignedBox::splitBox(AxisAlignedBox *boxes){
    int idx = 0;
    float halfWidth = _halfWidth/2;
    for(int k = -1; k < 2; k += 2)
        for(int j = -1; j < 2; j += 2)
            for(int i = -1; i < 2; i += 2)
            {
                boxes[idx] = AxisAlignedBox(Eigen::Vector3f (_center(0)+i*halfWidth,_center(1)+j*halfWidth,_center(2)+k*halfWidth),halfWidth);
                idx++;
            }
}

bool AxisAlignedBox::inRange(Eigen::Vector3f point){
    if(point(0) >= _min(0) && point(0) <= _max(0)
            && point(1) >= _min(1) && point(1) <= _max(1)
            && point(2) >= _min(2) && point(2) <= _max(2))
        return true;
    else
        return false;
}

std::vector<Eigen::Vector3f> AxisAlignedBox::vertices(){
    std::vector<Eigen::Vector3f> vertices;
    for(int k = -1; k < 2; k += 2)
        for(int j = -1; j < 2; j += 2)
            for(int i = -1; i < 2; i += 2)
                vertices.push_back(Eigen::Vector3f(_center(0)+i*_halfWidth,_center(1)+j*_halfWidth,_center(2)+k*_halfWidth));
    return vertices;
}

OctreeNode::OctreeNode(int depth, std::vector<Eigen::Vector3f> points, Eigen::Vector3f min, Eigen::Vector3f max):_depth(depth){
    if(_depth < 0)
        return;
    else{
        _box = AxisAlignedBox(min,max);
        _points = points;
        AxisAlignedBox boxes[8];
        _box.splitBox(boxes);
        vector<Eigen::Vector3f> splittedPoints[8];
        //        for(int i=0; i < 8; i++)
        //            splittedLocalMaps[i] = new MapNodeList;
        //        for(MapNodeList::iterator it = _local_maps->begin(); it != _local_maps->end(); it++)
        //            for(int boxId = 0; boxId < 8; boxId++) {
        //                Eigen::Vector3f origin = it->get()->transform().translation();
        //                if(boxes[boxId].inRange(origin))
        //                    splittedLocalMaps[boxId]->addElement(it->get());
        //            }
        for(int ii=0; ii < _points.size(); ii++)
            for(int boxId = 0; boxId < 8; boxId++)
                if(boxes[boxId].inRange(_points.at(ii)))
                    splittedPoints[boxId].push_back(_points.at(ii));

        //        for(int i = 0; i < 8; i++)
        //            if(!splittedLocalMaps[i]->empty())
        //                _children[i] = new OctreeNode(depth-1,splittedLocalMaps[i],boxes[i].min(),boxes[i].max());
        for(int i = 0; i < 8; i++)
            if(!splittedPoints[i].empty()){
                _children[i] = new OctreeNode(depth-1,splittedPoints[i],boxes[i].min(),boxes[i].max());
            }

    }
}

}
