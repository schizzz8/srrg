#pragma once
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>
#include <queue>
#include <Eigen/Core>
//#include <srrg_core_map/local_map.h>

namespace srrg_core{

class AxisAlignedBox {

public:
    AxisAlignedBox(){}
    AxisAlignedBox(Eigen::Vector3f min, Eigen::Vector3f max);
    AxisAlignedBox(Eigen::Vector3f center, float halfWidth);
    void splitBox(AxisAlignedBox *boxes);
    bool inRange(Eigen::Vector3f point);
    std::vector<Eigen::Vector3f> vertices();
    Eigen::Vector3f min(){return _min;}
    Eigen::Vector3f max(){return _max;}
    Eigen::Vector3f center(){return _center;}

protected:
    Eigen::Vector3f _min;
    Eigen::Vector3f _max;
    Eigen::Vector3f _center;
    float _halfWidth;
};

class OctreeNode {

public:
    OctreeNode(int depth,std::vector<Eigen::Vector3f> points, Eigen::Vector3f  min, Eigen::Vector3f max);
    OctreeNode* getChild1(){return _children[0];}
    OctreeNode* getChild2(){return _children[1];}
    OctreeNode* getChild3(){return _children[2];}
    OctreeNode* getChild4(){return _children[3];}
    OctreeNode* getChild5(){return _children[4];}
    OctreeNode* getChild6(){return _children[5];}
    OctreeNode* getChild7(){return _children[6];}
    OctreeNode* getChild8(){return _children[7];}
    int depth(){return _depth;}
    AxisAlignedBox box(){return _box;}
    std::vector<Eigen::Vector3f> points(){return _points;}

private:
    int _depth;
    AxisAlignedBox _box;
    std::vector<Eigen::Vector3f> _points;
    OctreeNode* _children[8] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
};

}
