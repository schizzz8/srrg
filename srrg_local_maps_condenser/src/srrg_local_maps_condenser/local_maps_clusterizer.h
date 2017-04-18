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

typedef std::map<Eigen::Vector2f,srrg_core_map::MapNode*> Vector2fMapNodeMap;

class AxisAlignedSquare {

public:
    AxisAlignedSquare(){}

    AxisAlignedSquare(Eigen::Vector2f min, Eigen::Vector2f max):_min(min),_max(max){
        _halfWidth = (_max.x() - _min.x())/2;
        _center << _min.x() + _halfWidth,
                _min.y() + _halfWidth;
    }

    AxisAlignedSquare(Eigen::Vector2f center, float halfWidth):_center(center),_halfWidth(halfWidth){
        _min << _center.x() - _halfWidth,
                _center.y() - _halfWidth;
        _max << _center.x() + _halfWidth,
                _center.y() + _halfWidth;
    }

    inline void splitSquare(AxisAlignedSquare *squares){
        int idx = 0;
        float halfWidth = _halfWidth/2;
        for(int j = -1; j < 2; j += 2)
            for(int i = -1; i < 2; i += 2){
                squares[idx] = AxisAlignedSquare(Eigen::Vector2f (_center.x()+i*halfWidth,
                                                                  _center.y()+j*halfWidth),
                                                 halfWidth);
                idx++;
            }
    }

    inline bool inRange(const Eigen::Vector2f& point){
        return (point.x() >= _min.x() && point.x() <= _max.x() &&
                point.y() >= _min.y() && point.y() <= _max.y()) ? true : false;
    }

    inline const Eigen::Vector2f& min() const {return _min;}
    inline const Eigen::Vector2f& max() const {return _max;}
    inline const Eigen::Vector2f& center() const {return _center;}

protected:
    Eigen::Vector2f _min;
    Eigen::Vector2f _max;
    Eigen::Vector2f _center;
    float _halfWidth;
};

class QuadtreeNode {

public:
    QuadtreeNode(int depth, std::vector<Eigen::Vector2f> points, Eigen::Vector2f min, Eigen::Vector2f max):_depth(depth){
        if(_depth < 0)
            return;
        else{
            _square = AxisAlignedSquare(min,max);
            _points = points;
            AxisAlignedSquare squares[4];
            _square.splitSquare(squares);
            std::vector<Eigen::Vector2f> splitted_points[4];

            for(int ii=0; ii < _points.size(); ii++)
                for(int id = 0; id < 4; id++)
                    if(squares[id].inRange(_points.at(ii)))
                        splitted_points[id].push_back(_points.at(ii));

            for(int i = 0; i < 4; i++)
                if(!splitted_points[i].empty())
                    _children[i] = new QuadtreeNode(depth-1,splitted_points[i],squares[i].min(),squares[i].max());

        }
    }

    inline QuadtreeNode* getChild1(){return _children[0];}
    inline QuadtreeNode* getChild2(){return _children[1];}
    inline QuadtreeNode* getChild3(){return _children[2];}
    inline QuadtreeNode* getChild4(){return _children[3];}
    inline const int depth() const {return _depth;}
    inline const AxisAlignedSquare& square() const {return _square;}
    inline const std::vector<Eigen::Vector2f>& points() const {return _points;}

private:
    int _depth;
    AxisAlignedSquare _square;
    std::vector<Eigen::Vector2f> _points;
    QuadtreeNode* _children[4] = {NULL, NULL, NULL, NULL};
};


class LocalMapsClusterizer {

public:
    LocalMapsClusterizer(int depth_ = 0, float resolution_ = 0.05, float range_ = 5, float voxel_size_ = 0.02):
        _depth(depth_),
        _resolution(resolution_),
        _range(range_),
        _voxel_size(voxel_size_),
        _merged_local_maps(new srrg_core_map::MapNodeList),
        _quadtree(NULL) {
        _structure_analyzer.setCellResolution(_resolution);
        _count = 0;
    }

    void computeBoundingBox(srrg_core_map::MapNodeList *local_maps_);

    inline void buildQuadtree(){
        std::cerr << "Building quadtree..." << std::endl;
        _quadtree = new QuadtreeNode(_depth,_points,_min,_max);
    }

    inline void mergeLocalMaps(){
        visit(_quadtree);
    }

    inline void visualizeQuadtree(){
        _image = cv::Mat (_size.x(),_size.y(),CV_8UC3,cv::Scalar(255,255,255));
        visualize(_quadtree);
        std::cerr << "Showing image..." << std::endl;
        cv::namedWindow("Image",cv::WINDOW_NORMAL);
        cv::imshow("Image",_image);
        cv::waitKey(0);
    }

    srrg_core_map::MapNodeList* clusters() {return _merged_local_maps;}

private:
    int _count;
    int _depth;
    float _resolution;
    float _range;
    float _voxel_size;
    Vector2fMapNodeMap _local_maps;
    std::vector<Eigen::Vector2f> _points;
    srrg_core_map::MapNodeList* _merged_local_maps;
    Eigen::Vector2f _min;
    Eigen::Vector2f _max;
    Eigen::Vector2i _size;
    QuadtreeNode* _quadtree;
    srrg_structure_analyzer::StructureAnalyzer _structure_analyzer;
    cv::Mat _image;
    void visit(QuadtreeNode* quadtree);
    void visualize(QuadtreeNode *quadtree);
};
}
