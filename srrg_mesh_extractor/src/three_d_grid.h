#pragma once

#include <iostream>
#include <math.h>
#include <limits>
#include <queue>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <utility>
#include <map>
#include <functional>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Dense>


#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vtkSmartPointer.h>
#include <vtkVersion.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkXMLImageDataWriter.h>

#include "colors.h"


typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;


class Cell {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Cell(const Eigen::Vector3i& idx=Eigen::Vector3i::Zero()):
        _idx(idx){
        _parent = 0;
        _distance = std::numeric_limits<float>::max();
    }

    inline bool operator < (const Cell& c) const {
        for (int i=0; i<3; i++){
            if (_idx[i]<c._idx[i])
                return true;
            if (_idx[i]>c._idx[i])
                return false;
        }
        return false;
    }

    inline bool operator == (const Cell& c) const {
        for (int i=0; i<3; i++)
            if(_idx[i] != c._idx[i])
                return false;
        return true;
    }

    inline void setCenter(Eigen::Vector3f origin, float resolution) {
        _center = origin + _idx.cast<float>()*resolution + Eigen::Vector3f(resolution/2,resolution/2,resolution/2);
    }

    Eigen::Vector3i _idx;
    Eigen::Vector3f _center;
    std::vector<size_t> _points;
    Cell* _parent;
    size_t _closest_point;
    float _distance;
    int _sign;
};

struct QEntry{
    QEntry(Cell* c=0, float d=std::numeric_limits<float>::max()) {
        _cell = c;
        _distance = d;
    }

    inline bool operator < (const QEntry& e) const {
        return e._distance < _distance ;
    }

    float _distance;
    Cell* _cell;
};

struct CellQueue : public std::priority_queue<QEntry> {
    typedef typename std::priority_queue<QEntry>::size_type size_type;
    CellQueue(size_type capacity = 0) { reserve(capacity); }
    inline void reserve(size_type capacity) { this->c.reserve(capacity); }
    inline size_type capacity() const { return this->c.capacity(); }
    inline Cell* top() { return std::priority_queue<QEntry>::top()._cell;}
    inline void push(Cell* c) { return std::priority_queue<QEntry>::push(QEntry(c, c->_distance));}
};

class BaseGrid {
public:
    BaseGrid(const PointCloud::Ptr &cloud, int prec = 5);
    virtual ~BaseGrid(){}
    inline float resolution(){ return _resolution;}
    inline const Eigen::Vector3i size(){ return _size;}
    inline const Eigen::Vector3f origin(){ return _origin;}
    inline int numCells(){ return _num_cells;}

    void toIdx(float x, float y, float z, Eigen::Vector3i& idx);
    void toWorld(int i, int j, int k, Eigen::Vector3f& point);
    int toInt(Eigen::Vector3i idx);
    void toIJK(int in, Eigen::Vector3i& idx);

    inline float euclideanDistance(Eigen::Vector3f a, PointType b){return sqrtf(pow(a.x()-b.x,2)+pow(a.y()-b.y,2)+pow(a.z()-b.z,2));}
    inline int computeSign(Eigen::Vector3f a, PointType b) {
        return (a.norm()-sqrtf(pow(b.x,2)+pow(b.y,2)+pow(b.z,2)) < 0) ? -1 : 1;
    }
    inline int sgn(float in){return (in < 0 ) ? -1 : in > 0;}

    virtual bool hasCell(const Eigen::Vector3i& idx){ return 0;}
    virtual int findNeighbors(Cell** neighbors, Cell* c){ return 0;}
    virtual void computeDistanceMap(float maxDistance=std::numeric_limits<float>::max()){}

    virtual void writeDataToFile(){}

protected:
    pcl::PointCloud<PointType>::Ptr _cloud;
    float _resolution;
    float _inverse_resolution;
    Eigen::Vector3f _origin;
    Eigen::Vector3i _size;
    int _num_cells;

private:
    double computeCloudResolution ();
    int getdigit(double number, int digit);
    void manageDirectories(std::string directory);
    int isDirectoryEmpty(const char *dirname);
};

template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
typedef std::unordered_map<Eigen::Vector3i,Cell*,matrix_hash<Eigen::Vector3i> > Vector3iCellPtrMap;

class AdaptiveGrid : public BaseGrid {
public:
    AdaptiveGrid (const PointCloud::Ptr &cloud, int prec = 5);
    void computeDistanceMap(float maxDistance=std::numeric_limits<float>::max());

    void writeDataToFile();

protected:
    Vector3iCellPtrMap _cells;

    bool hasCell(const Eigen::Vector3i& idx);
    int findNeighbors(Cell **neighbors, Cell *c);
};
