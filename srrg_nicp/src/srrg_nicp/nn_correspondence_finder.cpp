#include "nn_correspondence_finder.h"

namespace srrg_nicp {

  using namespace srrg_core;
  using namespace srrg_core_map;
  
  NNCorrespondenceFinder::NNCorrespondenceFinder(Solver* s):
    BaseCorrespondenceFinder(s) {
    _kdtree = 0;
    _normal_scaling = 1;
    _leaf_range = 0.2;
    _max_distance = 1e9;
  }

  NNCorrespondenceFinder::~NNCorrespondenceFinder() {
    if (_kdtree) {      
      delete _kdtree;
    }
    _kdtree = 0;
  }

  void model2linear(KDTree<float, NNCF_KDTREE_DIM>::VectorTDVector& dest, const Cloud& src, float nscale){
    int k = 0;
    dest.resize(src.size()); // relies on the packetization of the data
    for (size_t i = 0; i < src.size(); ++i) {
      dest[i].head<3>() = src[i].point();
      dest[i].tail<3>() = src[i].normal() * nscale;      
    }
  }

  void model2linear(KDTree<float, NNCF_KDTREE_DIM>::VectorTDVector& dest, const Cloud& src, float nscale, const Eigen::Isometry3f& T){
    int k = 0;
    dest.resize(src.size()); // relies on the packetization of the data
    Eigen::Matrix3f sR = T.linear() * nscale;
    for (size_t i = 0; i < src.size(); ++i) {
      dest[i].head<3>() = T * src[i].point();
      dest[i].tail<3>() = sR * src[i].normal();
    }
  }

  void NNCorrespondenceFinder::init(){
    if (! _solver->referenceModel()){
      throw std::runtime_error("NNCorrespondenceFinder::init(), no reference model in solver");
    }
    const Cloud& ref = *_solver->referenceModel();

    model2linear(_reference_points, ref, _normal_scaling);
    if (_kdtree)
      delete(_kdtree);
    _kdtree = 0;


    _kdtree = new KDTree<float, NNCF_KDTREE_DIM>(_reference_points, _leaf_range);
    
    if (! _solver->currentModel()){
      throw std::runtime_error("NNCorrespondenceFinder::init(), no reference model in solver");
    }

    // allocate the workspace for the current model and for the search
    const Cloud& curr = *_solver->currentModel();
    model2linear(_current_points, curr, _normal_scaling);
    _correspondences.reserve(curr.size());
  }

  void NNCorrespondenceFinder::compute(){
    const Cloud& curr = *_solver->currentModel();
    const Cloud& ref = *_solver->referenceModel();
    Eigen::Isometry3f T = _solver->T();
    float ndist = cos(_normal_angle);
    float pdist = _points_distance * _points_distance;

    model2linear(_current_points, curr, _normal_scaling, T);

    // scan through the returned indices and compute the correspondences
    _correspondences.clear();
    int k = 0;
    for(size_t i = 0; i < _reference_points.size(); ++i) {
      int index;
      float approx_distance;
      KDTree<float, NNCF_KDTREE_DIM>::VectorTD answer;
      KDTree<float, NNCF_KDTREE_DIM>::VectorTD query = _current_points[i];
      approx_distance = _kdtree->findNeighbor(answer, index, query, _max_distance);
      
      if (index < 0) {
	continue;
      }

      Eigen::Vector3f cn = T.linear() * curr[i].normal();
      Eigen::Vector3f cp = T * curr[i].point();
      
      const Eigen::Vector3f& rn = ref[index].normal();
      const Eigen::Vector3f& rp = ref[index].point();
      if ((cp - rp).squaredNorm() > pdist) {
	continue;
      }

      if (srrg_core::isNan(rn) || srrg_core::isNan(cn)) { 
	continue;
      }
      if (cn.dot(rn) < ndist) {
	continue;
      }
      _correspondences.push_back(std::make_pair(index, i));
    }
  }

}
