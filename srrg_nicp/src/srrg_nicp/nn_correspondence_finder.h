#pragma once

#include <flann/flann.hpp>

#include <srrg_types/defs.h>
#include <srrg_kdtree/kd_tree.hpp>

#include "base_correspondence_finder.h"
#include "solver.h"

namespace srrg_nicp {

  #define NNCF_KDTREE_DIM 6
  
  class NNCorrespondenceFinder: public BaseCorrespondenceFinder{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  public:
    //! ctor, p is the projector object used to compute the correspondences
    NNCorrespondenceFinder(Solver* s);

    //! dtor, destroys the index if created
    virtual ~NNCorrespondenceFinder();

    //! call this once before whenever you change the current cloud
    void init();

    //! overridden method from base class
    void compute();

    float normalScaling() const {return _normal_scaling;}
    void setNormalScaling(float ns)  {_normal_scaling =  ns;}

    float leafRange() const {return _leaf_range;}
    void setLeafRange(float leaf_range)  { _leaf_range = leaf_range; }
    
    float maxDistance() const { return _max_distance; }
    void setMaxDistance(float max_distance)  { _max_distance =  max_distance; }

  protected:
    float _normal_scaling;
    float _leaf_range;
    float _max_distance;;
    srrg_core::KDTree<float, NNCF_KDTREE_DIM>* _kdtree;
      
    // working_area
    
    srrg_core::KDTree<float, NNCF_KDTREE_DIM>::VectorTDVector _reference_points;
    srrg_core::KDTree<float, NNCF_KDTREE_DIM>::VectorTDVector _current_points;
  };
}
