#pragma once

#include <srrg_types/defs.h>

#include "base_projector.h"
#include "base_correspondence_finder.h"
#include "solver.h"

namespace srrg_nicp {
  class ProjectiveCorrespondenceFinder: public BaseCorrespondenceFinder{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  public:
    //! ctor, p is the projector object used to compute the correspondences
    ProjectiveCorrespondenceFinder(Solver* s, BaseProjector* p);

    //! call this once before whenever you change the current cloud
    void init();

    //! call this once before whenever you change the current cloud
    void init(srrg_core::FloatImage& reference_buffer, srrg_core::IntImage& reference_indices,
	      srrg_core::FloatImage& current_buffer, srrg_core::IntImage& current_indices, float current_scale=1);

    //! overridden method from base class
    void compute();
    //! returns the depth buffer of the current projection
    inline const srrg_core::FloatImage& zBuffer() const {return _zbuffer;}

    //! returns the depth buffer of the reference projection
    inline const srrg_core::FloatImage& referenceZBuffer() const {return _reference_zbuffer;}
    
    //! the potential indices of the correspondences of the current cloud
    //! all correspondences returned will be between one of the points having these indices
    //! in the current cloud and a point in the reference cloud
    std::vector<int> potentialReferenceIndices() {return _potential_reference_indices; }


    //! testing
    inline bool useFullModelProjection() const { return _use_full_model_projection;}
    inline void setUseFullModelProjection(bool fmp) { _use_full_model_projection = fmp;}
    
    inline const srrg_core::IntImage& referenceIndices() {return _reference_indices;}
    inline const srrg_core::IntImage& indices() {return _indices;}

    inline int columnSearchWindow() const { return _column_search_window; }
    inline int rowSearchWindow() const { return _row_search_window; }
    void setColumnSearchWindow(int window) { _column_search_window=window;}
    void setRowSearchWindow(int window) { _row_search_window=window;}
  protected:
    BaseProjector* _projector;
    
    srrg_core::FloatImage _zbuffer, _current_zbuffer, _reference_zbuffer;
    srrg_core::IndexImage _indices, _current_indices,  _reference_indices;
    
    bool _use_full_model_projection;
    int _row_search_window;
    int _column_search_window;

    std::vector<int> _potential_reference_indices;
    float _current_scale;
  };
}
