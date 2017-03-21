#include <iostream>
#include <stdexcept>

#include "projective_correspondence_finder.h"

namespace srrg_nicp {

  using namespace std;
  using namespace Eigen;
  using namespace srrg_core;
  using namespace srrg_core_map;
  
  ProjectiveCorrespondenceFinder::ProjectiveCorrespondenceFinder(Solver* s, BaseProjector* p):
    BaseCorrespondenceFinder(s),
    _zbuffer(0,0),
    _reference_indices(0,0){
    _projector = p;
    _column_search_window=0;
    _row_search_window=0;
    _use_full_model_projection = true;
  }

  void ProjectiveCorrespondenceFinder::init() {
    if (!_solver) {
      throw std::runtime_error ("solver not set");
    }
    if (!_projector) {
      throw std::runtime_error ("projector not set");
    }
    _projector->project(_reference_zbuffer, _reference_indices, Eigen::Isometry3f::Identity(), *_solver->referenceModel());
    _projector->project(_current_zbuffer, _current_indices, Eigen::Isometry3f::Identity(), *_solver->currentModel());
    init(_zbuffer, _reference_indices, _current_zbuffer, _current_indices);
  }

  //! call this once before whenever you change the current cloud
  void ProjectiveCorrespondenceFinder::init(FloatImage& reference_buffer, IntImage& reference_indices,
					    FloatImage& current_buffer, IntImage& current_indices,
					    float current_scale){
    if (!_solver) {
      throw std::runtime_error ("solver not set");
    }
    if (!_projector) {
      throw std::runtime_error ("projector not set");
    }
 
    if (_projector->imageRows()!=reference_indices.rows ||
	_projector->imageCols()!=reference_indices.cols) {
      throw std::runtime_error("Projective Correspondence Finder: init(FloatImage& , IntImage&  ), projector size does not match");
    }
    int nnz = 0;
    _reference_zbuffer=reference_buffer; 
    _reference_indices=reference_indices;
    _current_zbuffer = current_buffer;
    _current_indices = current_indices;
    _potential_reference_indices.reserve(_reference_indices.rows*_reference_indices.cols);
    _potential_reference_indices.resize(_reference_indices.rows*_reference_indices.cols);
    _correspondences.reserve(_reference_indices.rows*_reference_indices.cols);
    for (int r=0;r<_reference_indices.rows; r++)
      for (int c=0;c<_reference_indices.cols; c++) {
	int idx = _reference_indices.at<int>(r,c);
	if (idx>-1) {
	  _potential_reference_indices[nnz] = idx;
	  nnz++;
	}
      }
    _potential_reference_indices.resize(nnz);
    _current_scale = current_scale;
  }

  void ProjectiveCorrespondenceFinder::compute(){
    if (isNan(_solver->T().matrix())) {
      _correspondences.clear();
      return;
    }
    
    Vector2fVector projected;
    _correspondences.resize(_solver->referenceModel()->size());
    _indices.create(_projector->imageRows(), _projector->imageCols());
    _indices = -1;
    Eigen::Isometry3f T = _solver->T();
    Eigen::Matrix3f R = T.linear();
    
    if (_use_full_model_projection)
      _projector->project(_zbuffer, _indices, T, *_solver->currentModel());
    else {
      int subsample = _current_scale/2;
      if(subsample<1)
	subsample = 1;
      _projector->project(_zbuffer, _indices, T, _current_zbuffer, _current_indices, _current_scale, subsample);
    }


    float ndist = cos(_normal_angle);
    float pdist = _points_distance * _points_distance;

    const Cloud& reference = *_solver->referenceModel();
    const Cloud& current = *_solver->currentModel();
    
    int search_multiplier=(1+2*_column_search_window)*(1+2*_row_search_window);
    _correspondences.resize(_potential_reference_indices.size()*search_multiplier);
    int ncorr = 0;
    
    for (int row_offset=-_row_search_window; row_offset<=_row_search_window; row_offset++){
      for (int r=0; r<_reference_indices.rows; ++r) {
	int* rptr=_reference_indices.ptr<int>(r);
	int crow=r+row_offset;
	if (crow<0 || crow>=_reference_indices.rows)
	  continue;
	for (int col_offset=-_column_search_window; col_offset<=_column_search_window; col_offset++){
	  int* cptr=_indices.ptr<int>(crow)+col_offset;
	  for (int c=0; c<_reference_indices.cols; ++c, ++rptr, ++cptr) {
	    int ccol=c+col_offset;
	    if (ccol<0 || ccol>=_reference_indices.cols)
	      continue;
	    int ridx = *rptr;
	    int cidx = *cptr;
	    if (ridx <0 || cidx<0)
	      continue;
	    
	    const Vector3f& rp = reference[ridx].point();
	    Vector3f cp=T*current[cidx].point();
	    Vector3f dp = cp-rp;
	    if (dp.squaredNorm()>pdist) {
	      continue;
	    }
	
	    const Vector3f& rn = reference[ridx].normal();
	    Vector3f cn=R*current[cidx].normal();
	    // if (isNan(rn) || isNan(cn)) 
	    //   continue;
	    if (cn.squaredNorm()>0.1 && rn.squaredNorm()>0.1 &&  cn.dot(rn)<ndist ) {
	      continue;
	    }
	    _correspondences[ncorr]=std::make_pair(ridx,cidx);
	    ncorr++;
	  }
	}
      }
    }
    _correspondences.resize(ncorr);
  }

}
