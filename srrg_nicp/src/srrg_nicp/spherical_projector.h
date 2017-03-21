#pragma once

#include <stack>

#include <srrg_core_map/cloud.h>
#include "base_projector.h"

namespace srrg_nicp {

  /**
     This class encapsulates algorithm and parameters to project a 3D model onto an image,
     through a cylndrical model.
   */
  class SphericalProjector : public BaseProjector{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! ctor, initializes the inner fields
    SphericalProjector();

    /** does the projection, taking into account the normals and the occlusions.
	@param zbuffer: a float matrix used to handle occlusions, filled by the method
	@param indices: an int image, filled by the algorithm. Each cell contains the index of the point in model, that projects onto a pixel
	@param T: the transform world_to_camera applied before the projection
	@param model: input
     */
    virtual void project(srrg_core::FloatImage& zbuffer, 
			 srrg_core::IndexImage& indices,
			 const Eigen::Isometry3f& T,
			 const srrg_core_map::Cloud& model) const;

    // projection from z buffer to z buffer, should be faster
    virtual void project(srrg_core::FloatImage& zbuffer, 
			 srrg_core::IndexImage& indices,
			 const Eigen::Isometry3f& T,
			 const srrg_core::FloatImage& src_zbuffer, 
			 const srrg_core::IntImage& src_indices,
			 float src_scale=1,
			 int subsample = 1) const;
			 

    //! overridden from base class
    //! applies a scaling factor to the camera, not the image  
    //! computes the center so that it is the center of the image
    //! @param s: the scale (s=0.5 corresponds to half the image)
    virtual void scaleCamera(float s);


    //! overridden from base class
    virtual void pushState();

    //! overridden from base class
    virtual void popState();


    virtual bool supportsCameraInfo(srrg_core_map::BaseCameraInfo* _camera_info) const;

    virtual void setCameraInfo(srrg_core_map::BaseCameraInfo* camera_info);

    //! the camera matrix
    inline const Eigen::Vector4f& K() const { return _K;}

    inline void setK(const Eigen::Vector4f& K_) {  _K = K_;}
    
  protected:
    virtual void unprojectPoints(const srrg_core::RawDepthImage& depth_image);

    Eigen::Vector4f _K;
    


    //! method used in a multithreaded run, that operates on a subset of the indices
    void _project(srrg_core::FloatImage& zbuffer, 
		  srrg_core::IndexImage& indices,
		  const Eigen::Isometry3f& T,
		  const srrg_core_map::Cloud& model, int imin, int imax, bool allocate_images = true) const;


    
    int _num_threads;
    mutable std::vector<srrg_core::FloatImage> _zbuffers;
    mutable std::vector<srrg_core::IntImage> _indicess;

    // for push and pop
    struct State {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      State(const Eigen::Vector4f& K, const Eigen::Isometry3f& offset, int r, int c);
      Eigen::Vector4f K;
      Eigen::Isometry3f offset;
      int rows;
      int cols;
    };

    std::stack<State, std::deque< State, Eigen::aligned_allocator<State> > > _states;
    
  private:
    mutable size_t _max_image_size;
    mutable std::vector< std::vector<int> > _indicess_buf;
    mutable std::vector <std::vector<float> >_zbuffers_buf;

    void allocateBuffers() const;
  };

}
