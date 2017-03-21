#include <stdexcept>

#ifdef _GO_PARALLEL_
#include <omp.h>
#endif

#include "spherical_projector.h"
#include <srrg_core_map/spherical_camera_info.h>

namespace srrg_nicp {

  using namespace std;
  using namespace srrg_core;
  using namespace srrg_core_map;

  void SphericalProjector::allocateBuffers() const {
    size_t current_size = _image_cols * _image_rows;
    if (current_size>_max_image_size){
      cerr << "realloc: " << _max_image_size << " ->" << current_size << endl;
      _max_image_size = current_size;
      for (size_t i = 0; i<_num_threads; i++){
	_zbuffers_buf[i].reserve(_max_image_size);
	_indicess_buf[i].reserve(_max_image_size);
      }
    }
    for (size_t i = 0; i<_num_threads; i++){
      _zbuffers_buf[i].resize(current_size);
      _indicess_buf[i].resize(current_size);
      _zbuffers[i]=FloatImage(_image_rows, _image_cols, &(_zbuffers_buf[i][0]));
      _indicess[i]=IntImage(_image_rows, _image_cols, &(_indicess_buf[i][0]));
    }
  }

  SphericalProjector::SphericalProjector() {
    _K <<  M_PI, M_PI/4, 360/M_PI, 360/M_PI;
    _image_cols=_K(0)*_K(2);
    _image_rows=_K(1)*_K(3);
    cerr << __PRETTY_FUNCTION__ 
	 << ": rows= " << _image_rows 
	 << ", cols= " << _image_cols << endl; 
    _min_distance = 0.1;
    _max_distance = 3;
    setIncidenceAngle(0.5* M_PI);

    _max_image_size = 0;
#ifdef _GO_PARALLEL_
    _num_threads = omp_get_max_threads();
    _zbuffers.resize(_num_threads);
    _indicess.resize(_num_threads);
    _zbuffers_buf.resize(_num_threads);
    _indicess_buf.resize(_num_threads);
    cerr << "parallel projector initialized with " << _num_threads << " threads" << endl;
#else //_GO_PARALLEL_
    _num_threads = 1;
    _max_image_size = 0;
    cerr << "sequential projector initialized" << endl;
#endif //_GO_PARALLEL_
    _information_criterion=InverseDistance;
  }

  void SphericalProjector::_project(FloatImage& zbuffer, IndexImage& indices,
				    const Eigen::Isometry3f& T,
				    const Cloud& model, int imin, int imax, 
				    bool allocate_images) const {
    if (allocate_images ) {
      zbuffer.create(_image_rows, _image_cols);
      indices.create(_image_rows, _image_cols);
    }
    zbuffer = 1e9f;
    indices = -1;


    float horizontal_fov=_K(0);
    float vertical_fov=_K(1);
    float horizontal_res=_K(2);
    float horizontal_ires=1./horizontal_res;
    float vertical_res=_K(3);
    float vertical_ires=1./vertical_res;

    int good_points=0;
    for (size_t i = imin; i<(size_t)imax; i++){
      Eigen::Vector3f p=T*model[i].point();
      float p_norm=p.norm();
      if (p_norm<_min_distance || p_norm>_max_distance) 
	continue;
      float theta=atan2(p.y(), p.x());
      if (fabs(theta)>.5*horizontal_fov)			      
	continue;

      float rho=atan2(p.z(), sqrt(p.x()*p.x()+p.y()*p.y()));
      if (fabs(rho)>.5*vertical_fov)
	continue;
      
      // seek the coordinates in the image
      int c=theta*horizontal_res+_image_cols/2;
      int r=rho*vertical_res+_image_rows/2;
      int idx = i;
      if (r<0 || r>=_image_rows ||
	  c<0 || c>=_image_cols)
	continue;

      float& pz=zbuffer.at<float>(r,c);
      {
	if (pz>p_norm){
	  good_points++;
	  pz=p_norm;
	  indices.at<int>(r,c)=idx;
	}
      }
    }
  }

#ifndef _GO_PARALLEL_
  void SphericalProjector::project(FloatImage& zbuffer, IndexImage& indices,
			    const Eigen::Isometry3f& T,
			    const Cloud& model) const {
    _project(zbuffer, indices, _inverse_offset*T, model, 0, model.size());
    return;
  }

#else //_GO_PARALLEL_
  void SphericalProjector::project(FloatImage& zbuffer, IndexImage& indices,
			    const Eigen::Isometry3f& T,
			    const Cloud& model) const {


    int iterationsPerThread =  model.size() / _num_threads;
    allocateBuffers();
#pragma omp parallel
    {
      int threadId = omp_get_thread_num();
      int imin = iterationsPerThread * threadId;
      int imax = imin + iterationsPerThread;
      if((size_t)imax > model.size())
	imax = model.size();
      _project(_zbuffers.at(threadId), _indicess[threadId], _inverse_offset*T, model, imin, imax, false);
    }
    
    // cerr << "reduce" << endl;
    zbuffer.create(_image_rows, _image_cols);
    indices.create(_image_rows, _image_cols);

# pragma omp parallel for
    for (int r=0; r<zbuffer.rows; r++) {
      float *thread_z_ptr[_num_threads];
      int   *thread_idx_ptr[_num_threads];
      // prepare the row indices
      float* z_ptr = zbuffer.ptr<float>(r);
      int* idx_ptr = indices.ptr<int>(r);
      for (int k=0; k<_num_threads; k++){
	thread_z_ptr[k] = _zbuffers[k].ptr<float>(r);
	thread_idx_ptr[k] = _indicess[k].ptr<int>(r);
      }

      for (int c=0; c<zbuffer.cols; c++) {
	float& depth = *z_ptr;
	int& idx = *idx_ptr;
	idx = -1;
	depth = 1e9f;
	for (int k=0; k<_num_threads; k++) {
	  float cdepth = *thread_z_ptr[k];
	  int cidx = *thread_idx_ptr[k];
	  if (cdepth<depth) {
	    depth = cdepth;
	    idx = cidx;
	  }
	  thread_z_ptr[k]++;
	  thread_idx_ptr[k]++;
	}
	z_ptr ++;
	idx_ptr ++;
      }
    }
  }

#endif // _GO_PARALLEL_
  
  bool SphericalProjector::supportsCameraInfo(BaseCameraInfo* camera_info) const {
    SphericalCameraInfo* cam=dynamic_cast<SphericalCameraInfo*>(camera_info);
    return cam;
  }

  void SphericalProjector::setCameraInfo(BaseCameraInfo* camera_info) {
    if (! camera_info) {
      _camera_info = 0;
      return;
    }
    SphericalCameraInfo* cam=dynamic_cast<SphericalCameraInfo*>(camera_info);
    if (! cam) {
      throw std::runtime_error("wrong type of camera for spherical projector");
    }
    _raw_depth_scale=cam->depthScale();
    _camera_info = cam;
    _K=cam->cameraMatrix();
    setOffset(cam->offset());
  }

  void SphericalProjector::project(FloatImage& zbuffer, 
				 IndexImage& indices,
				 const Eigen::Isometry3f& T,
				 const FloatImage& src_zbuffer, 
				 const IntImage& src_indices, 
				 float src_scale,
				 int subsample) const {
    throw std::runtime_error("not yet implemented");
  }


  void SphericalProjector::scaleCamera(float s) {
    _K(2)*=s;
    _K(3)*=s;
  }


  SphericalProjector::State::State(const Eigen::Vector4f& K, const Eigen::Isometry3f& offset, int r, int c) {
    this->K=K;
    this->offset = offset;
    rows = r;
    cols = c;
  }

  void SphericalProjector::pushState() {
    State s(_K, offset(), _image_rows, _image_cols);
    _states.push(s);
  }

  void SphericalProjector::popState() {
    State s = _states.top();
    _K = s.K;
    setOffset(s.offset);
    _image_rows = s.rows;
    _image_cols = s.cols;
    _states.pop();
  }

  void SphericalProjector::unprojectPoints(const RawDepthImage& depth_image) {
    _points.create(depth_image.rows, depth_image.cols);
    _points=cv::Vec3f(0,0,0);

    float horizontal_fov=_K(0);
    float vertical_fov=_K(1);
    float horizontal_res=_K(2);
    float horizontal_ires=1./horizontal_res;
    float vertical_res=_K(3);
    float vertical_ires=1./vertical_res;
    
    int good_points=0;
    for (int r=0; r<depth_image.rows; r++) {
      float rho=(r-depth_image.rows/2)*vertical_ires;
      if (fabs(rho)>.5*vertical_fov)
	continue;
      float s_rho=sin(rho);
      float c_rho=cos(rho);
      const unsigned short* id_ptr  = depth_image.ptr<unsigned short>(r);
      cv::Vec3f* dest_ptr = _points.ptr<cv::Vec3f>(r);
      for (int c=0; c<depth_image.cols; c++) {
	unsigned short id = *id_ptr;
	float theta=(c-depth_image.cols/2)*horizontal_ires;
	if (id>0 && fabs(theta)<.5*horizontal_fov) {
 	  cv::Vec3f& dest = *dest_ptr;
	  float d = id * _raw_depth_scale;
	  if (d<_max_distance && d>_min_distance) {
	    Eigen::Vector3f p(d*c_rho*cos(theta), d*c_rho*sin(theta), d*s_rho);
	    dest = cv::Vec3f(p.x(), p.y(), p.z());
	    good_points++;
	  }
	}
	id_ptr++;
	dest_ptr++;
      }
    }
    cerr << "unproject: " << good_points<< endl;
  }

}
