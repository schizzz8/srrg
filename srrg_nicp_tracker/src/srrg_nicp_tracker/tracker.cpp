#include <srrg_system_utils/system_utils.h>
#include <srrg_nicp/projective_aligner.h>
#include <srrg_nicp/spherical_projector.h>
#include <srrg_core_map/spherical_camera_info.h>
#include <srrg_core_map/pinhole_camera_info.h>
#include "tracker.h"
#include <tr1/memory>
#include <typeinfo>
#include <stdexcept>
#include <srrg_nicp/nn_aligner.h>

namespace srrg_nicp_tracker {

  using namespace std;
  using namespace Eigen;
  using namespace srrg_core;
  using namespace srrg_core_map;
  using namespace srrg_nicp;
  
  Tracker* Tracker::makeTracker(const string& type, const string config) {
    Tracker* dt=0;
    if (type == "projective") {
      cerr << "created a pinhole projective aligner" << endl;
      ProjectiveAligner * al = new ProjectiveAligner(new PinholeProjector);
      dt = new Tracker(al);
      al->setDefaultConfig(config);
      return dt;
    }
    if (type == "spherical") {
      /*
      NNAligner * al=new NNAligner();
      al->solver().setMaxError(0.1);
      al->finder().setPointsDistance(1.0);
      dt = new Tracker(al, new SphericalProjector);

      dt->setReferenceVoxelizeDensity(0.025);
      dt->setCurrentVoxelizeDensity(0.025);
      al->setIterations(30);
      al->solver().setDamping(0);
     return dt;
      */
      cerr << "created a spherical projective aligner" << endl;
      
      ProjectiveAligner * al = new ProjectiveAligner(new SphericalProjector);
      SphericalCameraInfo * spherical_camera=new SphericalCameraInfo;
      Eigen::Isometry3f offset;
      offset.setIdentity();
      spherical_camera->setOffset(offset);
      
      int cols=360;
      int rows=90;
      float horizontal_fov=2*M_PI;
      float vertical_fov=M_PI/2;
      Eigen::Vector4f camera_matrix;
      camera_matrix << horizontal_fov, vertical_fov, cols/horizontal_fov, rows/vertical_fov;
      spherical_camera->setCameraMatrix(camera_matrix);
      al->projector().setImageSize(rows,cols);
      al->projector().setCameraInfo(spherical_camera);
      al->solver().setMaxError(0.01);
      al->finder().setPointsDistance(1.0);
      dt = new Tracker(al, &al->projector());
      al->setDefaultConfig("1Level");
      dt->enableMerging(false);
       return dt;
     
    }
    if (type == "nn") {
      NNAligner * al = new NNAligner();
      dt = new Tracker(al);
      dt->setReferenceVoxelizeDensity(0.025);
      dt->setCurrentVoxelizeDensity(0.025);
      al->setIterations(10);
      return dt;
    }
    return 0;
  }


  Tracker::Trigger::Trigger(Tracker* t, int e, int p) {
    _tracker = t;
    _event = e;
    _priority = p;
    cerr << "Trigger ctor, Events: " << e << endl;
    for (Tracker::EventTriggeMap::iterator it =_tracker->_triggers.begin();
	 it!=_tracker->_triggers.end(); it++){
      if ( (e & it->first) ) {
	Tracker::PriorityTriggerMap::iterator pit = it->second.find(_priority);
	if (pit != it->second.end())
	  throw std::runtime_error("error, trigger with the same priority already exists");
	it->second.insert(make_pair(_priority, this));
	cerr << "  adding trigger:" << this << " for event: " << it->first << " priority: " << p;
      }
    }
  }

  Tracker::Trigger::~Trigger() {
    for (Tracker::EventTriggeMap::iterator it =_tracker->_triggers.begin();
	 it!=_tracker->_triggers.end(); it++){
      if (_event & it->first ) {
	Tracker::PriorityTriggerMap::iterator pit = it->second.find(_priority);
	if (pit == it->second.end())
	  throw std::runtime_error("error, deleting a non existing trigger");
	it->second.erase(pit); 
	cerr << "destroying trigger" << endl;
      }
    }
 }

  Tracker::Tracker(BaseAligner* a, BaseProjector* p){
    if (!a) {
      _aligner=new ProjectiveAligner(new PinholeProjector());
    } else
      _aligner = a;

    _start_time = 0;
    _current_time = 0;
    _make_cloud_time = 0;
    _alignment_time = 0;
    _validate_time = 0;
    _merge_time = 0;
    _tail_time = 0;
    
    _reference = 0;
    _current = 0;
    _merging_gain = .5;
    _total_points = 0;
    _image_shrink = 1;

    _global_transform.setIdentity();
    _merging_enabled = true;
    _reference_voxelize_density = 0.005;
    _current_voxelize_density = 0;
    _reference_offset.setIdentity();
    _bad_points_ratio = 0.1;
    _inlier_distance_threshold = 0.1;
    _merging_distance = 0.1;

    // populate the trigger map
    _triggers.insert(make_pair(NEW_FRAME_CREATED, PriorityTriggerMap()));
    _triggers.insert(make_pair(NEW_CAMERA_ADDED, PriorityTriggerMap()));
    _triggers.insert(make_pair(ALIGNMENT_DONE, PriorityTriggerMap()));
    _triggers.insert(make_pair(TRACK_GOOD, PriorityTriggerMap()));
    _triggers.insert(make_pair(TRACK_BROKEN, PriorityTriggerMap()));
    _triggers.insert(make_pair(TRACKING_DONE, PriorityTriggerMap()));
    _triggers.insert(make_pair(PROCESSING_DONE, PriorityTriggerMap()));
    _triggers.insert(make_pair(REFERENCE_FRAME_RESET, PriorityTriggerMap()));
    _track_broken = false;
    _last_odom.setIdentity();
    _last_camera_offset.setIdentity();
    _last_initial_guess.setIdentity();
    _last_topic="";
    if (!p)
      _projector=new PinholeProjector();
    else
      _projector=p;
    _frame_count = 0;
    _frame_skip = 0;
    resetInformationMatrix();

    _changes_threshold=0;
    tracker_debug_stream.open("/home/giorgio/tracker_debug.txt");
  }

  void Tracker::setFrameSkip(int skip) {_frame_skip = skip;}

  void Tracker::callTriggers(TriggerEvent event){
    Tracker::EventTriggeMap::iterator it = _triggers.find(event);
    if (it == _triggers.end()) {
      throw std::runtime_error("error, unsupported event in tracker");
    }
    
    for (PriorityTriggerMap::iterator pit = it->second.begin(); pit!= it->second.end(); pit++){
      pit->second->action(event);
    }
  }

  void Tracker::reprojectBoth(int rows, int cols, float scale){
    _projector->pushState();
    _projector->setImageSize(scale*rows, scale*cols);
    _projector->scaleCamera(scale);
    _projector->project(_cur_buffer, _cur_indices, Eigen::Isometry3f::Identity(), *_current);
    _projector->project(_ref_buffer, _ref_indices, Eigen::Isometry3f::Identity(), *_reference);
    _projector->popState();
  }

  void Tracker::computeTrackBroken() {
    float in_distance, out_distance;
    int in_num, out_num;
    compareDepths(in_distance, in_num, out_distance, out_num,
		  _cur_buffer, _cur_indices,
		  _ref_buffer, _ref_indices, 
		  _inlier_distance_threshold);
      
    _last_outliers_ratio = (float)out_num/(float)(in_num+out_num);
    // cerr << " good: " << in_distance/in_num << " p: " << in_num;
    // cerr << " bad: " << out_distance/out_num << " p: " << out_num;
    // cerr << " ratio: " << _last_outliers_ratio  << endl;

    _track_broken=_last_outliers_ratio > _bad_points_ratio;
  }


  void Tracker::updateInformation() {
    if (_information_matrix_reset) {
      _information_matrix_origin.setIdentity();
      _information_matrix.setZero();
      _information_matrix_reset = false;
    } else {
      _information_matrix_origin=_information_matrix_origin*_aligner->T();
      _information_matrix = _aligner->solver().remapInformationMatrix(_information_matrix_origin);
    }
  }

  void Tracker::processFrame(const RawDepthImage& depth, 
			     const RGBImage& rgb, 
			     const Eigen::Matrix3f& K, 
			     float depthScale, 
			     int seq, 
			     double timestamp,
			     const std::string& topic,
			     const std::string& frame_id,
			     const Eigen::Isometry3f& sensor_offset_, 
			     const Eigen::Isometry3f& odom_guess,
			     const Matrix6f& odom_info){
	
    _frame_count ++;
    if (_frame_skip && (_frame_count%_frame_skip))
      return;

    _last_seq = seq;
    _start_time  = getTime();
    //_last_K = K;
    _last_depth_scale = depthScale;
    _last_topic = topic;
    _last_initial_guess = odom_guess;
    _last_timestamp = timestamp;
    
    Eigen::Matrix3f myK = K;
    _last_raw_rgb=RGBImage();
    if (_image_shrink>1) {
      shrinkRawDepth(_last_raw_depth, depth, _image_shrink);
      //myK.block<2,3>(0,0)*= (1./_image_shrink);
      if (rgb.rows && rgb.cols)
	cv::resize(rgb, _last_raw_rgb,  cv::Size(0,0), 1./_image_shrink, 1./_image_shrink, cv::INTER_LINEAR);
    } else {
      _last_raw_depth = depth;
      if (rgb.rows && rgb.cols)
	_last_raw_rgb = rgb;
    }
    
    BaseCameraInfo* cam = _cameras.getCamera(topic);
    bool new_camera = false;
    if (! cam) {
      cam=new PinholeCameraInfo(topic, frame_id, myK, sensor_offset_, depthScale);
      _cameras.addCamera(cam);
      new_camera = true;
    } 
    _last_camera = cam;
    _last_camera_offset = cam->offset();
    if (new_camera)
      callTriggers(NEW_CAMERA_ADDED);

    std::tr1::shared_ptr<BaseCameraInfo> internal_camera(cam->scale(1./_image_shrink));
    
    if (depth.cols == 0 && depth.rows == 0)
      return;

    if (_current && _current!=_reference)
      delete _current;
    

    _make_cloud_time = getTime();
    // build the cloud from the image
    _current = new Cloud;
 
    _projector->setImageSize(_last_raw_depth.rows, _last_raw_depth.cols);
    _projector->setCameraInfo(internal_camera.get());
    _projector->unproject(*_current, _last_raw_depth, _last_raw_rgb);

   _current_time = getTime();

    _make_cloud_time = getTime() - _make_cloud_time;
    callTriggers(NEW_FRAME_CREATED);

    // make it sparse if necessary
    if (_current_voxelize_density>0){
      voxelize(*_current, _current_voxelize_density);
    }
    
    _track_broken = false;
    // if it is not the first cloud, do the work
    if (_reference ) {
      Eigen::Isometry3f odom_delta = _last_odom.inverse()*odom_guess;
      Eigen::AngleAxisf aa(odom_delta.linear());
//      if(odom_delta.translation().norm() > 1.0f || fabs(aa.angle()) > M_PI_2) {
//	odom_delta = Eigen::Isometry3f::Identity();
//      }

      if (! new_camera) {
	// registration
	_alignment_time = getTime();
	_aligner->setReferenceModel(_reference);
	{
	  ProjectiveAligner* pal= dynamic_cast<ProjectiveAligner*>(_aligner);
	  if(pal)
	    if (pal->projector().supportsCameraInfo(internal_camera.get())) {
	      pal->projector().setCameraInfo(internal_camera.get());
	    } else {
	      cerr << "warning, unsupported camera info in tracker" << endl;
	    }
	}
	_aligner->setCurrentModel(_current);
	_aligner->align(odom_delta, odom_info);
	//cerr << "igi" << endl;
	//cerr << odom_info << endl;

	_alignment_time = getTime() - _alignment_time;
	_current_time = getTime();
	callTriggers(ALIGNMENT_DONE);
	updateInformation();

        

	_validate_time = getTime();
	// see if the track is broken by doing some reprojections anc omparing the images
	_current->transformInPlace(_aligner->T());
	reprojectBoth(_last_raw_depth.rows, _last_raw_depth.cols, _merging_gain);
	computeTrackBroken();
	_validate_time = getTime() - _validate_time;

	_current_time = getTime();
	if (_track_broken) {
	  callTriggers(TRACK_BROKEN);
	} else {
	  callTriggers(TRACK_GOOD);
	}
      } else {
	_aligner->solver().setT(odom_delta);
	_current->transformInPlace(odom_delta);
	_reference->add(*_current);
      }

      if (!_track_broken) {
	if (_merging_enabled) {
	  _merge_time = getTime();
 	  merge(_ref_buffer, _ref_indices, *_reference, 
		_cur_buffer, _cur_indices, *_current, 
		_merging_distance);
	  _merge_time = getTime() - _merge_time;
	}	
	computeChanges();
      }
      _tail_time = getTime();
      _global_transform = _global_transform * _aligner->T();
      Eigen::Isometry3f iso=_aligner->T().inverse();
      _reference->transformInPlace(iso);
      _tail_time = getTime() - _tail_time;

    }
    callTriggers(TRACKING_DONE);
    
    if (! _reference) {
      _reference = _current;
      _current_time = getTime();
      callTriggers(REFERENCE_FRAME_RESET);
    }
    _reference_offset = _last_camera_offset;

    _current_time = getTime();
    callTriggers(PROCESSING_DONE);
    _last_odom = odom_guess;
  }						

  void Tracker::processSphericalFrame(const RawDepthImage& depth, 
				      const RGBImage& rgb, 
				      const Eigen::Vector4f& K, 
				      float depthScale, 
				      int seq, 
				      double timestamp,
				      const std::string& topic,
				      const std::string& frame_id,
				      const Eigen::Isometry3f& sensor_offset_, 
				      const Eigen::Isometry3f& odom_guess,
				      const Matrix6f& odom_info){
	
    _frame_count ++;
    if (_frame_skip && (_frame_count%_frame_skip))
      return;

    _last_seq = seq;
    _start_time  = getTime();
    //_last_K = K;
    _last_depth_scale = depthScale;
    _last_topic = topic;
    _last_initial_guess = odom_guess;
    _last_timestamp = timestamp;
    
    cerr << "camera matrix: " << K.transpose() << endl;
    Eigen::Vector4f myK = K;
    _last_raw_rgb=RGBImage();
    if (_image_shrink>1) {
      shrinkRawDepth(_last_raw_depth, depth, _image_shrink);
      //myK.block<2,3>(0,0)*= (1./_image_shrink);
      if (rgb.rows && rgb.cols)
	cv::resize(rgb, _last_raw_rgb,  cv::Size(0,0), 1./_image_shrink, 1./_image_shrink, cv::INTER_LINEAR);
    } else {
      _last_raw_depth = depth;
      if (rgb.rows && rgb.cols)
	_last_raw_rgb = rgb;
    }
    
    BaseCameraInfo* cam = _cameras.getCamera(topic);
    bool new_camera = false;
    if (! cam) {
      cam=new SphericalCameraInfo(topic, frame_id, myK, sensor_offset_, depthScale);
      _cameras.addCamera(cam);
      new_camera = true;
    } 
    _last_camera = cam;
    _last_camera_offset = cam->offset();
    if (new_camera)
      callTriggers(NEW_CAMERA_ADDED);

    std::tr1::shared_ptr<BaseCameraInfo> internal_camera(cam->scale(1./_image_shrink));
    
    if (depth.cols == 0 && depth.rows == 0)
      return;

    if (_current && _current!=_reference)
      delete _current;
    

    _make_cloud_time = getTime();
    // build the cloud from the image
    _current = new Cloud;
 
    _projector->setImageSize(_last_raw_depth.rows, _last_raw_depth.cols);
    _projector->setCameraInfo(internal_camera.get());
    _projector->unproject(*_current, _last_raw_depth, _last_raw_rgb);

    char cloud_name[1024];
    sprintf(cloud_name, "cloud-%05d.dat", seq);
    ofstream os_cloud(cloud_name);
    _current->write(os_cloud);
    os_cloud.close();
   _current_time = getTime();

    _make_cloud_time = getTime() - _make_cloud_time;
    callTriggers(NEW_FRAME_CREATED);

    // make it sparse if necessary
    if (_current_voxelize_density>0){
      voxelize(*_current, _current_voxelize_density);
    }
    
    _track_broken = false;
    // if it is not the first cloud, do the work

    Eigen::Isometry3f odom_delta;
    odom_delta.setIdentity();
    if (_reference ) {
      //Eigen::Isometry3f odom_delta = _last_odom.inverse()*odom_guess;
      odom_delta = _last_odom.inverse()*odom_guess;
      
      // float delta_translation=0.7;
      // float delta_orientation=0.7;
      // odom_delta.translation()*=delta_translation;
      // Eigen::AngleAxisf aa(odom_delta.linear());
      // odom_delta.linear()=AngleAxisf(aa.angle()*delta_orientation, aa.axis()).toRotationMatrix();
      cerr << "Odom delta: " << t2v(odom_delta).transpose() << endl;
      if (! new_camera) {
	// registration
	_alignment_time = getTime();
	_aligner->setReferenceModel(_reference);
	{
	  ProjectiveAligner* pal= dynamic_cast<ProjectiveAligner*>(_aligner);
	  if(pal)
	    if (pal->projector().supportsCameraInfo(internal_camera.get())) {
	      pal->projector().setCameraInfo(internal_camera.get());
	    } else {
	      cerr << "warning, unsupported camera info in tracker" << endl;
	    }
	}
	_aligner->setCurrentModel(_current);
	_aligner->align(odom_delta, odom_info);
	//cerr << "igi" << endl;
	//cerr << odom_info << endl;

	_alignment_time = getTime() - _alignment_time;
	_current_time = getTime();
	callTriggers(ALIGNMENT_DONE);
	updateInformation();

        

	_validate_time = getTime();
	// see if the track is broken by doing some reprojections anc omparing the images
	_current->transformInPlace(_aligner->T());
	reprojectBoth(_last_raw_depth.rows, _last_raw_depth.cols, _merging_gain);
	computeTrackBroken();
	_validate_time = getTime() - _validate_time;

	_current_time = getTime();
	if (_track_broken) {
	  callTriggers(TRACK_BROKEN);
	} else {
	  callTriggers(TRACK_GOOD);
	}
      } else {
	_aligner->solver().setT(odom_delta);
	_current->transformInPlace(odom_delta);
	_reference->add(*_current);
      }

      if (!_track_broken) {
	if (_merging_enabled) {
	  _merge_time = getTime();
 	  merge(_ref_buffer, _ref_indices, *_reference, 
		_cur_buffer, _cur_indices, *_current, 
		_merging_distance);
	  _merge_time = getTime() - _merge_time;
	} else {
	  _reference->add(*_current);
	}

      }
      _tail_time = getTime();
      _global_transform = _global_transform * _aligner->T();
      Eigen::Isometry3f iso=_aligner->T().inverse();
      _reference->transformInPlace(iso);
      _tail_time = getTime() - _tail_time;

    }
    callTriggers(TRACKING_DONE);
    
    if (! _reference) {
      _reference = _current;
      _current_time = getTime();
      callTriggers(REFERENCE_FRAME_RESET);
    }
    _reference_offset = _last_camera_offset;

    _current_time = getTime();
    callTriggers(PROCESSING_DONE);
    _last_odom = odom_guess;
    tracker_debug_stream << t2v(_global_transform).transpose() << endl;

    Eigen::Isometry3f difference_between_input_and_output=_aligner->T().inverse()*odom_delta;
    cerr << "delta is: " << t2v(difference_between_input_and_output).transpose() << endl;
  }						

  void Tracker::clearStatus() {
    if (_reference && _current!=_reference) {
      delete _reference;
      _reference = 0;
      // if this became the current transform, we need to
      // bring it back to the original reference frame.
      if (_current)
	_current->transformInPlace(_aligner->T().inverse());
    }
    _total_points = 0;
    resetInformationMatrix();
  }

  void Tracker::computeChanges() {
    if (_changes_threshold>0.0f && _reference && _current){
      _projector->setImageSize(_last_raw_depth.rows, _last_raw_depth.cols);
      _changes_current_indices=-1;
      _changes_reference_indices=-1;
      _projector->project(_changes_reference_depth, 
			  _changes_reference_indices,
			  Eigen::Isometry3f::Identity(),
			  *_reference);
      _projector->project(_changes_current_depth, 
			  _changes_current_indices,
			  Eigen::Isometry3f::Identity(),
			  *_current);
      _changes_mask=(_changes_reference_indices>-1)&(_changes_current_indices>-1);
      _changes_image.create(_last_raw_depth.rows, _last_raw_depth.cols);
      _changes_image=0;
      _changes_image=(abs(_changes_reference_depth-_changes_current_depth)>_changes_threshold)&_changes_mask;
      for (int r=0; r<_changes_reference_depth.rows; ++r){
	int* current_idx_ptr=_changes_current_indices.ptr<int>(r);
        int* reference_idx_ptr=_changes_reference_indices.ptr<int>(r);
        unsigned char* changes_ptr=_changes_image.ptr<unsigned char>(r);
	for (int c=0; c<_changes_reference_depth.cols; 
	     ++c, ++changes_ptr, ++reference_idx_ptr, ++current_idx_ptr) {
	  if (*current_idx_ptr<0)
	    continue;
	  if (*reference_idx_ptr<0)
	    continue;
	  if (*changes_ptr) {
	    _reference->at(*reference_idx_ptr)._rgb=Eigen::Vector3f(1,0,0); 
	    _current->at(*current_idx_ptr)._rgb=Eigen::Vector3f(1,0,0); 
	  } else {
	    _reference->at(*reference_idx_ptr)._rgb=Eigen::Vector3f(0,1,0);
	    _current->at(*current_idx_ptr)._rgb=Eigen::Vector3f(0,1,0);
	  }
 	}
      }
    }
  }

  void Tracker::resetInformationMatrix() {
    _information_matrix.setZero();
    _information_matrix_reset=true;
  }

  void Tracker::setMaxDistance(float max_distance) {
    if (_projector)
      _projector->setMaxDistance(max_distance);
    _aligner->setMaxDistance(max_distance);
  }

  void Tracker::setMinDistance(float min_distance) {
    if (_projector)
      _projector->setMinDistance(min_distance);
  }

}
