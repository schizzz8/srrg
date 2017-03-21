#pragma once
#include "projective_correspondence_finder.h"
#include "solver.h"
#include "base_aligner.h"

#include <fstream>

namespace srrg_nicp {

  class ProjectiveAligner : public BaseAligner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct LevelParameters{
      LevelParameters(int s=1, int it=10) {
	scale = s;
	iterations = it;
      }
      int scale;
      int iterations;
    };
    //! ctor, if the projector is left null it creates its own
    ProjectiveAligner(BaseProjector* projector=0);
    ~ProjectiveAligner();

    static std::vector<std::string> defaultConfigs();

    void setDefaultConfig(const std::string config);

    inline std::vector<LevelParameters>& levelParameters() {return _level_parameters;}

    virtual float maxDistance() const;

    virtual void setMaxDistance(float md);

    virtual void align(const Eigen::Isometry3f& initial_guess=Eigen::Isometry3f::Identity(),
		       const srrg_core::Matrix6f& initial_guess_information = srrg_core::Matrix6f::Zero());

    inline ProjectiveCorrespondenceFinder& finder() { return _finder; }
    inline BaseProjector& projector() { return *_projector; }

    inline float referenceCanvasScale() const { return _reference_canvas_scale; }
    void setReferenceCanvasScale(float reference_canvas_scale) {_reference_canvas_scale=reference_canvas_scale;}
  protected:
    std::vector<LevelParameters> _level_parameters;
    BaseProjector* _projector;
    ProjectiveCorrespondenceFinder _finder;

    srrg_core::FloatImage _zbuffer;
    srrg_core::IndexImage _indices;
    float _reference_canvas_scale;

    srrg_core::FloatImage _reference_buffer_hres;
    srrg_core::IndexImage _reference_indices_hres;
    srrg_core::FloatImage _current_buffer_hres;
    srrg_core::IndexImage _current_indices_hres;


    static void _initDefaultConfigs();
    static std::map<std::string, std::vector<LevelParameters> > _default_configs;

  };

}
