#pragma once

#include "nn_correspondence_finder.h"
#include "solver.h"
#include "base_aligner.h"

#include <fstream>

namespace srrg_nicp {

  class NNAligner : public BaseAligner{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    NNAligner();
    virtual ~NNAligner();

    void align(const Eigen::Isometry3f& initial_guess = Eigen::Isometry3f::Identity(),
	       const srrg_core::Matrix6f& initial_guess_information = srrg_core::Matrix6f::Zero());
    inline NNCorrespondenceFinder& finder() { return _finder; }
    inline int iterations() const {return _iterations;}
    inline void setIterations(int i) {_iterations = i;}

    virtual float maxDistance() const;
    virtual void setMaxDistance(float md);

  protected:

    int _iterations;
    NNCorrespondenceFinder _finder;
  };

}
