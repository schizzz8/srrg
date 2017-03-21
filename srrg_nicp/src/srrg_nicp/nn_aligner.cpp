#include "nn_aligner.h"
#include <stdexcept>

namespace srrg_nicp {

using namespace std;
using namespace Eigen;


  NNAligner::NNAligner():
    _finder(&_solver) {
    _solver.setMaxError(.01);
    _solver.setDamping(100);
    _solver.setGICP(false);
    setMaxDistance(0.1);
    _iterations = 10;
  }

  NNAligner::~NNAligner() {}

  float NNAligner::maxDistance() const {return _finder.pointsDistance(); }

  void NNAligner::setMaxDistance(float md)  {_finder.setPointsDistance(md); }

  void NNAligner::align(const Eigen::Isometry3f& initial_guess,
 			const srrg_core::Matrix6f& initial_guess_information){
    if (! _solver.referenceModel()) {
      throw std::runtime_error("NNAligner: align(), reference model not set");
    }
    if (!_solver.currentModel()){
      throw std::runtime_error("NNAligner: align(), current model not set");
    }
    if (initial_guess_information == srrg_core::Matrix6f::Zero()) {
      _solver.setT(initial_guess);
    } else 
      _solver.setT(initial_guess,initial_guess_information);
    
    _finder.init();
    callTriggers(BaseAligner::Initialized);
    for(int i = 0; i<_iterations; i++){
      callTriggers(BaseAligner::Iteration);
      _finder.compute();
      callTriggers(BaseAligner::Correspondences);
      const BaseCorrespondenceFinder::CorrespondenceVector& corr=_finder.correspondences();
      bool computeStats = (i == _iterations-1);
      _solver.oneRound(corr, computeStats);
      callTriggers(BaseAligner::Optimization);
    }
  }
}
