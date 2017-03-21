#include "base_correspondence_finder.h"
#include "solver.h"

namespace srrg_nicp {
  
  BaseCorrespondenceFinder::BaseCorrespondenceFinder(Solver* s) {
    _solver = s;
    _normal_angle = .5;
    _points_distance = 0.2;
  }

  BaseCorrespondenceFinder::~BaseCorrespondenceFinder() {}

}
