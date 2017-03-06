#pragma once

#include <vector>
#include <Eigen/Core>
#include "srrg_types/defs.h"

namespace srrg_core {

  class Bresenham {
  public:
    static void line(Vector2iVector& points,  
		     const Eigen::Vector2i& start, 
		     const Eigen::Vector2i& end);
  protected:
    static void lineCore(Vector2iVector& points,  
			 const Eigen::Vector2i& start, 
			 const Eigen::Vector2i& end);
  };

}
