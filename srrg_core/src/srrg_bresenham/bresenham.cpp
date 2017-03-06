#include "bresenham.h"
namespace srrg_core {

  void Bresenham::lineCore(Vector2iVector& points,  
			   const Eigen::Vector2i& start, 
			   const Eigen::Vector2i& end) {
    int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
    int cnt = 0;

    dx = std::abs(end.x() - start.x());
    dy = std::abs(end.y() - start.y());
    points.resize(dx+dy);
  
    if(dy <= dx) {
      d = 2 * dy - dx; 
      incr1 = 2 * dy; 
      incr2 = 2 * (dy - dx);
      if(start.x() > end.x()) {
	x = end.x(); 
	y = end.y();
	ydirflag = (-1);
	xend = start.x();
      } 
      else {
	x = start.x(); 
	y = start.y();
	ydirflag = 1;
	xend = end.x();
      }
      points[cnt].x() = x;
      points[cnt].y() = y;
      cnt++;
      if(((end.y() - start.y()) * ydirflag) > 0) {
	while(x < xend) {
	  x++;
	  if(d < 0) {
	    d += incr1;
	  } 
	  else {
	    y++; 
	    d += incr2;
	  }
	  points[cnt].x() = x;
	  points[cnt].y() = y;
	  cnt++;
	}
      } 
      else {
	while(x < xend) {
	  x++;
	  if(d < 0) {
	    d += incr1;
	  } 
	  else {
	    y--; 
	    d += incr2;
	  }
	  points[cnt].x() = x;
	  points[cnt].y() = y;
	  cnt++;
	}
      }		
    } 
    else {
      d = 2 * dx - dy;
      incr1 = 2 * dx; 
      incr2 = 2 * (dx - dy);
      if(start.y() > end.y()) {
	y = end.y(); 
	x = end.x();
	yend = start.y();
	xdirflag = (-1);
      } 
      else {
	y = start.y(); 
	x = start.x();
	yend = end.y();
	xdirflag = 1;
      }
      points[cnt].x() = x;
      points[cnt].y() = y;
      cnt++;
      if(((end.x() - start.x()) * xdirflag) > 0) {
	while(y < yend) {
	  y++;
	  if(d < 0) {
	    d += incr1;
	  } 
	  else {
	    x++; 
	    d += incr2;
	  }
	  points[cnt].x()=x;
	  points[cnt].y()=y;
	  cnt++;
	}
      } 
      else {
	while(y < yend) {
	  y++;
	  if(d < 0) {
	    d += incr1;
	  } 
	  else {
	    x--; 
	    d += incr2;
	  }
	  points[cnt].x() = x;
	  points[cnt].y() = y;
	  cnt++;
	}
      }
    }
    points.resize(cnt);
  }

  void Bresenham::line(Vector2iVector& points,  
		       const Eigen::Vector2i& start, 
		       const Eigen::Vector2i& end) {
    int i, j;
    int half;
    Eigen::Vector2i v;
    lineCore(points, start, end);
    int num_points=points.size();
    if(start.x() != points[0].x() || start.y() != points[0].y()) {
      half = num_points / 2;
      for(i = 0, j = num_points - 1; i < half; i++, j--) {
	v = points[i];
	points[i] = points[j];
	points[j] = v;
      }
    }
  }

}
