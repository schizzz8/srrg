#include "distance_map.h"
#include "points_utils.h"
using namespace pr;
using namespace std;

int main(int argc, char** argv){
  int rows=480;
  int cols=640;
  int num_points=100;

  // generate n random points
  Vector2fVector points(num_points);
  for (size_t i=0; i<points.size(); i++){
    points[i]=Eigen::Vector2f(rows*drand48(), cols*drand48());
  }
  
  // create an image containing the random points;
  IntImage points_image(rows, cols);
  points_image=-1;

  // copy the points in the image
  putPointIndices(points_image, points);

  // build a distance map
  DistanceMap distance_map;
  FloatImage distance_image(rows, cols);
  IntImage indices_image(rows,cols);
  RGBImage shown_image;

  float d_max=100;
  cv::namedWindow("distance map");
  
  int d_curr=0;

  // show progressive construction of distance map
  while (d_curr<d_max) {
    distance_map.compute(indices_image, distance_image, points_image, d_curr);
    drawDistanceMap(shown_image, distance_image, d_curr-1);
    cv::imshow("distance map", shown_image);
    cv::waitKey(10);
    d_curr++;
  }



}
