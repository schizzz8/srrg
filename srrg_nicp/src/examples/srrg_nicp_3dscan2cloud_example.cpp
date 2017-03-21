#include <sstream>
#include <fstream>

#include "srrg_nicp/depth_utils.h"
#include <srrg_core_map/spherical_camera_info.h>
#include "srrg_nicp/spherical_projector.h"
#include <srrg_system_utils/system_utils.h>

using namespace std;
using namespace srrg_core;
using namespace srrg_boss;
using namespace srrg_core_map;
using namespace srrg_nicp;

void loadRawCloud (Vector3fVector& points, istream& is) {
  while(is) {
    char line_buf[1024];
    is.getline(line_buf, 1024);
    istringstream iss(line_buf);
    Eigen::Vector3f p;
    iss >> p.x();
    iss >> p.y();
    iss >> p.z();
    points.push_back(p);
  }
}

void makeSphericalImage(FloatImage& depth, 
			const Vector3fVector& points,
			float horizontal_fov=M_PI,
			float vertical_fov=M_PI/2,
			float horizontal_res=360/M_PI,
			float vertical_res=360/M_PI,
			float range_max=30.0f){
  // compute the size of the image
  int cols=ceil(2*horizontal_fov*horizontal_res);
  int rows=ceil(2*vertical_fov*vertical_res);
  depth.create(rows,cols);
  depth=range_max;
  for (size_t i=0; i<points.size(); i++){
    const Eigen::Vector3f p=points[i];
    float range=p.norm();
    if (range>range_max){
      continue;
    }
    // compute azimuth and elevation, in polar coordinates
    float azimuth=atan2(p.y(),p.x());
    if (fabs(azimuth)>horizontal_fov){
      continue;
    }
    float xy_norm=p.head<2>().norm();
    float elevation=atan2(p.z(),xy_norm);
      
    if (fabs(elevation)>vertical_fov){
      continue;
    }
    // compute the binning
    float r=rows/2+round(vertical_res*elevation);
    float c=cols/2+round(horizontal_res*azimuth);
    if (r<0||r>=rows) {
      continue;
    }
    if (c<0||c>=cols)
      continue;
    float &d=depth.at<float>(r,c);
    if (range<d){
      d=range;
    }
  }
}

const char* banner[] = {
  "srrg_nicp_3dscan2cloud_example: example on how to convert a point cloud from txt file to nicp format using a spherical projector",
  "usage:",
  " srrg_nicp_3dscan2cloud_example <txt file with x, y, z point coordinates one per line> <model.dat>",
  0
};

int main(int argc, char** argv) {
  if(argc < 3) {
    printBanner(banner);
    return 0;
  }

  ifstream is1(argv[1]);
  if(!is1) {
    cerr << "unable to load file " << argv[1] << endl;
    return 0;
  }
  
  cerr << "loading points" << endl;  
  Vector3fVector points;
  loadRawCloud(points, is1);
  cerr << "read " << points.size() << " points" << endl;
  

  cerr << "making spherical image" << endl;
  FloatImage spherical_float_depth;

  float hfov = 1.5 * M_PI;
  float hres = 360 / M_PI;
  float vfov = 0.5 * M_PI;
  float vres = 360 / M_PI;
  float depth_scale = 1e2; // resolution: 1cm/pixel
  Eigen::Vector4f K;
  K << hfov, vfov, hres, vres;
  makeSphericalImage(spherical_float_depth, 
		     points,
		     hfov, vfov, hres, vres, 30.0f);
   
  cv::imshow("depth", spherical_float_depth);
  cv::waitKey();
  UnsignedShortImage spherical_short_image;
  convert_32FC1_to_16UC1(spherical_short_image, spherical_float_depth, depth_scale);
  
  SphericalProjector projector;
  SphericalCameraInfo camera_info;
  camera_info.setCameraMatrix(K);
  camera_info.setDepthScale(1.0/depth_scale);
  projector.setCameraInfo(&camera_info);
  projector.setImageSize(spherical_float_depth.rows, spherical_float_depth.cols);
  projector.setMaxDistance(50);
  projector.setCrossProductMaxDistance(2);
  cerr << "generating cloud to file " << argv[2] << endl;
 
  Cloud dest_model;
  projector.unproject(dest_model,spherical_short_image);
  ofstream os(argv[2]);
  dest_model.write(os);

  return 0;
}
