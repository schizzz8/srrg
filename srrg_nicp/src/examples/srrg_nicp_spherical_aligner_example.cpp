#include <fstream>

#include "srrg_nicp/depth_utils.h"
#include "srrg_nicp/projective_aligner.h"
#include "srrg_nicp/spherical_projector.h"
#include <srrg_system_utils/system_utils.h>

using namespace std;
using namespace srrg_core;
using namespace srrg_core_map;
using namespace srrg_nicp;

const char* banner[] = {
  "srrg_nicp_spherical_aligner_example: example on how to register two point clouds using a spherical projection based aligner",
  "usage:",
  " srrg_nicp_spherical_aligner_example <model1.dat> <model2.dat> <output.dat>",
  0
};

int main(int argc, char** argv) {
  if(argc < 4) {
    printBanner(banner);
    return 0;
  }

  ifstream is1(argv[1]);
  if(!is1) {
    cerr << "unable to load file " << argv[1] << endl;
    return 0;
  }

  ifstream is2(argv[2]);
  if(!is2) {
    cerr << "unable to load file " << argv[2] << endl;
    return 0;
  }

  cerr << "loading models" << endl;
  Cloud reference;
  reference.read(is1);
  cerr << "reference has " << reference.size() << " points" << endl;
  
  Cloud current;
  current.read(is2);
  cerr << "current has " << current.size() << " points" << endl;

  SphericalProjector* projector = new SphericalProjector();

  ProjectiveAligner aligner(projector);
  aligner.projector().setMaxDistance(100.0);
  aligner.finder().setPointsDistance(1.0);
  aligner.solver().setMaxError(0.01);
  aligner.setDefaultConfig("1Level");
  aligner.solver().setDamping(0);
  aligner.setReferenceModel(&reference);
  aligner.setCurrentModel(&current);
  aligner.align(Eigen::Isometry3f::Identity());
  
  float factor = 255.0f/6;

  FloatImage img;
  sub_32FC1(img, aligner.finder().zBuffer(), aligner.finder().referenceZBuffer());
  cv::imwrite("difference.png", img * factor);
  cv::imwrite("current.png", aligner.finder().zBuffer() * factor);
  cv::imwrite("reference.png", aligner.finder().referenceZBuffer() * factor);
  
  cerr << "T: " << endl << aligner.T().matrix() << endl;
  
  current.transformInPlace(aligner.T());
  reference.add(current);
  ofstream os(argv[3]);
  reference.write(os);

  delete projector;

  return 0;
}
