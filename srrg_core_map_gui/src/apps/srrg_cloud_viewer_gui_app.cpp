#include <iostream>
#include <fstream>
#include <limits>
#include <deque>
#include <queue>
#include <vector>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <qapplication.h>
#include <qglviewer.h>

#include <srrg_system_utils/system_utils.h>
#include "srrg_core_map_viewers/cloud_viewer.h"

using namespace std;
using namespace Eigen;
using namespace srrg_core;
using namespace srrg_core_map;
using namespace srrg_core_map_gui;

const char* banner[] = {
  "srrg_cloud_viewer_gui_app",
  "shows a set of clouds, shift + left click to select clouds in the viewer",
  "usage:",
  " srrg_cloud_viewer_gui_app <cloud1.dat> <cloud2.dat> ...",
  0
};

int main(int argc, char** argv) {
  std::list<Cloud*> clouds;
  if(argc < 2 || !strcmp(argv[1], "-h")) { 
    printBanner(banner);
    return 0;
  }

  int c = 1;
  while(c < argc) {
    Cloud* cloud = new Cloud;
    ifstream is(argv[c]);
    cloud->read(is);
    clouds.push_back(cloud);
    cerr << "loaded cloud [" << argv[c] << "] with " << cloud->size() << "] points" << endl;
    c++;
  }
  
  QApplication app(argc, argv);
  CloudViewer viewer;
  for(std::list<Cloud*>::iterator it = clouds.begin(); it != clouds.end(); ++it) {
    viewer.addCloud(*it);
  }  
  viewer.show();
  app.exec();

  for(std::list<Cloud*>::iterator it = clouds.begin(); it != clouds.end(); ++it) {
    delete *it;
  }  
  
  return 0;
}
