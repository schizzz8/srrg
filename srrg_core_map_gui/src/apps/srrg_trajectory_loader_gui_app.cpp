#include <fstream>
#include <stdexcept>

#include <qapplication.h>

#include <srrg_boss/deserializer.h>
#include <srrg_boss/trusted_loaders.h>
#include <srrg_core_map/cloud.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_core_map/image_map_node.h>
#include <srrg_core_map/local_map.h>
#include "srrg_core_map_viewers/trajectory_viewer.h"

using namespace std;
using namespace srrg_boss;
using namespace srrg_core;
using namespace srrg_core_map;
using namespace srrg_core_map_gui;

// Help objects to force linking 
BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
BinaryNodeRelation rel;

const char* banner[] = {
  "srrg_trajectory_loader_gui_app: load and show a set of boss objects constituting a boss map",
  "usage:",
  " srrg_trajectory_loader_gui_app <boss log file>",
  0
};

int main(int argc, char** argv) {
  if(argc < 2 || !strcmp(argv[1], "-h")) {
    printBanner(banner);
    return 0 ;
  }
  
  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(argv[1]);
  Serializable* o;
  MapNodeList lmaps;
  BinaryNodeRelationSet rels;
  while((o = des.readObject()) ){
    LocalMap* lmap = dynamic_cast<LocalMap*>(o);
    if(lmap) {
      lmaps.addElement(lmap);
    }
    BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(o);
    if(rel) {
      LocalMap* from = dynamic_cast<LocalMap*>(rel->from());
      LocalMap* to = dynamic_cast<LocalMap*>(rel->to());      
      if(from && to) { 
	rels.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
      }
    }
    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;
  cerr << "Read: " << lmaps.size() << " local maps" << endl;
  cerr << "Read: " << rels.size() << " binary relations" << endl;

  QApplication app(argc, argv);
  TrajectoryViewer viewer;
  viewer.nodes = lmaps;
  viewer.relations = rels;
  viewer.show();
  app.exec();

  return 0;
}
