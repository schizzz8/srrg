#include <fstream>
#include <stdexcept>

#include <srrg_boss/deserializer.h>
#include <srrg_boss/trusted_loaders.h>
#include <srrg_system_utils/system_utils.h>
#include "srrg_core_map/image_map_node.h"
#include "srrg_core_map/local_map.h"

using namespace std;
using namespace srrg_boss;
using namespace srrg_core;
using namespace srrg_core_map;

// Help objects to force linking 
BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
BinaryNodeRelation rel;

const char* banner[] = {
  "srrg_trajectory_loader_app: example on how to load a set of boss objects constituting a boss map",
  "usage:",
  " srrg_trajectory_loader_app <boss log file>",
  0
};

int main(int argc, char** argv) {
  if(argc < 2 || !strcmp(argv[1],"-h")) {
    printBanner(banner);
    return 0 ;
  }
  
  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(argv[1]);
  Serializable* o;
  while((o = des.readObject())) {
    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;

  return 0;
}
