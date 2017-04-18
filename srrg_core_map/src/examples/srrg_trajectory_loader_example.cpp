#include <fstream>
#include <stdexcept>

#include <srrg_boss/deserializer.h>
#include <srrg_boss/serializer.h>
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

    string input_filename = argv[1];

    int count = 4;
    std::list<Serializable*> objects;
    Deserializer des;
    des.setFilePath(input_filename);
    Serializable* o;
    while((o = des.readObject())) {
        LocalMap* lmap = dynamic_cast<LocalMap*>(o);
        if (lmap && count > 0){
            cerr << "Local map: " << lmap->getId() << endl;
            cerr << "\t>> " << lmap->cloud()->size() << " points" << endl;
            cerr << "\t>> " << lmap->nodes().size() << " nodes" << endl;
            cerr << "\t>> " << lmap->relations().size() << " relations" << endl;

            for(MapNodeList::iterator it = lmap->nodes().begin();
                it != lmap->nodes().end();
                it++){
                objects.push_back(it->get());
            }

            for(BinaryNodeRelationSet::iterator it = lmap->relations().begin();
                it != lmap->relations().end();
                it++){
                objects.push_back(it->get());
            }

            objects.push_back(o);

            count--;

        }

    }

    cerr << "Writing: " << objects.size() << " objects" << endl;
    Serializer ser;
    string output_filename = input_filename.substr(0,input_filename.find("."))+"_filtered.lmaps";
    ser.setFilePath(output_filename);
    ser.setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
    for(std::list<Serializable*>::iterator it = objects.begin(); it != objects.end(); it++){
        Serializable* s = *it;
        ser.writeObject(*s);
    }

    return 0;
}
