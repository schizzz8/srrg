#include <fstream>
#include <srrg_types/defs.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_core_map/cloud.h>
#include <srrg_core_map/image_map_node.h>
#include "srrg_core_map/local_map_with_traversability.h"
#include "srrg_local_maps_condenser_viewers/condenser_viewer.h"
#include "qapplication.h"
#include <stdexcept>
#include <srrg_boss/deserializer.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/trusted_loaders.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "srrg_local_maps_condenser/connectivity_refiner.h"
#include "srrg_local_maps_condenser/local_maps_clusterizer.h"

using namespace std;
using namespace srrg_local_maps_condenser;
using namespace srrg_local_maps_condenser_gui;
using namespace srrg_boss;
using namespace srrg_core_map;


const char* banner[]= {
    "fps_lmap_viewer_app: example on how to show/load a set of boss objects constituting a boss map",
    "usage:",
    " fps_lmap_viewer_app  <boss filename>",
    0
};

void printBanner() {
  const char** b = banner;
  while(*b) {
    cout << *b << endl;
    b++;
  }
}

int main (int argc, char** argv) {
    if (argc<2 || ! strcmp(argv[1],"-h")) {
        printBanner();
        return 0 ;
    }
    int depth = 3;
    float resolution = 0.05;
    float voxel_size = 0.02;
    float distance_threshold = 20;
    int connected_points = 1000;
    std::string filename = "";
    std::string output_filename="";

    int c = 1;
    while(c<argc){
        if(! strcmp(argv[c],"-depth")){
            c++;
            depth = atoi(argv[c]);
        }else if(! strcmp(argv[c],"-res")){
            c++;
            resolution = atof(argv[c]);
        }else if(! strcmp(argv[c],"-vsize")){
            c++;
            voxel_size = atof(argv[c]);
        }else if(! strcmp(argv[c],"-dth")){
            c++;
            distance_threshold = atof(argv[c]);
        }else if(! strcmp(argv[c],"-cp")){
            c++;
            connected_points = atoi(argv[c]);
        }else if (! strcmp(argv[c], "-o")){
            c++;
            output_filename = argv[c];
        }else{
            filename = argv[c];
            break;
        }
        c++;
    }

    std::list<Serializable*> objects;
    Deserializer des;
    des.setFilePath(filename);
    Serializable* o;
    MapNodeList* lmaps = new MapNodeList;
    while ( (o = des.readObject()) ){
        LocalMap* lmap = dynamic_cast<LocalMap*>(o);
        if (lmap)
            lmaps->addElement(lmap);
        else
            objects.push_back(o);
    }

    cerr << "Running clustering algorithm for " << lmaps->size() << " maps" << endl;
    cerr << "Depth: " << depth << endl;
    cerr << "Traversability Map resolution: " << resolution << endl;
    cerr << "Voxel size: " << voxel_size << endl;

    LocalMapsClusterizer clusterizer (depth,resolution,1,voxel_size);
    clusterizer.computeBoundingBox(lmaps);
    clusterizer.buildQuadtree();
    //clusterizer.visualizeQuadtree();
    clusterizer.mergeLocalMaps();

    MapNodeList* clusters = clusterizer.clusters();
    for(MapNodeList::iterator it = clusters->begin(); it != clusters->end(); it++){
        LocalMapWithTraversability* lmap = dynamic_cast<LocalMapWithTraversability*> (it->get());
        if(lmap)
            objects.push_back(lmap);
    }
    cerr << "--------------------------------------------------------------------------------" << endl;

    cerr << "Executing connectivity refinement for " << clusters->size() << " local maps" << endl;
    cerr << "Distance threshold: " << distance_threshold << "m" << endl;
    cerr << "Minimum number of connected points: " << connected_points << endl;
    ConnectivityRefiner refiner(distance_threshold,connected_points,resolution);
    refiner.setInput(clusters);
    refiner.execute();

    BinaryNodeRelationSet* edges = refiner.edges();
    for(BinaryNodeRelationSet::iterator it = edges->begin(); it != edges->end(); it++) {
        BinaryNodeRelation* edge = dynamic_cast<BinaryNodeRelation*> (it->get());
        if(edge)
            objects.push_back(edge);
    }
    cerr << "--------------------------------------------------------------------------------" << endl;

    cerr << "Writing: " << objects.size() << " objects" << endl;
    Serializer ser;
    ser.setFilePath(output_filename);
    ser.setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
    for(std::list<Serializable*>::iterator it = objects.begin(); it != objects.end(); it++){
        Serializable* s = *it;
        ser.writeObject(*s);
    }

    MapNodeList nodes;
    BinaryNodeRelationSet relations;
    for(std::list<Serializable*>::iterator it = objects.begin(); it != objects.end(); it++){
        Serializable* o = *it;
        LocalMapWithTraversability* lmap = dynamic_cast<LocalMapWithTraversability*>(o);
        if(lmap)
            nodes.addElement(lmap);

        BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(o);
        if(rel){
            LocalMapWithTraversability* from = dynamic_cast<LocalMapWithTraversability*>(rel->from());
            LocalMapWithTraversability* to = dynamic_cast<LocalMapWithTraversability*>(rel->to());
            if(from && to)
                relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
        }
    }
    QApplication app(argc, argv);
    CondenserViewer viewer;
    viewer.nodes = nodes;
    viewer.relations = relations;
    viewer.show();
    app.exec();
    return 1;
}
