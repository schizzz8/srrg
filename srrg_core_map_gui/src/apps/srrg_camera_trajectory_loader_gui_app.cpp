#include <fstream>
#include <stdexcept>

#include <qapplication.h>

#include <srrg_boss/deserializer.h>
#include <srrg_boss/trusted_loaders.h>
#include <srrg_core_map/cloud.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_txt_io/message_reader.h>
#include <srrg_txt_io/pinhole_image_message.h>
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

    MessageReader reader;
    reader.open(argv[1]);

    BaseMessage* msg = 0;

    BaseCameraInfo* camera_info = 0;
    bool first=true;

    MapNode* new_node=0;
    MapNode* previous_node=0;
    BinaryNodeRelation* rel=0;

    MapNodeList nodes;
    BinaryNodeRelationSet relations;

    while ((msg = reader.readMessage())) {
        msg->untaint();
        PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
        if (pinhole_image_msg) {
            if(first){
                camera_info = new BaseCameraInfo(pinhole_image_msg->topic(),
                                                                 pinhole_image_msg->frameId(),
                                                                 pinhole_image_msg->offset(),
                                                                 pinhole_image_msg->depthScale());
                previous_node = new ImageMapNode(pinhole_image_msg->odometry(),
                                                     camera_info,
                                                     pinhole_image_msg->topic(),
                                                     pinhole_image_msg->seq());
                nodes.addElement(previous_node);
                first=false;
                continue;
            }
            new_node = new ImageMapNode(pinhole_image_msg->odometry(),
                                                 camera_info,
                                                 pinhole_image_msg->topic(),
                                                 pinhole_image_msg->seq());
            nodes.addElement(new_node);

            rel = new BinaryNodeRelation;
            rel->setFrom(previous_node);
            rel->setTo(new_node);
            rel->setTransform(previous_node->transform().inverse()*new_node->transform());
            relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
            previous_node=new_node;
        }
    }

    QApplication app(argc, argv);
    TrajectoryViewer viewer;
    viewer.nodes = nodes;
    viewer.relations = relations;
    viewer.show();
    app.exec();

    return 0;
}
