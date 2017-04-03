#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_txt_io/sensor_message_sorter.h"
#include "srrg_depth2laser/depth2laser.h"
#include "srrg_depth2laser_viewers/depth2laser_viewer.h"
#include <QApplication>

using namespace std;
using namespace srrg_core;
using namespace srrg_depth2laser_gui;
using namespace srrg_depth2laser;

// Help objects to force linking
PinholeImageMessage i;
LaserMessage l;

QApplication* app;
Depth2Laser* depth2laser = 0;

const char* banner[] = {
    "srrg_depth2laser_app: example on how to convert depth images into laser scans",
    "",
    "usage: srrg_depth2laser_app [options] <dump_file>",
    0
};

int main(int argc, char ** argv) {
    if (argc < 2 || !strcmp(argv[1], "-h")) {
        printBanner(banner);
        return 0;
    }

    string filename = argv[1];
    depth2laser = new Depth2Laser (filename);

    string input_filename = argv[2];
    MessageReader reader;
    reader.open(input_filename.c_str());

    BaseMessage* msg = 0;
    MessageWriter writer;
    writer.open(input_filename.substr(0,input_filename.find("."))+"_with_laser.txt");

    app = new QApplication(argc,argv);
    Depth2LaserViewer* viewer = new Depth2LaserViewer(depth2laser);


    bool got_info = false;

    while ((msg = reader.readMessage())) {
        msg->untaint();
        PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
        if (pinhole_image_msg){
            if(!got_info){
                depth2laser->setK(pinhole_image_msg->cameraMatrix());
                cerr << "Camera Matrix:" << endl << depth2laser->K() << endl;
                depth2laser->setCameraTransform(pinhole_image_msg->offset());
                Eigen::Vector3f camera_translation = depth2laser->cameraTransform().translation();
                Eigen::Quaternion<float> camera_rotation(depth2laser->cameraTransform().rotation());
                cerr << "Camera Transform: " << camera_translation.transpose() << " - ";
                cerr << camera_rotation.x() << " " << camera_rotation.y() << " " << camera_rotation.z() << " " << camera_rotation.w() << endl;
                depth2laser->setLaserTransform();
                Eigen::Vector3f laser_translation = depth2laser->laserTransform().translation();
                Eigen::Quaternion<float> laser_rotation(depth2laser->laserTransform().rotation());
                cerr << "Laser Transform: " << laser_translation.transpose() << " - ";
                cerr << laser_rotation.x() << " " << laser_rotation.y() << " " << laser_rotation.z() << " " << laser_rotation.w() << endl;
                got_info = true;
            }
            if(got_info && !strcmp(pinhole_image_msg->topic().c_str(),"/camera/depth/image_raw")) {
                LaserMessage* laser_msg = new LaserMessage;
                depth2laser->setParameters(pinhole_image_msg->seq(),pinhole_image_msg->timestamp(),*laser_msg);
                depth2laser->compute(pinhole_image_msg->image(),*laser_msg);
                laser_msg->setOdometry(pinhole_image_msg->odometry());
                writer.writeMessage(*laser_msg);
                viewer->scans.push_back(laser_msg);
                //viewer->_scan = laser_msg;


            }

        }
    }
    viewer->show();
    viewer->updateGL();
    app->processEvents();

    cerr << "done" << endl;
    //app->exec();

}
