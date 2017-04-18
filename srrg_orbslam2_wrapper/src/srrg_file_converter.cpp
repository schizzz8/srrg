#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/base_sensor_message.h"

using namespace std;
using namespace srrg_core;


const char* banner[] = {
    "srrg_open_file_example_app: example on how to open a txt tio file and read the stuff",
    "",
    "usage: srrg_open_file_example_app <dump_file>",
    0
};

int main(int argc, char ** argv) {
    if (argc < 2 || !strcmp(argv[1], "-h")) {
        printBanner(banner);
        return 0;
    }
    Eigen::Isometry3f global_transform = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f previous_transform = Eigen::Isometry3f::Identity();

    string orb_filename = argv[1];
    string txt_io_filename = argv[2];

    MessageReader reader;
    reader.open(txt_io_filename.c_str());

    MessageWriter writer;
    writer.open(txt_io_filename.substr(0,txt_io_filename.find("."))+"_with_trajectory.txt");
    BaseMessage* msg = 0;

    ifstream orb_file;
    orb_file.open(orb_filename.c_str());

    if(reader.good())
        cerr << "Opened file: " << txt_io_filename << endl;
    else
        cerr << "Error opening file: " << txt_io_filename << endl;

    if(orb_file.is_open())
        cerr << "Opened file: " << orb_filename << endl;
    else
        cerr << "Error opening file: " << orb_filename << endl;

    cerr << endl;


    string line;
    while(getline(orb_file,line)) {
        istringstream iss(line);
        double pose_stamp;
        float x,y,z,qx,qy,qz,qw;
        iss >> pose_stamp
            >> x >> y >> z
            >> qx >> qy >> qz >> qw;

        //cerr << fixed << pose_stamp <<  " ";

        Eigen::Isometry3f current_transform = Eigen::Isometry3f::Identity();
        current_transform.translation() = Eigen::Vector3f (x,y,z);
        current_transform.rotate(Eigen::Quaternion<float> (qw,qx,qy,qz));

        double tolerance = 1.0/27.0;

        bool found = false;
        while(!found && (msg = reader.readMessage())){
            BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(msg);
            if (sensor_msg){
                //cerr << sensor_msg->topic().c_str() << " ";
                //cerr << fixed << sensor_msg->timestamp() << " ";
                if(!strcmp(sensor_msg->topic().c_str(),"/kinect2/sd/image_depth")
                        && fabs(sensor_msg->timestamp()-pose_stamp)<tolerance) {


                    Eigen::Isometry3f delta_camera = previous_transform.inverse() * current_transform;
                    Eigen::Isometry3f delta_robot = sensor_msg->offset() * delta_camera * sensor_msg->offset().inverse();
                    global_transform = global_transform*delta_robot;
                    sensor_msg->setOdometry(global_transform);
                    cerr << ". ";
                    found = true;
                    writer.writeMessage(*msg);
                    //cerr << fixed << sensor_msg->timestamp()-pose_stamp << " " << tolerance << endl;
                }
            }
        }
        previous_transform = current_transform;
    }
    cerr << endl << "done" << endl;
}
