#include <fstream>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_depth2laser_viewers/depth2laser_viewer.h"
#include <QApplication>

using namespace std;
using namespace srrg_core;
using namespace srrg_depth2laser_gui;

// Help objects to force linking
LaserMessage l;

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

    MessageReader reader;
    reader.open(argv[1]);

    vector<LaserMessage*> scans;

    BaseMessage* msg = 0;
    while ((msg = reader.readMessage())) {
        msg->untaint();
        LaserMessage* sensor_msg = dynamic_cast<LaserMessage*>(msg);
        if (sensor_msg) {
            cerr << sensor_msg->tag() << endl;
            scans.push_back(sensor_msg);
        }
    }

    cerr << "Read " << scans.size() << " laser scans!" << endl;

//    QApplication app(argc,argv);
//    Depth2LaserViewer viewer;
//    viewer.scans = scans;
//    viewer.show();
//    app.exec();

    cerr << "done" << endl;
}
