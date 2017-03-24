#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/pinhole_image_message.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "three_d_grid.h"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

using namespace std;
using namespace srrg_core;

// Help objects to force linking
PinholeImageMessage i;

string filename = "";
int prec = 5;
string method = "";

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void parseCommandLine(int argc, char *argv[])
{
    std::cout << "\n";
    std::cout << BOLD(FRED("Reconstruct implicit surface from a 3D data set.\n"));
    std::cout << "\n";



    if(argc > 1)
    {
        filename = argv[1];

        for (int i = 2; i < argc; i++) {
            if(strcmp(argv[i],"-p") == 0)
                prec = atoi(argv[i+1]);
            if(strcmp(argv[i],"-method") == 0)
                method = argv[i+1];
        }
    }
    int c = 1;
    while (c<argc){
        if(argc < 2 || !strcmp(argv[1], "-h")){
            std::cout << FBLU("Syntax is: signed_distance_function <options> filename\n");
            std::cout << "  where options are:\n";
            std::cout << "\t-p n" << "\t= grid precision (default: " << FCYN("10") << ")\n";
            std::cout << "\t-method X" << "\t= grid type: sparse/dense (default: " << FCYN("sparse") << ")\n";
            exit(0);
        }
        if (! strcmp(argv[c], "-p")){
            c++;
            prec = atoi(argv[c]);
        } else if (! strcmp(argv[c], "-method")){
            c++;
            method = argv[c];
        } else {
            filename = argv[c];
        }
        c++;
    }

}


int main(int argc, char ** argv) {
    parseCommandLine(argc,argv);

    MessageReader reader;
    reader.open(filename.c_str());

    cv::Mat image;
    Eigen::Matrix3f invK;
    float depth_scale;
    PointCloud::Ptr cloud (new PointCloud);
    float min_distance = 0.01;
    float max_distance = 10.0;
    int count = 1;

    BaseMessage* msg = 0;
    while ((msg = reader.readMessage())) {
        msg->untaint();

        PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
        if (pinhole_image_msg &&
                !strcmp(pinhole_image_msg->topic().c_str(),"/camera/depth/image_raw") &&
                count>0) {
            image = pinhole_image_msg->image();
            invK = pinhole_image_msg->cameraMatrix().inverse();
            depth_scale = pinhole_image_msg->depthScale();

            cv::Mat filtered_image;
            cv::GaussianBlur(image,filtered_image, cv::Size(1,1), 0, 0);

//            filtered_image = image;

            for(int r=0; r<filtered_image.rows;r++) {
                const unsigned short* id_ptr  = filtered_image.ptr<unsigned short>(r);
                for(int c=0; c<filtered_image.cols;c++){
                    unsigned short id = *id_ptr;
                    float d = id * depth_scale;
                    Eigen::Vector3f p = invK * Eigen::Vector3f(c*d,r*d,d);
                    if (id>0 && d<max_distance && d>min_distance)
                        cloud->push_back(PointType(p.x(),p.y(),p.z()));
                    id_ptr++;
                }
            }

            AdaptiveGrid grid(cloud,prec);
            grid.computeDistanceMap();
            grid.writeDataToFile();

            count--;

        }
    }
    cerr << "done" << endl;
}
