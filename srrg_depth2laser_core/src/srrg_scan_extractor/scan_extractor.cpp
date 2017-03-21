#include "scan_extractor.h"

namespace srrg_depth2laser_core{
using namespace std;
using namespace cv;
using namespace srrg_core;

ScanExtractor::ScanExtractor(string filename){
    cerr << "Reading configuration file!" << endl << endl;
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    
    _topic_name = (string) fs["TopicName"];
    cerr << "Laser topic name: " << _topic_name << endl;
    
    _frame_id = (string) fs["FrameId"];
    cerr << "Laser frame ID: " << _frame_id << endl;

    _num_ranges = (int) fs["NumRanges"];
    cerr << "Number of ranges: " << _num_ranges << endl;

    _min_range = (float) fs["MinRange"];
    cerr << "Min Range: " << _min_range << endl;

    _max_range = (float) fs["MaxRange"];
    cerr << "Max Range: " << _max_range << endl;
    
    _min_angle = (float) fs["MinAngle"];
    cerr << "Min Angle: " << _min_angle << endl;

    _max_angle = (float) fs["MaxAngle"];
    cerr << "Max Angle: " << _max_angle << endl;

    _angle_increment = (_max_angle - _min_angle)/_num_ranges;
    _inverse_angle_increment = 1./_angle_increment;

    _laser_plane_thickness = (float) fs["LaserPlaneThickness"];
    cerr << "Laser Plane Thickness: " << _laser_plane_thickness << endl;

    _squared_max_norm = _max_range*_max_range;

    _squared_min_norm = _min_range*_min_range;

    //    float fx = (float) fs["CameraMatrix.fx"];
    //    float fy = (float) fs["CameraMatrix.fy"];
    //    float tx = (float) fs["CameraMatrix.tx"];
    //    float ty = (float) fs["CameraMatrix.ty"];

    //    _K = Eigen::Matrix3f::Zero();
    //    _K(0,0) = fx;
    //    _K(1,1) = fy;
    //    _K(0,2) = tx;
    //    _K(1,2) = ty;
    //    _K(2,2) = 1;

    //    _invK = _K.inverse();
    //    cerr << "Camera Matrix:" << endl << _K << endl;

    //    float cx = (float) fs["CameraTransform.x"];
    //    float cy = (float) fs["CameraTransform.y"];
    //    float cz = (float) fs["CameraTransform.z"];
    //    float cqx = (float) fs["CameraTransform.qx"];
    //    float cqy = (float) fs["CameraTransform.qy"];
    //    float cqz = (float) fs["CameraTransform.qz"];
    //    float cqw = (float) fs["CameraTransform.qw"];

    //    _camera_transform = Eigen::Isometry3f::Identity();
    //    _camera_transform.translation() = Eigen::Vector3f(cx,cy,cz);
    //    Eigen::Quaternion<float> camera_rotation(cqw,cqx,cqy,cqz);
    //    _camera_transform.rotate(camera_rotation);

    //    cerr << "Camera Transform: " << _camera_transform.translation().transpose() << " - ";
    //    cerr << camera_rotation.x() << " " << camera_rotation.y() << " " << camera_rotation.z() << " " << camera_rotation.w() << endl;

    //    float lx = (float) fs["LaserTransform.x"];
    //    float ly = (float) fs["LaserTransform.y"];
    //    float lz = (float) fs["LaserTransform.z"];
    //    float lqx = (float) fs["LaserTransform.qx"];
    //    float lqy = (float) fs["LaserTransform.qy"];
    //    float lqz = (float) fs["LaserTransform.qz"];
    //    float lqw = (float) fs["LaserTransform.qw"];

    //    _laser_transform = Eigen::Isometry3f::Identity();
    //    _laser_transform.translation() = Eigen::Vector3f(lx,ly,lz);
    //    Eigen::Quaternion<float> laser_rotation(lqw,lqx,lqy,lqz);
    //    _laser_transform.rotate(laser_rotation);

    //    cerr << "Laser Transform: " << _laser_transform.translation().transpose() << " - ";
    //    cerr << laser_rotation.x() << " " << laser_rotation.y() << " " << laser_rotation.z() << " " << laser_rotation.w() << endl;

    //    camera2laser_transform=_laser_transform.inverse()*_camera_transform;

}

void ScanExtractor::setParameters(int seq,double timestamp,srrg_core::LaserMessage& laser_msg){
    laser_msg.setTopic(_topic_name);
    laser_msg.setFrameId(_frame_id);
    laser_msg.setSeq(seq);
    laser_msg.setTimestamp(timestamp);
    laser_msg.setMinRange(_min_range);
    laser_msg.setMaxRange(_max_range);
    laser_msg.setMinAngle(_min_angle);
    laser_msg.setMaxAngle(_max_angle);
    laser_msg.setAngleIncrement(_angle_increment);
    laser_msg.setOffset(_laser_transform);
}

void ScanExtractor::compute(const cv::Mat& image,srrg_core::LaserMessage& laser_msg){
    vector<float> ranges(_num_ranges,_max_range+0.1);
    for(int i =0;i<image.rows;i++){
        const ushort* row_ptr = image.ptr<ushort>(i);
        for(int j=0;j<image.cols;j++){
            ushort id=row_ptr[j];
            if(id!=0){
                float d=1e-3*id;
                Eigen::Vector3f image_point(j*d,i*d,d);
                Eigen::Vector3f camera_point=_invK*image_point;
                Eigen::Vector3f laser_point=camera2laser_transform*camera_point;
                if (fabs(laser_point.z())<_laser_plane_thickness){
                    float theta=atan2(laser_point.y(),laser_point.x());
                    float range=laser_point.head<2>().squaredNorm();
                    if (range<_squared_min_norm)
                        continue;
                    if (range>_squared_max_norm)
                        continue;
                    range=sqrt(range);
                    int bin=(int)((theta-_min_angle)*_inverse_angle_increment);
                    if (bin<0||bin>=_num_ranges)
                        continue;
                    if(ranges[bin]>range)
                        ranges[bin]=range;
                }
            }
        }
    }
    laser_msg.setRanges(ranges);
}

}
