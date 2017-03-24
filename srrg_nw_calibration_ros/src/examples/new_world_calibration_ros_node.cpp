#include "../utils/utils.h"

using namespace new_world_calibration;

/// VARS
int so3_sensor_num = 0;
int so2_sensor_num = 0;

tf::TransformListener* tf_listener;
TfFrameVector so3_tf_frames;
TfFrameVector so2_tf_frames;

Dataset2Vectorf dataset2vector;
Dataset3Vectorf dataset3vector;

TfTransformVector old_so2, curr_so2;
TfTransformVector old_so3, curr_so3;
Eigen::Vector2f curr_ticks;
std::vector<Eigen::Vector2f> old2_ticks, old3_ticks;



bool firstMessage = true;
bool tf_found = false;

void jointCallback(const sensor_msgs::JointStateConstPtr &joint){

    /// LISTEN DATA
    /// JOINT
    float tl = (float)joint->position[0];
    float tr = (float)joint->position[1];
    curr_ticks = Eigen::Vector2f(tl, tr);


    /// TFs
    tf::StampedTransform current_T;
    size_t curr_so2_dataset = 0;
    tf_found = true;
    ///seek for tf(s) of the SO2 sensors
    for(TfFrameVector::iterator so2it = so2_tf_frames.begin(); so2it != so2_tf_frames.end(); ++so2it, ++curr_so2_dataset){
        try{
            tf_listener->lookupTransform((*so2it).frame_id_, (*so2it).child_frame_id_, ros::Time(0), current_T);
            curr_so2.at(curr_so2_dataset) = current_T;
        }
        catch (tf::TransformException &ex){
            tf_found = false;
            ROS_ERROR("%s", ex.what());
            continue;
        }
    }

    ///seek for the tf(s) of the SO3 sensors
    size_t curr_so3_dataset = 0;
    for(TfFrameVector::iterator so3it = so3_tf_frames.begin(); so3it != so3_tf_frames.end(); ++so3it, ++curr_so3_dataset){
        try{
            tf_listener->lookupTransform((*so3it).frame_id_, (*so3it).child_frame_id_, ros::Time(0), current_T);
            curr_so3.at(curr_so3_dataset) = current_T;
        }
        catch (tf::TransformException &ex){
            tf_found = false;
            ROS_ERROR("%s", ex.what());
            continue;
        }
    }


    /// IF FIRST MESSAGE AND TFs HAVE BEEN RECEIVED, SIMPLY BACKUP
    if(firstMessage && tf_found){
        firstMessage = false;
        tf_found = false;
        for(size_t i = 0; i < so2_sensor_num; ++i)
            old2_ticks.push_back(curr_ticks);
        for(size_t i = 0; i < so3_sensor_num; ++i)
            old3_ticks.push_back(curr_ticks);

        old_so2 = curr_so2;
        old_so3 = curr_so3;
    }
    /// OTHERWISE, COMPUTE RELATIVE MOVEs AND STORE IN DATASET
    else if (tf_found)
    {
        tf_found = false;
        /// RELATIVE MOVEs OF SO2 SENSORS
        for(size_t i = 0; i < so2_sensor_num; ++i)
        {
            Eigen::Vector2f relative2_ticks = curr_ticks - old2_ticks.at(i);
            normalizeTheta(relative2_ticks);

            Sample2f* sample2f = new Sample2f();
            sample2f->setTicks(relative2_ticks);

            tf::Transform relative_T = old_so2.at(i).inverse() * curr_so2.at(i);
            Eigen::Isometry3f iso3f;
            transformTFToEigenImpl(relative_T ,iso3f);
            Eigen::Isometry2f iso2f = iso2(iso3f);

            sample2f->setIsometry(iso2f);
            std::cerr<<"x";
            dataset2vector.at(i)->push_back(sample2f);
            old2_ticks.at(i) = curr_ticks;
            old_so2.at(i) = curr_so2.at(i);

        }

        /// RELATIVE MOVEs OF SO3 SENSORS
        for(size_t i = 0; i < so3_sensor_num; ++i)
        {
            Eigen::Vector2f relative3_ticks = curr_ticks - old3_ticks.at(i);
            normalizeTheta(relative3_ticks);

            Sample3f* sample3f = new Sample3f();
            sample3f->setTicks(relative3_ticks);

            tf::Transform relative_T = old_so3.at(i).inverse() * curr_so3.at(i);
            Eigen::Isometry3f iso3f;
            transformTFToEigenImpl(relative_T ,iso3f);

            sample3f->setIsometry(iso3f);
            std::cerr<<"o";
            dataset3vector.at(i)->push_back(sample3f);
            old3_ticks.at(i) = curr_ticks;
            old_so3.at(i) = curr_so3.at(i);

        }
    } //end else if (tf_found)

} //end of callback


void printBanner(const char** banner) {
    const char** b = banner;
    while(*b) {
        std::cerr << *b << std::endl;
        b++;
    }
}

const char* banner[]={
        "\n\nUsage:  new_world_calibration_node -joint-topic <joint_topic> -guess <init_guess.txt> [Options]\n",
        "Example:  new_world_calibration_node -joint-topic /joint_state -guess init_guess.txt\n\n",
        "Options:\n",
        "------------------------------------------\n",
        "-joint-topic <string>       topic name of joints of the platform",
        "-guess <string>             file containing the initial guess of the sensors. see initial_guess/ folder",
        "-iterations <int>           iterations of the LS, default 10",
        "-block <int>                dimension of the block of data collected for a solver call, default 50",
        "-rate <float>               log rate expressed in Hz, default 2",
        "-info                       flag to store calibration info on file",
        "-h                          this help\n",
        0
};


int main(int argc, char ** argv){

    ros::init(argc, argv, "new_world_calibration_ros_node");
    ros::NodeHandle nh("~");
    ROS_INFO("new_world_calibration_node started...");

    if(argc < 2){
        printBanner(banner);
        return 1;
    }

    std::string joint_topic = "/joint_state";
    std::string init_guess_file;
    int iterations = 10;
    int block = 50;
    double rate = 2;
    bool store_calibration_info = false;

    int c=1;

    while(c<argc){
        if (! strcmp(argv[c],"-h")){
            printBanner(banner);
            return 1;
        }
        else if(! strcmp(argv[c],"-joint-topic")){
            c++;
            joint_topic = argv[c];
        }
        else if(! strcmp(argv[c],"-guess")){
            c++;
            init_guess_file = argv[c];
        }
        else if(! strcmp(argv[c],"-iterations")){
            c++;
            iterations = std::atoi(argv[c]);
        }
        else if(! strcmp(argv[c],"-block")){
            c++;
            block = std::atoi(argv[c]);
        }
        else if(! strcmp(argv[c],"-rate")){
            c++;
            rate = std::atof(argv[c]);
        }
        else if(! strcmp(argv[c],"-info")){
            store_calibration_info = true;
        }
        c++;
    }

    if(init_guess_file.empty())
    {
        std::cerr<<"\n***[error]: no valid path to a valid init_guess.txt file. An example file can be found in srrg_nw_calibration_ros package, in the init_guess folder***\n"<<std::endl;
        std::exit(-1);
    }

    std::cerr<<"guess:       "<<init_guess_file<<std::endl;
    std::cerr<<"joint-topic: "<<joint_topic<<std::endl;
    std::cerr<<"iterations:  "<<iterations<<std::endl;
    std::cerr<<"block:       "<<block<<std::endl;
    std::cerr<<"rate:        "<<rate<<std::endl;

    Eigen::VectorXf guess_odom;
    SensorParams2Vector<float> sensor2d_params;
    SensorParams3Vector<float> sensor3d_params;
    readInitialGuess(init_guess_file, guess_odom, sensor2d_params, so2_tf_frames, sensor3d_params, so3_tf_frames);

    so2_sensor_num = so2_tf_frames.size();
    curr_so2.resize(so2_sensor_num);
    so3_sensor_num = so3_tf_frames.size();
    curr_so3.resize(so3_sensor_num);

    //init dataset vectors and samples
    for(size_t i=0; i<so2_sensor_num; ++i){
        Dataset2f* data2 = new Dataset2f;
        dataset2vector.push_back(data2);
    }
    for(size_t i=0; i<so3_sensor_num; ++i){
        Dataset3f* data3 = new Dataset3f;
        dataset3vector.push_back(data3);
    }

    ros::Subscriber joint_sub = nh.subscribe(joint_topic, 1, jointCallback);
    tf_listener = new tf::TransformListener;

    /// INIT THE ROBOT
    DifferentialDrive<float>* robot = new DifferentialDrive<float>();

    MultiSolver<float>* multisolver = new MultiSolver<float>(so2_sensor_num, so3_sensor_num);
    multisolver->setKinematics(robot);
    multisolver->setEpsilon(1e-7);
    multisolver->setIterations(iterations);

    OdomParams<float>* odom_params = new OdomParams<float>(guess_odom, std::string("odom_params") , std::string("/base_link"));

    multisolver->setSensorParams2(sensor2d_params);
    multisolver->setSensorParams3(sensor3d_params);
    multisolver->setOdomParams(odom_params);

    //add a weight of 100 on existing 2d sensors (more reliables in terms of precision)
    std::vector<int> information_weights_2d_sensor(so2_sensor_num, 100);
    multisolver->setInformationWeights2(information_weights_2d_sensor);

    multisolver->init();

    std::cerr<<"solver is ready"<<std::endl;

    ///**** Store calibration information
    ///****
    CalibInfoVector calibration_info_vector;
    ros::Time starting_time = ros::Time::now();
    ros::Time current_time;

    bool calibrate = true;
    ros::Rate r(rate);
    int id = 1;

    /// this kind of ros::ok management allows to intercept the ctrl+C from user, doing things,
    /// i.e. print on file the calibration info.
    for(; ros::ok() ;){
//        while(ros::ok()){

        for(Dataset2Vectorf::iterator so2it = dataset2vector.begin(); so2it != dataset2vector.end(); ++so2it){
            if((*so2it)->size() < id*block)
                calibrate = false;
            else
                calibrate = true;
        }
        for(Dataset3Vectorf::iterator so3it = dataset3vector.begin(); so3it != dataset3vector.end(); ++so3it){
            if((*so3it)->size() < id*block)
                calibrate = false;
            else
                calibrate = true;
        }

        if(calibrate){
            std::cerr<<std::endl;
            multisolver->solve(dataset2vector, dataset3vector);
            std::cerr<<std::endl;
            std::cerr<<"data block: "<<id*block<<std::endl;

            current_time = ros::Time::now();
            double c_time = (current_time-starting_time).toSec();
            CalibInfo calib_status(c_time, multisolver->odomParams(), multisolver->sensorParams2(), multisolver->sensorParams3(), multisolver->H());
            calibration_info_vector.push_back(calib_status);


            calibrate = true;
            id++;
        }
        r.sleep();
        ros::spinOnce(); //queue size of jointCallback is 1
    }
    if(ros::ok()){
        ros::shutdown();
    }
    /// The following else intercepts the ctrl+C from terminal
    else{
        if(store_calibration_info) {
            std::ostringstream oss;
            time_t theTime = time(NULL);
            struct tm *aTime = localtime(&theTime);
            int day = aTime->tm_mday;
            int month = aTime->tm_mon + 1; // Month is 0 - 11, add 1 to get a jan-dec 1-12 concept
            int year = aTime->tm_year + 1900;
            int hour = aTime->tm_hour;
            int min = aTime->tm_min;
            int sec = aTime->tm_sec;

            oss << year << month << day << hour << min << sec << "_calibInfo.txt";
            std::string str = oss.str();
            if(calibration_info_vector.size() > 0)
                writeCalibrationInfo(calibration_info_vector, str);
        }
    }

    return 0;

}