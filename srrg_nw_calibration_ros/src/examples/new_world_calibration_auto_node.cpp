#include "../utils/utils.h"
#include "../motions/motions.hpp"

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
bool tf_arrived = false; //used to start moving the platform
ros::Time robot_motions_init_time;

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
        if(!tf_arrived) {
            tf_arrived = true;
            robot_motions_init_time = ros::Time::now();
        }
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
        "\n\nUsage:  new_world_calibration_ros_auto_node -twist-topic <twist_topic> -joint-topic <joint_topic> -guess <init_guess.txt> [Options]\n",
        "Example:  new_world_calibration_ros_auto_node -twist-topic /cmd_vel -joint-topic /joint_state -guess init_guess.txt\n\n",
        "Options:\n",
        "------------------------------------------\n",
        "-twist-topic <string>           topic name of twist to be applied to move the platform, default /cmd_vel",
        "-joint-topic <string>           topic name of joints of the platform, default /joint_state",
        "-motion-time <double>           time of execution of each motion in seconds, default 5",
        "-iterations <int>               iterations of the LS, default 10",
        "-rate <double>                  log rate expressed in Hz, default 2",
        "-motion_multiplier <double>     multiplied by 0.02 to define twist values, default 5",
        "-epsilon <double>               termination criterion part1: chi^2(t) - chi^2(t-1) < epsilon, default 1e-9",
        "-lambda <double>                termination criterion part2: eigenvalue_min/eigenvalue_max > lambda, default 1e-3",
        "-info                           flag to store calibration info on file",
        "-h                              this help\n",
        0
};


int main(int argc, char ** argv){

    ros::init(argc, argv, "new_world_calibration_ros_auto_node");
    ros::NodeHandle nh("~");
    ROS_INFO("new_world_calibration_auto_node started...");

    if(argc < 2){
        printBanner(banner);
        return 1;
    }

    std::string joint_topic = "/joint_state";
    std::string twist_topic = "/cmd_vel";
    std::string init_guess_file;
    int iterations = 10;
    double rate = 2;
    double motion_time = 5;
    double epsilon_threshold = 1e-9;
    double lambda_threshold = 1e-3;
    float motion_multiplier = 5;
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
        else if(! strcmp(argv[c],"-twist-topic")){
            c++;
            twist_topic = argv[c];
        }
        else if(! strcmp(argv[c],"-guess")){
            c++;
            init_guess_file = argv[c];
        }
        else if(! strcmp(argv[c],"-iterations")){
            c++;
            iterations = std::atoi(argv[c]);
        }
        else if(! strcmp(argv[c],"-motion-time")){
            c++;
            motion_time = std::atof(argv[c]);
        }
        else if(! strcmp(argv[c],"-rate")){
            c++;
            rate = std::atof(argv[c]);
        }
        else if(! strcmp(argv[c],"-motion-multiplier")){
            c++;
            motion_multiplier = std::atof(argv[c]);
        }
        else if(! strcmp(argv[c],"-epsilon")){
            c++;
            epsilon_threshold = std::atof(argv[c]);
        }
        else if(! strcmp(argv[c],"-lambda")){
            c++;
            lambda_threshold = std::atof(argv[c]);
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

    std::cerr<<"guess:          "<<init_guess_file<<std::endl;
    std::cerr<<"joint-topic:    "<<joint_topic<<std::endl;
    std::cerr<<"twist-topic:    "<<twist_topic<<std::endl;
    std::cerr<<"iterations:     "<<iterations<<std::endl;
    std::cerr<<"motion-time:    "<<motion_time<<std::endl;
    std::cerr<<"rate:           "<<rate<<std::endl;
    std::cerr<<"epsilon:        "<<epsilon_threshold<<std::endl;
    std::cerr<<"lambda:         "<<lambda_threshold<<std::endl;

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
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(twist_topic, 1000);


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


    /*******************
     *  MOTIONS LIBRARY
     ******************/
        /// MOTION POPULATION
    Motions motions;
//    float lin_distance_to_cover = 1.f; //meter
//    float ang_distance_to_cover = 1.4f; //rad
        /// FORWARD/BACKWARD
    geometry_msgs::Twist forward_twist;
    forward_twist.linear.x = 0.02*motion_multiplier;
    geometry_msgs::Twist backward_twist;
    backward_twist.linear.x = -0.02*motion_multiplier;
    Motion forward(forward_twist, backward_twist, "forward/backward"/*, lin_distance_to_cover, 999.0f*/);
        /// ARC_1
    geometry_msgs::Twist forward_arc_1_twist;
    forward_arc_1_twist.linear.x = 0.02*motion_multiplier;
    forward_arc_1_twist.angular.z = 0.02*motion_multiplier;
    geometry_msgs::Twist backward_arc_1_twist;
    backward_arc_1_twist.linear.x = -0.02*motion_multiplier;
    backward_arc_1_twist.angular.z = -0.02*motion_multiplier;
    Motion arc_1(forward_arc_1_twist, backward_arc_1_twist, "arc_1"/*, lin_distance_to_cover*2/3, 999.0f*/);
        /// ARC_2
    geometry_msgs::Twist forward_arc_2_twist;
    forward_arc_2_twist.linear.x = 0.02*motion_multiplier;
    forward_arc_2_twist.angular.z = -0.02*motion_multiplier;
    geometry_msgs::Twist backward_arc_2_twist;
    backward_arc_2_twist.linear.x = -0.02*motion_multiplier;
    backward_arc_2_twist.angular.z = 0.02*motion_multiplier;
    Motion arc_2(forward_arc_2_twist, backward_arc_2_twist, "arc_2"/*, lin_distance_to_cover*2/3, 999.0f*/);
        /// ROTATE_IN_PLACE
    geometry_msgs::Twist rotate_clockwise_twist;
    rotate_clockwise_twist.angular.z = 0.05*motion_multiplier;
    geometry_msgs::Twist rotate_counterclockwise_twist;
    rotate_counterclockwise_twist.angular.z = -0.05*motion_multiplier;
    Motion rotate_in_place(rotate_clockwise_twist, rotate_counterclockwise_twist, "rotate_in_place"/*, 999.0f, ang_distance_to_cover*/);

    motions.push_back(forward);
    motions.push_back(arc_1);
    motions.push_back(arc_2);
    motions.push_back(rotate_in_place);
    Motions::iterator current_motion = motions.begin();

    bool exploration = false, exploitation = false; //start exploring
    bool go = true; //start with forward motion (go = false means backward)
    bool calibrate = true;
    float lin_covered_distance = 0;
    float ang_covered_distance = 0;
    tf_arrived = false;
    double time_elapsed = 0.f;

    /// TERMINATION CRITERIA VARIABLES
    std::vector<float> chi2_vector;

    ///**** Store calibration information
    ///****
    CalibInfoVector calibration_info_vector;
    ros::Time starting_time = ros::Time::now();
    ros::Time current_time;


    ros::Rate r(rate);
    int id = 1;
    while(ros::ok()){

        if(exploration){
            time_elapsed = (ros::Time::now() - robot_motions_init_time).toSec();
            if(time_elapsed < motion_time){
                //continue current motion
                twist_pub.publish(go? current_motion->_twist_go : current_motion->_twist_back_home);
            }else{
                //this sleep avoids slippage in motions switch
                r.sleep();
                //change motion
                printf("change motion\n");
                if(go){
                    //if platform was moving forward, change direction
                    go = false;
                    printf("backward\n");
                }
                else{
                    //otherwise go to next motion
                    go = true;
                    printf("calibrating\n");
                    multisolver->solve(dataset2vector,dataset3vector);
                    printf("storing H matrix\n");
                    current_motion->setH(multisolver->H());
                    current_time = ros::Time::now();
                    double c_time = (current_time-starting_time).toSec();
                    CalibInfo calib_status(c_time, multisolver->odomParams(), multisolver->sensorParams2(), multisolver->sensorParams3(), multisolver->H());
                    calibration_info_vector.push_back(calib_status);

                    printf("reset multi solver\n");
                    cleanDataset(dataset2vector);
                    cleanDataset(dataset3vector);
                    //this is bad, I have to fix the architecture. damn!
                    multisolver->setSensorParams2(sensor2d_params);
                    multisolver->setSensorParams3(sensor3d_params);
                    multisolver->setOdomParams(odom_params);
                    multisolver->init();
                    printf("init done\n");
                    fflush(stdout);
                    if(current_motion == --(motions.end())){
                        exploration = false;
                        exploitation = true; //redundant, but at least legible
                        current_motion = findBestMotion(motions);
                        printf("EXPLOITING FROM NOW\n");
                    }
                    else{
                        current_motion++;
                    }
                }
                robot_motions_init_time = ros::Time::now();
            }
        }
        else if(exploitation) {
            time_elapsed = (ros::Time::now() - robot_motions_init_time).toSec();
            if (time_elapsed < motion_time) {
                //continue current motion
                twist_pub.publish(go ? current_motion->_twist_go : current_motion->_twist_back_home);
            } else {
//                printf("change motion\n");
                r.sleep();
                //change motion
                if (go) {
                    go = false;
//                    printf("\nbackward\n");
                } else {
                    printf("\ncalibrating\n");
                    multisolver->solve(dataset2vector, dataset3vector);
                    current_time = ros::Time::now();
                    double c_time = (current_time-starting_time).toSec();
                    CalibInfo calib_status(c_time, multisolver->odomParams(), multisolver->sensorParams2(), multisolver->sensorParams3(), multisolver->H());
                    calibration_info_vector.push_back(calib_status);

                    go = true;
                    current_motion = findBestMotion(motions, multisolver->H());

                    chi2_vector.push_back(multisolver->chi2());
                    if(chi2_vector.size() > 1)
                    {
                        ///check termination criterion
                        float chi2_evol =chi2_vector[0] - chi2_vector[1];

                         if((chi2_vector[0] - chi2_vector[1]) < epsilon_threshold){
                            std::cerr<<"[Termination Criterion 1] epsilon condition reached!"<<std::endl;
                            Eigen::VectorXf eigenvalues = multisolver->H().eigenvalues().real();
                            float min_eig = eigenvalues.minCoeff();
                            float max_eig = eigenvalues.maxCoeff();
                            float ratio =min_eig/max_eig;
                            if(ratio > lambda_threshold){
                                std::cerr<<"[Termination Criterion 2] lambda condition reached!"<<std::endl;
                                exploitation = false;
                                std::cerr<<"Calibration Complete. May the force be with you."<<std::endl;
                                return 0;
                            }
                        }
                        chi2_vector.erase(chi2_vector.begin());
                    }
                }

                robot_motions_init_time = ros::Time::now();
            }
        }

        r.sleep();

        if(tf_arrived) {
            if(!exploitation)
                exploration = true;
        }else{
            printf("waiting for Tf\n");
        }
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
                writeCalibrationInfo(calibration_info_vector, str, motions.size());
        }
    }

    return 0;
}