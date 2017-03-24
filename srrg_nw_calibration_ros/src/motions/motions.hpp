#ifndef MOTIONS_HPP
#define MOTIONS_HPP

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

//define here motions - typedef - stuff
struct Motion{
    Motion(){}
    ~Motion(){}
    Motion(const geometry_msgs::Twist &twist_go, const geometry_msgs::Twist &twist_back_home, const std::string &name/*, const float &lin_distance_to_cover, const float &ang_distance_to_cover*/)
    {
        _twist_go = twist_go;
        _twist_back_home = twist_back_home;
        _name = name;
//        _lin_distance_to_cover = lin_distance_to_cover;
//        _ang_distance_to_cover = ang_distance_to_cover;
        _H_mat_already_set = false;
    }

    /** if an H matrix has already been set, it means that the backward motions is going to be stored,
     * so sum the contributions of both motions in a single H matrix
     *
     * @param H, the "Hessian" of the ls problem
     */
    inline void setH(const Eigen::MatrixXf& H){(_H_mat_already_set)? _H += H : _H = H; _H_mat_already_set = true;}
    Eigen::MatrixXf H(){return _H;}

    geometry_msgs::Twist _twist_go;
    geometry_msgs::Twist _twist_back_home;
    std::string _name;
//    float _lin_distance_to_cover;
//    float _ang_distance_to_cover;
    Eigen::MatrixXf _H;
    bool _H_mat_already_set;

};

typedef std::vector<Motion> Motions;

inline Motions::iterator findBestMotion(Motions& motions, const Eigen::MatrixXf &H = Eigen::MatrixXf::Zero(1,1)){
    float best_determinant = 0;
    Motions::iterator best;
    best= motions.begin();

    for(Motions::iterator it = motions.begin(); it != motions.end(); ++it){

        float current_det;
        if(H == Eigen::MatrixXf::Zero(1,1))
            current_det = ((*it)._H).determinant();
        else
            current_det = (H + (*it)._H).determinant();
        if(current_det > best_determinant)
        {
            best_determinant = current_det;
            best = it;
        }
    }
    std::cerr<<"selected motion: "<<(*best)._name<<std::endl;

    return best;
}


#endif // MOTIONS_HPP
