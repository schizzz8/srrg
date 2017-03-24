#pragma once

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include "srrg_nw_calibration_solver/multi_solver.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>

    typedef new_world_calibration::Vector6<float> Vector6f;
    typedef new_world_calibration::Sample2<float> Sample2f;
    typedef new_world_calibration::Sample3<float> Sample3f;
    typedef new_world_calibration::Sample2<float> Sample2f;
    typedef new_world_calibration::Sample3<float> Sample3f;

//Copyright by WillowGarage, ma ch sfaccimm nelle API le chiamate so sul a double, all'anm e chivemmourt!
// http://docs.ros.org/kinetic/api/tf_conversions/html/c++/tf__eigen_8h_source.html
    template<typename Transform>
    void transformTFToEigenImpl(const tf::Transform &t, Transform &e) {
      for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
          e.matrix()(i, j) = t.getBasis()[i][j];
        }
      }
      // Fill in identity in last row
      for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
      e.matrix()(3, 3) = 1;
    }
// ---------- end


    struct tf_frame {
        std::string frame_id_;
        std::string child_frame_id_;
    };

    typedef std::vector<tf_frame> TfFrameVector;
    typedef std::vector<tf::StampedTransform> TfTransformVector;

    std::ostream &operator<<(std::ostream &os, const tf_frame &obj);

    typedef new_world_calibration::Dataset3<float> Dataset3f;
    typedef new_world_calibration::Dataset2<float> Dataset2f;
    typedef new_world_calibration::Dataset2Vector<float> Dataset2Vectorf;
    typedef new_world_calibration::Dataset3Vector<float> Dataset3Vectorf;


    void readInitialGuess(const std::string &init_guess_file,
                          Eigen::VectorXf &guess_odom,
                          new_world_calibration::SensorParams2Vector<float> &sensor2d_params,
                          TfFrameVector &so2_tf_frames,
                          new_world_calibration::SensorParams3Vector<float> &sensor3d_params,
                          TfFrameVector &so3_tf_frames);

    void cleanDataset(Dataset2Vectorf& dataset_vector);
    void cleanDataset(Dataset3Vectorf& dataset_vector);

    struct CalibInfo{
        double time_;
        new_world_calibration::OdomParams<float>* odomparams_;
        new_world_calibration::SensorParams2Vector<float> sensor2params_;
        new_world_calibration::SensorParams3Vector<float> sensor3params_;
        Eigen::MatrixXf H_;

        CalibInfo(const double& time,
                  const new_world_calibration::OdomParams<float>& odomparams,
                  const new_world_calibration::SensorParams2Vector<float>& sensor2params,
                  const new_world_calibration::SensorParams3Vector<float>& sensor3params,
                  const Eigen::MatrixXf& H): time_(time), H_(H)
        {
                odomparams_ = new new_world_calibration::OdomParams<float>(&odomparams);

                new_world_calibration::SensorParams2Vector<float>::const_iterator it = sensor2params.begin();
                for(it; it!=sensor2params.end(); ++it){
                    new_world_calibration::SensorParams2<float>* curr_sensor_params = new new_world_calibration::SensorParams2<float>(*it);
                    sensor2params_.push_back(curr_sensor_params);
                }
                new_world_calibration::SensorParams3Vector<float>::const_iterator it3 = sensor3params.begin();
                for(it3; it3!=sensor3params.end(); ++it3){
                    new_world_calibration::SensorParams3<float>* curr_sensor_params = new new_world_calibration::SensorParams3<float>(*it3);
                    sensor3params_.push_back(curr_sensor_params);
                }
        }
    };

    typedef std::vector<CalibInfo> CalibInfoVector;

    void writeCalibrationInfo(CalibInfoVector& calibration_info_vector, const std::string& time_filename, int motions = 0);