#include "utils.h"
#include <fstream>

using namespace new_world_calibration;

std::ostream &operator<<(std::ostream &os, const tf_frame &obj) {
  os << "frame_id: " << obj.frame_id_ << "\tchild_frame_id: " << obj.child_frame_id_;
  return os;
}

void readInitialGuess(const std::string &init_guess_file,
		      Eigen::VectorXf &guess_odom,
		      SensorParams2Vector<float> &sensor2d_params,
		      TfFrameVector &so2_tf_frames,
		      SensorParams3Vector<float> &sensor3d_params,
		      TfFrameVector &so3_tf_frames) {

  sensor2d_params.clear();
  so2_tf_frames.clear();
  sensor3d_params.clear();
  so3_tf_frames.clear();


  std::ifstream inputfile;
  inputfile.open(init_guess_file);
  if (!inputfile.is_open()) {
    std::cerr << "***[error]: file path not valid. Error in opening ***" << std::endl;
    std::exit(-1);
  }
  std::string line, prefix;
  while (!inputfile.eof()) {
    getline(inputfile, line);

    if (line.length() != 0 && line[0] != '#') //discard line beginning with '#'
      {
	//std::cout << line << "\n";
	char *pch;
	char *cstr_line = new char[line.length() + 1];
	strcpy(cstr_line, line.c_str());
	pch = std::strtok(cstr_line, " ");

	if (!strcmp(pch, "ODOM_PARAMS")) {
	  std::cerr << "reading odom params" << std::endl;
	  pch = std::strtok(NULL, " ");
	  std::vector<float> odom_params;
	  while (pch != NULL) {

	    std::istringstream ss(pch);
	    float f;
	    if (ss >> f)
	      odom_params.push_back(f);
	    else
	      std::cerr << "***[error]: invalid conversion from string to float ***" << std::endl;
	    pch = std::strtok(NULL, " ");
	    //                        std::cerr<<" "<<f<<std::endl;
	  }
	  guess_odom.setZero(odom_params.size());
	  //                guess_odom = Eigen::VectorXf(odom_params.data());
	  for (size_t i = 0; i < odom_params.size(); ++i)
	    guess_odom(i) = odom_params.at(i);
	} else if (!strcmp(pch, "SENSOR2_PARAMS")) {
	  std::cerr << "reading sensor2 params with frames [";
	  pch = std::strtok(NULL, " ");
	  std::string base_frame = pch;
	  std::cerr << base_frame;
	  pch = std::strtok(NULL, " ");
	  std::string sensor_frame = pch;
	  std::cerr << " > " << sensor_frame << "]" << std::endl;
	  pch = std::strtok(NULL, " ");
	  std::vector<float> sensor_params;
	  while (pch != NULL) {
	    std::istringstream ss(pch);
	    float f;
	    if (ss >> f)
	      sensor_params.push_back(f);
	    else
	      std::cerr << "***[error]: invalid conversion from string to float ***" << std::endl;
	    pch = std::strtok(NULL, " ");
	    //                        std::cerr<<" "<<f<<std::endl;
	  }
	  Eigen::Vector3f guess_sensor2(sensor_params.data());
	  SensorParams2<float> *sensor2_params = new SensorParams2<float>(
									  v2t((Eigen::Vector3f) guess_sensor2),
									  sensor_frame,
									  base_frame);
	  sensor2d_params.push_back(sensor2_params);
	  tf_frame frame;
	  frame.frame_id_ = base_frame;
	  frame.child_frame_id_ = sensor_frame;
	  so2_tf_frames.push_back(frame);

	} else if (!strcmp(pch, "SENSOR3_PARAMS")) {
	  std::cerr << "reading sensor3 params with frames [";
	  pch = std::strtok(NULL, " ");
	  std::string base_frame = pch;
	  std::cerr << base_frame;
	  pch = std::strtok(NULL, " ");
	  std::string sensor_frame = pch;
	  std::cerr << " > " << sensor_frame << "]" << std::endl;
	  pch = std::strtok(NULL, " ");
	  std::vector<float> sensor_params;
	  while (pch != NULL) {
	    std::istringstream ss(pch);
	    float f;
	    if (ss >> f)
	      sensor_params.push_back(f);
	    else
	      std::cerr << "***[error]: invalid conversion from string to float ***" << std::endl;
	    pch = std::strtok(NULL, " ");
	  }
	  Vector6f guess_sensor3(sensor_params.data());
	  SensorParams3<float> *sensor3_params = new SensorParams3<float>(v2t((Vector6f) guess_sensor3),
									  sensor_frame,
									  base_frame);
	  sensor3d_params.push_back(sensor3_params);
	  tf_frame frame;
	  frame.frame_id_ = base_frame;
	  frame.child_frame_id_ = sensor_frame;
	  so3_tf_frames.push_back(frame);
	}

	delete[] cstr_line;

      }

  }// End While
  inputfile.close();

}


void cleanDataset(Dataset2Vectorf& dataset_vector){
  for(Dataset2Vectorf::iterator it = dataset_vector.begin(); it != dataset_vector.end(); ++it){
    (*it)->clear();
  }
}

void cleanDataset(Dataset3Vectorf& dataset_vector){
  for(Dataset3Vectorf::iterator it = dataset_vector.begin(); it != dataset_vector.end(); ++it){
    (*it)->clear();
  }
}

void writeCalibrationInfo(CalibInfoVector& calibration_info_vector, const std::string& filename, int motions){

  std::ofstream writer;
  writer.open(filename, std::ios::out | std::ios::app | std::ios::binary);

  //first write an info rows indicating how many sensors && their type are involved
  writer << "AUTO "<< motions << " ODOM_SIZE " << calibration_info_vector.at(0).odomparams_->size() << " SENSOR2 " << calibration_info_vector.at(0).sensor2params_.size()
	 << " SENSOR3 "<< calibration_info_vector.at(0).sensor3params_.size() << std::endl;

  for(CalibInfoVector::iterator it=calibration_info_vector.begin(); it!=calibration_info_vector.end(); ++it){
    writer << "TIME "<<(*it).time_ << " ";
    writer << "/odom " << ((*it).odomparams_->odomParams()).transpose() << " ";
    for(new_world_calibration::SensorParams2Vector<float>::iterator s2 = (*it).sensor2params_.begin();
	s2 != (*it).sensor2params_.end();
	++s2){
      writer << (*s2)->frame() << " ";
      Eigen::Vector3f params = t2v((*s2)->isometry());
      writer << params.transpose() << " ";
    }
    for(new_world_calibration::SensorParams3Vector<float>::iterator s3 = (*it).sensor3params_.begin();
	s3 != (*it).sensor3params_.end();
	++s3){
      writer << (*s3)->frame() << " ";
      new_world_calibration::Vector6<float> params = t2v((*s3)->isometry());
      writer << params.transpose() << " ";
    }
    Eigen::VectorXf rolled_mat = Eigen::Map<Eigen::VectorXf>((*it).H_.data(), (*it).H_.cols()*(*it).H_.rows());
    writer << "H " << rolled_mat.transpose();
    writer << std::endl;
  }

  writer.close();

}
