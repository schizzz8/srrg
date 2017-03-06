#include "laser_message.h"
#include "message_factory.h"
#include <iostream>

namespace srrg_core{
  using namespace std;

  LaserMessage::LaserMessage(const std::string& topic, const std::string& frame_id,int seq, double timestamp):
    BaseSensorMessage(topic, frame_id, seq, timestamp){
      
  }
  
  const std::string LaserMessage::_tag="LASER_MESSAGE";

  const std::string& LaserMessage::tag() const {
    return _tag;
  }
  void LaserMessage::fromStream(std::istream& is) {
    BaseSensorMessage::fromStream(is);
    is >> _min_range >> _max_range >> _min_angle >> _max_angle >> _angle_increment >> _time_increment >> _scan_time;
    int num_ranges;
    is >> num_ranges;
    if(num_ranges)
      _ranges.resize(num_ranges);
    else
      _ranges.clear();

    for (size_t i=0; i<num_ranges; i++){
      is >> _ranges[i];
    }
      
    int num_intensities;
    is >> num_intensities;
    if(num_intensities)
      _intensities.resize(num_intensities);
    else
      _intensities.clear();
      
    for (size_t i=0; i<num_intensities; i++){
      is >> _intensities[i];
    }
      
  }

  void  LaserMessage::toStream(std::ostream& os) const {
    BaseSensorMessage::toStream(os);
    os << " " << _min_range
       << " " << _max_range
       << " " << _min_angle
       << " " << _max_angle
       << " " << _angle_increment
       << " " << _time_increment
       << " " << _scan_time;

    os << " " << _ranges.size() << " ";
    for (size_t i=0; i<_ranges.size(); i++){
      os << _ranges[i] << " ";
    }
    os << " " << _intensities.size() << " ";
    for (size_t i=0; i<_intensities.size(); i++){
      os << _intensities[i] << " ";
    }
  }

  static MessageFactory::MessageRegisterer<LaserMessage> laser_registerer;
}
