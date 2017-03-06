#pragma once
#include <vector>
#include "base_sensor_message.h"

namespace srrg_core{
  
  class LaserMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      LaserMessage(const std::string& topic="", const std::string& frame_id="",int seq=-1, double timestamp=-1);
      virtual void fromStream(std::istream& is);
      virtual void  toStream(std::ostream& os) const;
      virtual const std::string& tag() const;
    
      inline void setMinAngle(float min_angle) {_min_angle = min_angle;}
      inline float minAngle() {return _min_angle;}
      inline void setMaxAngle(float max_angle) {_max_angle = max_angle;}
      inline float maxAngle() {return _max_angle;}
      inline void setAngleIncrement(float angle_increment) {_angle_increment = angle_increment;}
      inline float angleIncrement() {return _angle_increment;}
      inline void setMinRange(float min_range) {_min_range = min_range;}
      inline float minRange() {return _min_range;}
      inline void setMaxRange(float max_range) {_max_range = max_range;}
      inline float maxRange() {return _max_range;}

      inline void setTimeIncrement(float time_increment) {_time_increment = time_increment;}
      inline float timeIncrement() {return _time_increment;}
      inline void setScanTime(float scan_time) {_scan_time = scan_time;}
      inline float scanTime() {return _scan_time;}

      const std::vector<float>& ranges() const { return _ranges;}
      void setRanges(const std::vector<float>& ranges) {_ranges = ranges;}
      const std::vector<float>& intensities() const { return _intensities;}
      void setIntensities(const std::vector<float>& intensities) {_intensities = intensities;}

  protected:
      static const std::string _tag;
      std::vector<float> _ranges;
      std::vector<float> _intensities;
      float _min_range, _max_range;
      float _min_angle, _max_angle;
      float _angle_increment;
      float _time_increment;
      float _scan_time;
  };

}
