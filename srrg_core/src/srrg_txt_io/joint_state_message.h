#pragma once

#include "base_sensor_message.h"
#include <string>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace srrg_core
{

class JointStateMessage: public BaseSensorMessage
{
//ds ctor/dtor (MUST NOT THROW)
public:
  struct JointStatus{
    std::string name;
    double position;
    double velocity;
    double effort;
  };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    JointStateMessage( const std::string& p_strTopic = "", const std::string& p_strFrameID = "", const int& p_uSeq = -1, const double& p_dTimestamp = -1.0 );

//ds accessors
public:

  //ds overrides
  virtual void fromStream( std::istream& p_isMessage );
  virtual void toStream( std::ostream& p_osMessage ) const;
  virtual const std::string& tag( ) const { return m_strTag; }

  inline const std::vector<JointStatus>& joints() const {return _joints;}
  inline std::vector<JointStatus>& joints() {return _joints;}

protected:
  
  std::vector<JointStatus> _joints;
  
//ds tag
private:

    static const std::string m_strTag;

};

}  //namespace srrg_core
