# pragma once

#include "base_sensor_message.h"
#include <string>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace srrg_core {

  class CIMUMessage: public BaseSensorMessage {

    //ds ctor/dtor (MUST NOT THROW)
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      CIMUMessage( const std::string& p_strTopic = "", const std::string& p_strFrameID = "", const int& p_uSeq = -1, const double& p_dTimestamp = -1.0 );

    //ds accessors
  public:

    //ds overrides
    virtual void fromStream( std::istream& p_isMessage );
    virtual void toStream( std::ostream& p_osMessage ) const;
    virtual const std::string& tag( ) const { return m_strTag; }

    inline void setOrientation( const Eigen::Quaterniond& p_vecOrientation ){ m_vecOrientation = p_vecOrientation; }
    inline void setAngularVelocity( const Eigen::Vector3d& p_vecAngularVelocity ){ m_vecAngularVelocity = p_vecAngularVelocity; }
    inline void setLinearAcceleration( const Eigen::Vector3d& p_vecLinearAcceleration ){ m_vecLinearAcceleration = p_vecLinearAcceleration; }

    inline const Eigen::Quaterniond getOrientation( ) const { return m_vecOrientation; }
    inline const Eigen::Vector3d getAngularVelocity( ) const { return m_vecAngularVelocity; }
    inline const Eigen::Vector3d getLinearAcceleration( ) const { return m_vecLinearAcceleration; }

    //ds members (inheritable)
  protected:

    //ds IMU specific
    Eigen::Quaterniond m_vecOrientation;
    Eigen::Vector3d m_vecAngularVelocity;
    Eigen::Vector3d m_vecLinearAcceleration;


    //ds tag
  private:

    static const std::string m_strTag;

  };

}
