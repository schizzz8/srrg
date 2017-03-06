#ifndef POSE_MESSAGE_H
#define POSE_MESSAGE_H

#include "base_sensor_message.h"
#include <string>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace srrg_core
{

class CPoseMessage: public BaseSensorMessage
{

//ds ctor/dtor (MUST NOT THROW)
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CPoseMessage( const std::string& p_strTopic = "", const std::string& p_strFrameID = "", const int& p_uSeq = -1, const double& p_dTimestamp = -1.0 );

//ds accessors
public:

    //ds overrides
    virtual void fromStream( std::istream& p_isMessage );
    virtual void toStream( std::ostream& p_osMessage ) const;
    virtual const std::string& tag( ) const { return m_strTag; }

    inline void setOrientationQuaternion( const Eigen::Quaterniond& p_vecOrientationQuaternion ){ m_vecOrientationQuaternion = p_vecOrientationQuaternion; }
    inline void setOrientationEulerAngles( const Eigen::Vector3d& p_vecOrientationEulerAngles ){ m_vecOrientationEulerAngles = p_vecOrientationEulerAngles; }
    inline void setOrientationMatrix( const Eigen::Matrix3d& p_matOrientationMatrix ){ m_matOrientationMatrix = p_matOrientationMatrix; }
    inline void setPosition( const Eigen::Vector3d& p_vecPosition ){ m_vecPosition = p_vecPosition; }

    inline const Eigen::Quaterniond getOrientationQuaternion( ) const { return m_vecOrientationQuaternion; }
    inline const Eigen::Vector3d getOrientationEulerAngles( ){ return m_vecOrientationEulerAngles; }
    inline const Eigen::Matrix3d getOrientationMatrix( ){ return m_matOrientationMatrix; }
    inline const Eigen::Vector3d getPosition( ) const { return m_vecPosition; }

//ds members (inheritable)
protected:

    //ds pose specific
    Eigen::Quaterniond m_vecOrientationQuaternion;
    Eigen::Vector3d m_vecOrientationEulerAngles;
    Eigen::Matrix3d m_matOrientationMatrix;
    Eigen::Vector3d m_vecPosition;

//ds tag
private:

    static const std::string m_strTag;

};

}  //namespace srrg_core

#endif //#define POSE_MESSAGE_H
