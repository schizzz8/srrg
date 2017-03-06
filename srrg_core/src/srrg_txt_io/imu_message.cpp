#include "imu_message.h"

#include <stdio.h>
#include "message_factory.h"

namespace srrg_core
{
 
CIMUMessage::CIMUMessage( const std::string& p_strTopic, const std::string& p_strFrameID, const int& p_uSeq, const double& p_dTimestamp ): BaseSensorMessage( p_strTopic, p_strFrameID, p_uSeq, p_dTimestamp )
{
    //ds nothing to do
}

const std::string CIMUMessage::m_strTag = "IMU_MESSAGE";
  
void CIMUMessage::fromStream( std::istream& p_isMessage )
{
    //ds get base sensor info
    BaseSensorMessage::fromStream( p_isMessage );

    //ds set the other fields
    p_isMessage >> m_vecOrientation.w( );
    p_isMessage >> m_vecOrientation.x( );
    p_isMessage >> m_vecOrientation.y( );
    p_isMessage >> m_vecOrientation.z( );
    p_isMessage >> m_vecAngularVelocity[0];
    p_isMessage >> m_vecAngularVelocity[1];
    p_isMessage >> m_vecAngularVelocity[2];
    p_isMessage >> m_vecLinearAcceleration[0];
    p_isMessage >> m_vecLinearAcceleration[1];
    p_isMessage >> m_vecLinearAcceleration[2];
}
  
void CIMUMessage::toStream( std::ostream& p_osMessage ) const
{
    //ds stream the base elements
    BaseSensorMessage::toStream( p_osMessage );

    p_osMessage << " ";

    //ds stream buffer
    char chBuffer[1024];

    //ds format the buffer
    sprintf( chBuffer, "%.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf",
            m_vecOrientation.w( ), m_vecOrientation.x( ), m_vecOrientation.y( ), m_vecOrientation.z( ),
            m_vecAngularVelocity[0], m_vecAngularVelocity[1], m_vecAngularVelocity[2],
            m_vecLinearAcceleration[0], m_vecLinearAcceleration[1], m_vecLinearAcceleration[2] );

    //ds write buffer to stream
    p_osMessage << chBuffer;
}

//ds register the message for reading
static MessageFactory::MessageRegisterer< CIMUMessage > m_cRegisterer;

} //namespace srrg_core
