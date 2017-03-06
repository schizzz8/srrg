#include "pose_message.h"

#include <stdio.h>
#include "message_factory.h"

namespace srrg_core
{
 
CPoseMessage::CPoseMessage( const std::string& p_strTopic, const std::string& p_strFrameID, const int& p_uSeq, const double& p_dTimestamp ): BaseSensorMessage( p_strTopic, p_strFrameID, p_uSeq, p_dTimestamp )
{
    //ds nothing to do
}

const std::string CPoseMessage::m_strTag = "POSE_MESSAGE";
  
void CPoseMessage::fromStream( std::istream& p_isMessage )
{
    //ds get base sensor info
    BaseSensorMessage::fromStream( p_isMessage );

    //ds set the other fields
    p_isMessage >> m_vecOrientationQuaternion.w( );
    p_isMessage >> m_vecOrientationQuaternion.x( );
    p_isMessage >> m_vecOrientationQuaternion.y( );
    p_isMessage >> m_vecOrientationQuaternion.z( );
    p_isMessage >> m_vecOrientationEulerAngles[0];
    p_isMessage >> m_vecOrientationEulerAngles[1];
    p_isMessage >> m_vecOrientationEulerAngles[2];
    p_isMessage >> m_vecPosition[0];
    p_isMessage >> m_vecPosition[1];
    p_isMessage >> m_vecPosition[2];
}
  
void CPoseMessage::toStream( std::ostream& p_osMessage ) const
{
    //ds stream the base elements
    BaseSensorMessage::toStream( p_osMessage );

    p_osMessage << " ";

    //ds stream buffer
    char chBuffer[1024];

    //ds format the buffer
    sprintf( chBuffer, "%.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf",
            m_vecOrientationQuaternion.w( ), m_vecOrientationQuaternion.x( ), m_vecOrientationQuaternion.y( ), m_vecOrientationQuaternion.z( ),
            m_vecOrientationEulerAngles[0], m_vecOrientationEulerAngles[1], m_vecOrientationEulerAngles[2],
            m_vecPosition[0], m_vecPosition[1], m_vecPosition[2] );

    //ds write buffer to stream
    p_osMessage << chBuffer;
}

//ds register the message for reading
static MessageFactory::MessageRegisterer< CPoseMessage > m_cRegisterer;

} //namespace srrg_core
