#pragma once
#include "base_sensor_message.h"
#include <vector>
#include <string>
#include <tr1/memory>

namespace srrg_core {

  class MessageTimestampSynchronizer{
  public:
    MessageTimestampSynchronizer();
    inline double timeInterval() const {return _time_interval;}
    void setTimeInterval(double interval) {_time_interval=interval;}
    void setTopics(const std::vector<std::string>& topics_);
    inline const std::vector<std::string> topics() const {return _topics;}
    inline std::vector<std::tr1::shared_ptr<BaseSensorMessage> >& messages() { return _messages; }
    void putMessage(BaseSensorMessage* msg);
    void putMessage(std::tr1::shared_ptr<BaseSensorMessage>& msg);
    bool messagesReady();
    void reset();
  protected:
    std::vector<std::tr1::shared_ptr<BaseSensorMessage> > _messages;
    std::vector<std::string> _topics;
    double _last_stamp;
    double _time_interval;
  };

}
