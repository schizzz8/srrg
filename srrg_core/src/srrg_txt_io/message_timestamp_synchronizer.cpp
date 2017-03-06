#include "message_timestamp_synchronizer.h"
#include <iostream>

namespace srrg_core {
  using namespace std;

  MessageTimestampSynchronizer::MessageTimestampSynchronizer() {
    _last_stamp=-1;
  }

  void MessageTimestampSynchronizer::setTopics(const std::vector<std::string>& topics_) {
    _topics = topics_;
    _messages.resize(_topics.size());
  }

  void MessageTimestampSynchronizer::putMessage(BaseSensorMessage* msg) {
    for(size_t i=0; i<_topics.size(); i++){
      if (_topics[i]==msg->topic()) {
	_messages[i]=std::tr1::shared_ptr<BaseSensorMessage>(msg);
	if(msg->seq()>_last_stamp)
	  _last_stamp=msg->timestamp();
      }
    }
  }


  void MessageTimestampSynchronizer::putMessage(std::tr1::shared_ptr<BaseSensorMessage>& msg) {
    for(size_t i=0; i<_topics.size(); i++){
      if (_topics[i]==msg->topic()) {
	_messages[i]=msg;
	if(msg->seq()>_last_stamp)
	  _last_stamp=msg->timestamp();
      }
    }
  }


  bool MessageTimestampSynchronizer::messagesReady() {
    bool ready=false;
    for(size_t i=0; i<_messages.size(); i++){
      if (!_messages[i].get())
	return false;
      if (_last_stamp-_messages[i]->timestamp()>_time_interval)
	return false;
    }
    return true;
  }


  void MessageTimestampSynchronizer::reset() {
    for(size_t i=0; i<_messages.size(); i++){
      _messages[i].reset();
    }
    _last_stamp = -1;
  }

}
