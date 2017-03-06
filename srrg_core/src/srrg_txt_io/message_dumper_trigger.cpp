#include "message_dumper_trigger.h"
#include "srrg_types/defs.h"

namespace srrg_core {
  
  using namespace std;

  MessageDumperTrigger::MessageDumperTrigger(SensorMessageSorter* sorter,
					     int priority,
					     MessageWriter* writer, 
					     const std::string& file_prefix) :
    SensorMessageSorter::Trigger(sorter, priority){
    _writer = writer;
    _file_prefix = file_prefix;
  }

  void MessageDumperTrigger::action(std::tr1::shared_ptr<BaseSensorMessage> msg){
    _writer->writeMessage(*msg);
    msg->writeBack();
  }

}
