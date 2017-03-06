#include "message_dumper_trigger.h"
#include "srrg_types/defs.h"
#include <list>

namespace srrg_core {

  std::list<std::tr1::shared_ptr<BaseSensorMsg> > SensorMessageList;

  MessageEnqueuerTrigger::MessageEnlisterTrigger(SensorMessageSorter* sorter,
						 int priority,
						 SensorMessageQueue* queue_) :
    SensorMessageSorter::Trigger(sorter, priority) {
    _queue = queue;
  }

  void MessageDumperTrigger::action(BaseSensorMessage& msg) {}

}
