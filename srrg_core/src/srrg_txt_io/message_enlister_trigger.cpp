#include "message_enlister_trigger.h"
#include "srrg_types/defs.h"
#include <list>

namespace srrg_core {
  
  using namespace std;

  MessageEnlisterTrigger::MessageEnlisterTrigger(SensorMessageSorter* sorter,
						 int priority,
						 SensorMessageList* list_) :
    SensorMessageSorter::Trigger(sorter, priority){
    _list = list_;
    cerr << "message enlister created" << endl;
  }

  void MessageEnlisterTrigger::action(std::tr1::shared_ptr<BaseSensorMessage> msg) { 
    if (_list)
      _list->push_back(msg);
  }

}
