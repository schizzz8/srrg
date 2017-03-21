#pragma once

#include "tracker.h"
#include <srrg_system_utils/system_utils.h>

namespace srrg_nicp_tracker {
  
  class VerboseTrigger: public Tracker::Trigger{
  public:
    VerboseTrigger(Tracker* tracker, int event, int priorory, const std::string& message = std::string(""));
    virtual void action(Tracker::TriggerEvent e);
    inline void setMessage(const std::string& msg) {_message = msg;}
    inline const std::string& message() const {return _message; }
    inline std::ostream * outputStream() { return _os; }
    inline void setOutputStream(std::ostream* os) {_os = os;}
    const std::string& lastMessage() { return _last_message; }
  protected:
    void fillTagsMap();
    void replaceTags(std::string& str);
    std::ostream* _os;
    std::string _message;
    std::string _last_message;
    std::map<std::string, std::string> _tags_map;
  };


  class SetMergingTrigger: public Tracker::Trigger{
  public:
    SetMergingTrigger(Tracker* tracker, 
		      int event, 
		      int priorory,
		      bool enable);
    virtual void action(Tracker::TriggerEvent e);
  protected:
    bool _enable;
  };

  class ClearStatusTrigger: public Tracker::Trigger{
  public:
    ClearStatusTrigger(Tracker* tracker, 
		       int event, 
		       int priorory);
    virtual void action(Tracker::TriggerEvent e);
  };


  class ProfilerTrigger: public Tracker::Trigger{
  public:
    ProfilerTrigger(Tracker* tracker, 
		    int event, int priority,
		    srrg_core::SystemUsageCounter* counter) ;

    virtual void action(Tracker::TriggerEvent e);

    inline srrg_core::SystemUsageCounter* usageCounter() {return _usage_counter;}
  protected:
    int _count;
    int _window;
    srrg_core::SystemUsageCounter* _usage_counter;
    
  };

}
