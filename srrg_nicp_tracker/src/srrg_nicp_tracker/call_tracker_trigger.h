#pragma once

#include <srrg_txt_io/sensor_message_sorter.h>
#include <srrg_txt_io/message_seq_synchronizer.h>
#include <srrg_txt_io/pinhole_image_message.h>

#include "tracker.h"

namespace srrg_nicp_tracker {
  
  class CallTrackerTrigger: public srrg_core::SensorMessageSorter::Trigger{
  public:
    CallTrackerTrigger(srrg_core::SensorMessageSorter* sorter,
		       int priority,
		       Tracker* tracker_,
		       std::vector<srrg_core::MessageSeqSynchronizer>* synchronizers_=0);
    virtual void action(std::tr1::shared_ptr<srrg_core::BaseSensorMessage> msg);
    
    inline Tracker* tracker() {return _tracker;}
  protected:
    Tracker* _tracker;
    std::vector<srrg_core::MessageSeqSynchronizer>* _synchronizers;
    srrg_core::PinholeImageMessage* _depth_img, *_rgb_img;
  };
}
