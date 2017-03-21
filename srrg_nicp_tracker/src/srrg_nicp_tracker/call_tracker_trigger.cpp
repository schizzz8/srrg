#include "call_tracker_trigger.h"

namespace srrg_nicp_tracker {

  using namespace srrg_core; 
  
  CallTrackerTrigger::CallTrackerTrigger(SensorMessageSorter* sorter,
					 int priority,
					 Tracker* tracker_,
					 std::vector<MessageSeqSynchronizer>* synchronizers_)
    : SensorMessageSorter::Trigger(sorter, priority) {
    _tracker = tracker_;
    _synchronizers = synchronizers_;
     _depth_img = 0;
     _rgb_img = 0;
  }

  void CallTrackerTrigger::action(std::tr1::shared_ptr<BaseSensorMessage> msg) {
    PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg.get());
    if (! img)
      return;


    Matrix6f odom_info;
    odom_info.setIdentity();
 
    
    if (!_synchronizers) {
      _tracker->processFrame(img->image(),
			     RGBImage(),
			     img->cameraMatrix(),
			     img->depthScale(),
			     img->seq(),
			     img->timestamp(),
			     img->topic(),
			     img->frameId(),
			     img->offset(),
			     img->odometry(),
			     odom_info);
    } else {

      _depth_img = 0;
      _rgb_img = 0;
 
     size_t i;
      for (i=0; i<_synchronizers->size(); i++){
	_synchronizers->at(i).putMessage(msg);
	if (_synchronizers->at(i).messagesReady()) {
	  _depth_img=dynamic_cast<PinholeImageMessage*>(_synchronizers->at(i).messages()[0].get());
	  if (_synchronizers->at(i).messages().size()>1)
	    _rgb_img=dynamic_cast<PinholeImageMessage*>(_synchronizers->at(i).messages()[1].get());
	  break;
	}
      }
      if (_depth_img) {
	RGBImage rgb_image;
	if (_rgb_img)
	  rgb_image =_rgb_img->image();

	_tracker->processFrame(_depth_img->image(),
			       rgb_image,
			       _depth_img->cameraMatrix(),
			       _depth_img->depthScale(),
			       _depth_img->seq(),
			       _depth_img->timestamp(),
			       _depth_img->topic(),
			       _depth_img->frameId(),
			       _depth_img->offset(),
			       _depth_img->odometry(),
			       odom_info);
	_synchronizers->at(i).reset();
      }
    }
    img->untaint();
  }
}
