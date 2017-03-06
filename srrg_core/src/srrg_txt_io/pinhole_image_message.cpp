#include "pinhole_image_message.h"
#include <cstdio>
#include <cstring>
#include "message_factory.h"

namespace srrg_core {

  using namespace std;

  PinholeImageMessage::PinholeImageMessage(const std::string& topic_, const std::string& frame_id, int seq_, double timestamp_):
    BaseImageMessage(topic_,frame_id, seq_,timestamp_){
    _camera_matrix.setIdentity();
  }
  
  const std::string PinholeImageMessage::_tag="PINHOLE_IMAGE_MESSAGE";

  const std::string& PinholeImageMessage::tag() const { return _tag; }

  void PinholeImageMessage::fromStream(std::istream& is_) {
    BaseImageMessage::fromStream(is_);

    //ds deserialize camera matrix
    for (int r =0; r < 2; r++)
      for (int c = 0; c < 3; c++)
        is_ >> _camera_matrix(r,c);
    _camera_matrix.row(2) << 0,0,1;

    _is_fetched = false;
  }

  void PinholeImageMessage::toStream(std::ostream& os_) const {
    BaseImageMessage::toStream(os_);
    os_ << " ";

    //ds serialize camera matrix
    for (int r =0; r < 2; r++)
      for (int c = 0; c < 3; c++)
        os_ << _camera_matrix(r,c) << " ";
  }

  static MessageFactory::MessageRegisterer<PinholeImageMessage> registerer;
}
