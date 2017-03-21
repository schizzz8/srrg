#include "camera_info_manager.h"

namespace srrg_nicp {

  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_core_map;
    
  CameraInfoManager::CameraInfoManager(int id,
				       IdContext* context): Identifiable(id,context){
  }
  
  CameraInfoManager::~CameraInfoManager() {
    // scan the set and delete all camera infos;
    for (std::set<BaseCameraInfo*>::iterator it=_camera_info_set.begin();
	 it!=_camera_info_set.end(); it++) {
      delete *it;
    }
  }

  BaseCameraInfo* CameraInfoManager::getCamera(const std::string& topic) {
    std::map<std::string, BaseCameraInfo*>::iterator it = _camera_info_map.find(topic);
    if (it==_camera_info_map.end()){
      return 0;
    }
    return it->second;
  }

  BaseCameraInfo* CameraInfoManager::hasCamera(BaseCameraInfo* cam) {
    std::set<BaseCameraInfo*>::iterator it=_camera_info_set.find(cam);
    if (it!=_camera_info_set.end())
      return cam;
    return 0;
  }
    
  void CameraInfoManager::addCamera(BaseCameraInfo* cam) {
    BaseCameraInfo* other = getCamera(cam->topic());
    if (! other ) {
      _camera_info_map.insert(make_pair(cam->topic(), cam));
      _camera_info_set.insert(cam);
      _camera_info_vector.push_back(cam);
    } else {
      throw std::runtime_error("Camera is already in manager");
    }
  }

  void CameraInfoManager::serialize(ObjectData& data, IdContext& context){
    Identifiable::serialize(data,context);
    ArrayData* camerasArray = new ArrayData;
    for (std::set<BaseCameraInfo*>::iterator it = _camera_info_set.begin(); it!=_camera_info_set.end(); it++){
      BaseCameraInfo* cam=*it;
      camerasArray->add(new PointerData(cam));
    }
    data.setField("cameras", camerasArray);
  }

  void CameraInfoManager::deserialize(ObjectData& data, IdContext& context){
    Identifiable::deserialize(data,context);
    ArrayData& camerasArray=data.getField("cameras")->getArray();
    for (size_t i =0; i< camerasArray.size(); i++){
      ValueData& v = camerasArray[i];
      Identifiable* id = v.getPointer();
      BaseCameraInfo* cam = dynamic_cast<BaseCameraInfo*>(id);
      _camera_info_set.insert(cam);
      _camera_info_map.insert(make_pair(cam->topic(), cam));
    }

  }
}
