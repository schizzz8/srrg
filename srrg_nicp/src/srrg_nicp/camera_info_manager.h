#pragma once
#include <srrg_core_map/base_camera_info.h>
#include <set>

namespace srrg_nicp {

  class CameraInfoManager: public srrg_boss::Identifiable {
  public:
    CameraInfoManager(int id=-1,
		      srrg_boss::IdContext* context=0);

    ~CameraInfoManager() ;

    srrg_core_map::BaseCameraInfo* getCamera(const std::string& topic) ;

    srrg_core_map::BaseCameraInfo* hasCamera(srrg_core_map::BaseCameraInfo* cam);
    
    void addCamera(srrg_core_map::BaseCameraInfo* cam);

    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    
    std::vector<srrg_core_map::BaseCameraInfo*>& cameras() {return _camera_info_vector; }
  protected:
    std::vector<srrg_core_map::BaseCameraInfo*> _camera_info_vector;
    std::map<std::string, srrg_core_map::BaseCameraInfo*> _camera_info_map;
    std::set<srrg_core_map::BaseCameraInfo*> _camera_info_set;
  };

}
