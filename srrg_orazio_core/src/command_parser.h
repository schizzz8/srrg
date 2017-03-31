#pragma once
#include <string>
#include <iostream>
#include "orazio_robot_connection.h"
#include <map>
namespace srrg_orazio_core {

  class BaseCommandHandler{
  public:
    BaseCommandHandler(const std::string& tag, OrazioRobotConnection* robot_connection);
    inline const std::string tag() const {return _tag;}
    virtual bool handleCommand(istream& is)=0;
    virtual std::string help() const;
    virtual ~BaseCommandHandler();
  protected:
    OrazioRobotConnection* _robot_connection;
    std::string _tag;
  };
  typedef std::map<std::string, BaseCommandHandler*> CommandMap;

  void initCommandMap(OrazioRobotConnection* robot_connection, bool* halt_flag);
  bool callCommand(const std::string& tag, std::istream& is);

  extern CommandMap command_map;

  template <typename T>
  void addCommand(T command){
    command_map.insert(std::make_pair(command->tag(), command));
  }

}// end namespace
