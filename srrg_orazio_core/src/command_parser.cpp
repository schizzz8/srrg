#include "command_parser.h"
#include <map>

namespace srrg_orazio_core {

  using namespace std;

  BaseCommandHandler::BaseCommandHandler(const std::string& tag, OrazioRobotConnection* robot_connection) {
    _robot_connection=robot_connection;
    _tag=tag;
  }
  std::string BaseCommandHandler::help() const {return _tag;}
  BaseCommandHandler::~BaseCommandHandler(){};

  typedef std::map<std::string, BaseCommandHandler*> CommandMap;
  CommandMap command_map;

  class HelpCommandHandler: public BaseCommandHandler{
  public:
    HelpCommandHandler():BaseCommandHandler("help", 0){
    }
    virtual bool handleCommand(istream& is) { 
      for (CommandMap::iterator it=command_map.begin(); it!=command_map.end(); it++){
	cout << it->second->help() << endl;
      }
      return true;
    }
  protected:
    bool* _run;
  };

  class QuitCommandHandler: public BaseCommandHandler{
  public:
    QuitCommandHandler(OrazioRobotConnection* c, bool* run_variable):
      BaseCommandHandler("quit", c){
      _run=run_variable;
    }
    virtual bool handleCommand(istream& is) { *_run = false; return true;}
  protected:
    bool* _run;
  };




#define CALL_COMMAND_1_CLASS(command)					\
  class Call##command: public BaseCommandHandler{			\
  public:								\
  Call##command(OrazioRobotConnection* robot_connection):		\
  BaseCommandHandler(#command, robot_connection){}			\
  virtual std::string help() const {return _tag + " <command_arg>";}	\
  virtual bool handleCommand(istream& is) {				\
    int value;								\
    is >> value;							\
    if (! is)								\
      return false;							\
    cout << "argument: " << value << endl;				\
    return _robot_connection->command(value);				\
  }									\
  };									\

#define GETTER_COMMAND_CLASS(command)					\
  class Get##command: public BaseCommandHandler{			\
  public:								\
  Get##command(OrazioRobotConnection* robot_connection):		\
  BaseCommandHandler(#command, robot_connection){}			\
  virtual bool handleCommand(istream& is) {				\
    cout << #command << "=" << (float) _robot_connection->command() << endl; \
      return true;							\
  }									\
  };									\

#define GETTER_COMMAND_1_CLASS(command)					\
  class Get##command: public BaseCommandHandler{			\
  public:								\
  Get##command(OrazioRobotConnection* robot_connection):		\
  BaseCommandHandler(#command, robot_connection){}			\
  virtual std::string help() const {return _tag + " <joint_num>";}	\
  virtual bool handleCommand(istream& is) {				\
    int value;								\
    is >> value;							\
    if (! is)								\
      return false;							\
    cout << #command << "=" << _robot_connection->command(value) << endl; \
      return true;							\
  }									\
  };									\

#define SETTER_COMMAND_CLASS(command)				\
  class Set##command: public BaseCommandHandler{		\
  public:							\
  Set##command(OrazioRobotConnection* robot_connection):	\
  BaseCommandHandler(#command, robot_connection){}		\
  virtual std::string help() const {return _tag + " <value>";}	\
  virtual bool handleCommand(istream& is) {			\
    float value;						\
    is >> value;						\
    if (! is)							\
      return false;						\
    return _robot_connection->command (value);			\
  }								\
  };								\

#define SETTER_COMMAND_2_CLASS(command)					\
  class Set##command: public BaseCommandHandler{			\
  public:								\
  Set##command(OrazioRobotConnection* robot_connection):		\
  BaseCommandHandler(#command, robot_connection){}			\
  virtual std::string help() const {return _tag + " <joint_num> <value>";} \
  virtual bool handleCommand(istream& is) {				\
    float v1, v2;							\
    is >> v1;								\
    if (! is)								\
      return false;							\
    is >> v2;								\
    if (! is)								\
      return false;							\
    return _robot_connection->command (v1, v2);				\
  }									\
  };									\

  class SetJointsPWM: public BaseCommandHandler{
  public:
    SetJointsPWM(OrazioRobotConnection* robot_connection):
      BaseCommandHandler("setJointsPWM",robot_connection){}
    virtual std::string help() const {return _tag + " <pwm0> <pwm1> ...";} 
    virtual bool handleCommand(istream& is) { 
      int16_t pwm[num_motors];
      for (int i=0; i<num_motors; i++)
	is >> pwm[i];
      _robot_connection->setJointPWM(pwm);
      return true;
    }
  };

  class SetJointsSpeeds: public BaseCommandHandler{
  public:
    SetJointsSpeeds(OrazioRobotConnection* robot_connection):
      BaseCommandHandler("setJointsSpeed",robot_connection){}
    virtual std::string help() const {return _tag + " <speed0> <speed1> ...";} 
    virtual bool handleCommand(istream& is) { 
      int16_t speeds[num_motors];
      for (int i=0; i<num_motors; i++)
	is >> speeds[i];
      _robot_connection->setJointSpeeds(speeds);
      return true;
    }
  };

  class SetBaseVelocities: public BaseCommandHandler{
  public:
    SetBaseVelocities(OrazioRobotConnection* robot_connection):
      BaseCommandHandler("setBaseVelocities",robot_connection){}
    virtual std::string help() const {return _tag + " <tv> <rv>";} 
    virtual bool handleCommand(istream& is) { 
      float tv, rv;
      is >> tv;
      is >> rv;
      _robot_connection->setBaseVelocities(tv,rv);
      return true;
    }
  };


  bool callCommand(const std::string& tag, istream& is){
    CommandMap::iterator it=command_map.find(tag);
    if (it==command_map.end())
      return false;
    return it->second->handleCommand(is);
  }

  GETTER_COMMAND_CLASS(timerPeriod);
  GETTER_COMMAND_CLASS(commCycles);
  GETTER_COMMAND_CLASS(watchdogCycles);
  SETTER_COMMAND_CLASS(setCommCycles);
  SETTER_COMMAND_CLASS(setWatchdogCycles);
  GETTER_COMMAND_CLASS(systemSeq);
  GETTER_COMMAND_CLASS(serverRxPackets);
  GETTER_COMMAND_CLASS(serverTxPackets);
  GETTER_COMMAND_CLASS(serverRxErrors);
  GETTER_COMMAND_CLASS(numJoints);
  GETTER_COMMAND_CLASS(jointSeq);
  GETTER_COMMAND_CLASS(kinematicsSeq);
  GETTER_COMMAND_CLASS(kr);
  GETTER_COMMAND_CLASS(kl);
  GETTER_COMMAND_CLASS(baseline);
  GETTER_COMMAND_CLASS(rightMotorIndex);
  GETTER_COMMAND_CLASS(leftMotorIndex);
  GETTER_COMMAND_CLASS(x);
  GETTER_COMMAND_CLASS(y);
  GETTER_COMMAND_CLASS(theta);
  GETTER_COMMAND_CLASS(translationalVelocityMeasured);
  GETTER_COMMAND_CLASS(rotationalVelocityMeasured);
  GETTER_COMMAND_CLASS(translationalVelocityDesired);
  GETTER_COMMAND_CLASS(rotationalVelocityDesired);
  GETTER_COMMAND_CLASS(translationalVelocityAdjusted);
  GETTER_COMMAND_CLASS(rotationalVelocityAdjusted);
  GETTER_COMMAND_CLASS(motorMode);


  GETTER_COMMAND_1_CLASS(minPWM);
  GETTER_COMMAND_1_CLASS(maxPWM);
  GETTER_COMMAND_1_CLASS(kp);
  GETTER_COMMAND_1_CLASS(ki);
  GETTER_COMMAND_1_CLASS(kd);
  GETTER_COMMAND_1_CLASS(maxSpeed)
  GETTER_COMMAND_1_CLASS(maxI);
  GETTER_COMMAND_1_CLASS(slope);


  SETTER_COMMAND_2_CLASS(setMinPWM);
  SETTER_COMMAND_2_CLASS(setMaxPWM);
  SETTER_COMMAND_2_CLASS(setKp);
  SETTER_COMMAND_2_CLASS(setKi);
  SETTER_COMMAND_2_CLASS(setKd);
  SETTER_COMMAND_2_CLASS(setMaxSpeed)
  SETTER_COMMAND_2_CLASS(setMaxI);
  SETTER_COMMAND_2_CLASS(setSlope);
  SETTER_COMMAND_CLASS(setBaseline);
  SETTER_COMMAND_CLASS(setMotorMode);
  SETTER_COMMAND_CLASS(setKr);
  SETTER_COMMAND_CLASS(setKl);
  SETTER_COMMAND_CLASS(setRightMotorIndex);
  SETTER_COMMAND_CLASS(setLeftMotorIndex);
  SETTER_COMMAND_CLASS(setMaxTranslationalVelocity);
  SETTER_COMMAND_CLASS(setMaxTranslationalAcceleration);
  SETTER_COMMAND_CLASS(setMaxTranslationalBrake);
  SETTER_COMMAND_CLASS(setMaxRotationalVelocity);
  SETTER_COMMAND_CLASS(setMaxRotationalAcceleration);
  
  CALL_COMMAND_1_CLASS(pushParams);
  CALL_COMMAND_1_CLASS(loadParams);
  CALL_COMMAND_1_CLASS(queryParams);
  CALL_COMMAND_1_CLASS(saveParams);



  void initCommandMap(OrazioRobotConnection* robot_connection, bool* halt_flag){
    addCommand(new QuitCommandHandler(robot_connection, halt_flag));
    addCommand(new HelpCommandHandler());
    addCommand(new GettimerPeriod(robot_connection));

    addCommand(new GetcommCycles(robot_connection));
    addCommand(new SetsetCommCycles(robot_connection));

    addCommand(new GetwatchdogCycles(robot_connection));
    addCommand(new SetsetWatchdogCycles(robot_connection));

    addCommand(new SetsetMotorMode(robot_connection));
    addCommand(new GetmotorMode(robot_connection));

    addCommand(new GetsystemSeq(robot_connection));
    addCommand(new GetserverRxPackets(robot_connection));
    addCommand(new GetserverTxPackets(robot_connection));
    addCommand(new GetserverRxErrors(robot_connection));

    addCommand(new GetnumJoints(robot_connection));

    addCommand(new GetminPWM(robot_connection));
    addCommand(new GetmaxPWM(robot_connection));
    addCommand(new Getkp(robot_connection));
    addCommand(new Getki(robot_connection));
    addCommand(new Getkd(robot_connection));
    addCommand(new GetmaxSpeed(robot_connection));
    addCommand(new Getslope(robot_connection));
    addCommand(new GetmaxI(robot_connection));

    addCommand(new SetsetMinPWM(robot_connection));
    addCommand(new SetsetMaxPWM(robot_connection));
    addCommand(new SetsetKp(robot_connection));
    addCommand(new SetsetKi(robot_connection));
    addCommand(new SetsetKd(robot_connection));
    addCommand(new SetsetMaxSpeed(robot_connection));
    addCommand(new SetsetSlope(robot_connection));
    addCommand(new SetsetMaxI(robot_connection));

    addCommand(new GetjointSeq(robot_connection));
    addCommand(new GetkinematicsSeq(robot_connection));
    addCommand(new Getkr(robot_connection));
    addCommand(new Getkl(robot_connection));
    addCommand(new Getbaseline(robot_connection));
    addCommand(new GetrightMotorIndex(robot_connection));
    addCommand(new GetleftMotorIndex(robot_connection));
    addCommand(new Getx(robot_connection));
    addCommand(new Gety(robot_connection));
    addCommand(new Gettheta(robot_connection));
    addCommand(new GettranslationalVelocityMeasured(robot_connection));
    addCommand(new GetrotationalVelocityMeasured(robot_connection));
    addCommand(new GettranslationalVelocityDesired(robot_connection));
    addCommand(new GetrotationalVelocityDesired(robot_connection));
    addCommand(new GettranslationalVelocityAdjusted(robot_connection));
    addCommand(new GetrotationalVelocityAdjusted(robot_connection));
    addCommand(new SetsetKr(robot_connection));
    addCommand(new SetsetKl(robot_connection));
    addCommand(new SetsetBaseline(robot_connection));
    addCommand(new SetsetRightMotorIndex(robot_connection));
    addCommand(new SetsetLeftMotorIndex(robot_connection));
    addCommand(new SetsetMaxTranslationalVelocity(robot_connection));
    addCommand(new SetsetMaxTranslationalAcceleration(robot_connection));
    addCommand(new SetsetMaxTranslationalBrake(robot_connection));
    addCommand(new SetsetMaxRotationalVelocity(robot_connection));
    addCommand(new SetsetMaxRotationalAcceleration(robot_connection));

  
    addCommand(new CallpushParams(robot_connection));
    addCommand(new CallqueryParams(robot_connection));
    addCommand(new CallloadParams(robot_connection));
    addCommand(new CallsaveParams(robot_connection));
  
    addCommand(new SetJointsPWM(robot_connection));
    addCommand(new SetJointsSpeeds(robot_connection));
    addCommand(new SetBaseVelocities(robot_connection));
  }

} // end namespace
