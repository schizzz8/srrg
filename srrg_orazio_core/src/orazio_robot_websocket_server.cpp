#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <curses.h>
#include "orazio_robot_connection.h"
#include <iostream>
#include <sstream>
#include <map>
#include <libwebsockets.h>
#include <cstring>
#include <set>
#include "command_parser.h"

using namespace std;
using namespace srrg_orazio_core;

const char *banner[]={
  "orazio_robot_websocket_server",
  "configures orazio from a web browser",
  "usage:"
  "  $> orazio_robot_websocket_server <parameters>",
  "starts a web server on localhost:9000",
  "parameters: ",
  "-serial-device <string>: the serial port (default /dev/ttyACM0)",
  "-resource-path <string>:  the path containing the html ",
  "                         files that will be served by ",
  "                         embedded http server (default PWD)",
  " resource path should be set to the html folder of this repo",
  0
};

void printBanner(){
  const char*const* line=banner;
  while (*line) {
    cout << *line << endl;
    line++;
  }
}


// this is nasty! the libwebsocket API changed from14.04 to 16.04
// prefix was libwebsocket, and become lws

enum WindowMode {System=1, Joints=2, Kinematics=3, Joystick=4};

WindowMode mode=System;
const int PATH_SIZE=1024;
char resource_path[PATH_SIZE];

#ifdef __UBUNTU_16_04__
// orrible list of defines
#define libwebsocket                         lws
#define libwebsocket_context		     lws_context
#define libwebsocket_callback_reasons	     lws_callback_reasons
#define libwebsocket_protocols 		     lws_protocols
#define libwebsocket_get_internal_extensions lws_get_internal_extensions
#define libwebsocket_create_context	     lws_create_context
#define libwebsocket_write		     lws_write
#define libwebsocket_service		     lws_service
#define libwebsocket_context_destroy	     lws_context_destroy
#define libwebsockets_serve_http_file        lws_serve_http_file
#define libwebsocket_close_and_free_session  lws_close_and_free_session

static int callback_http(struct libwebsocket *wsi,
                         enum libwebsocket_callback_reasons reason, void *user,
                         void *in, size_t len) {
  switch (reason) {
  case LWS_CALLBACK_CLIENT_WRITEABLE:
    printf("connection established\n");
  case LWS_CALLBACK_HTTP: 
    char *requested_uri = (char *) in;
    printf("requested URI: %s\n", requested_uri);
    char* mime = (char*) "text/html";
    char base_uri[1024];
    char mode_string[1024];
    mode_string[0]=0;
    base_uri[0]=0;
    sscanf(requested_uri, "%[^?]s", base_uri);
    sscanf(requested_uri+strlen(base_uri),"%s",mode_string);
    cerr << "uri: " << base_uri << endl;
    cerr << "mode_string: " << mode_string << endl;

    if(!strcmp(mode_string,"?system")){
      mode=System;
    } else if(!strcmp(mode_string,"?joints")){
      mode=Joints;
    } else if(!strcmp(mode_string,"?kinematics")){
      mode=Kinematics;
    } else if(!strcmp(mode_string,"?joystick")){
      mode=Joystick;
    }
    if (! strcmp(base_uri,"/") ){
      strcpy(base_uri, "/index.html");
    }
    char resource_file[PATH_SIZE];
    strcpy(resource_file,resource_path);
    strcat(resource_file,base_uri);
    cerr << "serving" << resource_file << endl;
    lws_serve_http_file(wsi, resource_file, mime, 0, 0);

    return 1;
    break;
      
  }
    
  return 0;
}

#else

static int callback_http(struct libwebsocket_context * context,
                         struct libwebsocket *wsi,
                         enum libwebsocket_callback_reasons reason, void *user,
                         void *in, size_t len) {

   
  switch (reason) {
  case LWS_CALLBACK_CLIENT_WRITEABLE:
    printf("connection established\n");
  case LWS_CALLBACK_HTTP: {
    char *requested_uri = (char *) in;
    printf("requested URI: %s\n", requested_uri);
    char* mime = (char*) "text/html";
    char base_uri[1024];
    char mode_string[1024];
    mode_string[0]=0;
    base_uri[0]=0;
    sscanf(requested_uri, "%[^?]s", base_uri);
    sscanf(requested_uri+strlen(base_uri),"%s",mode_string);
    cerr << "uri: " << base_uri << endl;
    cerr << "mode_string: " << mode_string << endl;

    if(!strcmp(mode_string,"?system")){
      mode=System;
    } else if(!strcmp(mode_string,"?joints")){
      mode=Joints;
    } else if(!strcmp(mode_string,"?kinematics")){
      mode=Kinematics;
    } else if(!strcmp(mode_string,"?joystick")){
      mode=Joystick;
    }
    if (! strcmp(base_uri,"/") ){
      strcpy(base_uri, "/index.html");
    }
    char resource_file[PATH_SIZE];
    strcpy(resource_file,resource_path);
    strcat(resource_file,base_uri);
    cerr << "serving" << resource_file << endl;
    libwebsockets_serve_http_file(context, wsi, resource_file, mime);
            
    // close connection
    // libwebsocket_close_and_free_session(context, wsi,
    //                                     LWS_CLOSE_STATUS_NORMAL);
    return 1;
    break;
  }
  default:
    printf("unhandled callback\n");
    break;
  }
   
  return 0;
}

#endif

OrazioRobotConnection robot_connection;

void printResponsePacket(std::string& buf, const ResponsePacket& p){
  ostringstream  os;
  os << "<b> RESPONSE </b> s: " << p.seq 
     << " id: " << (int) p.src_id 
     << " e: " << (int) p.error_code << "<br/>";
  buf=os.str();
}


#define printAttribute(os,struct_name,name)				\
  os<< "<tr><td valign=\"top\"> "<< #name << "</td><td style=\"min-width:120px\" >" << struct_name.name <<"</td/></tr>"

#define printAttributeNoStruct(os,name)				\
  os<< "<tr><td valign=\"top\"> "<< #name << "</td><td style=\"min-width:120px\" >" << name <<"</td></tr>"

void printSystemStatusPacket(std::string& buf, const SystemStatusPacket& p){
  ostringstream  os;
  os <<"<table border=\"1\">";
  printAttribute(os,p,seq);
  printAttribute(os,p,rx_packets);
  printAttribute(os,p,rx_packet_errors);
  printAttribute(os,p,watchdog_count);
  printAttribute(os,p,battery_level);
  int last_packet_id=p.last_packet_id;
  printAttributeNoStruct(os,last_packet_id);
  printAttribute(os,p,idle_cycles);
  printAttribute(os,robot_connection,packetCount());
  printAttribute(os,robot_connection,packetErrors());
  os <<"</table>";
  buf=os.str();
}

void printSystemParamsPacket(std::string& buf, const SystemParamPacket& p){
  ostringstream  os;
  os <<"<table border=\"1\">";
  printAttribute(os,p,seq);
  printAttribute(os,p,comm_speed);
  printAttribute(os,p,comm_cycles);
  printAttribute(os,p,watchdog_cycles);
  int motor_mode=p.motor_mode;
  printAttributeNoStruct(os,motor_mode);
  printAttribute(os,p,timer_period);
  os <<"</table>";
  buf=os.str();
}

void printJointStatusPacket(std::string& buf, const JointStatusPacket& p){
  ostringstream  os;
  os <<"<table border=\"1\">";
  printAttribute(os,p,seq);
  os <<"<tr><th> num </th><th>attributes</th></tr>"; 
  for(int i=0; i<num_motors; i++){
    os <<"<tr> <td> " << i << "</td><td><table border=\"1\">";
    const JointInfo& joint=p.joints[i];
    printAttribute(os,joint,mode);
    printAttribute(os,joint,encoder_position);
    printAttribute(os,joint,encoder_speed);
    printAttribute(os,joint,desired_speed);
    printAttribute(os,joint,pwm);
    printAttribute(os,joint,sensed_current);
    os << "</table></td></tr>";
  }
  os <<"</table>";
  buf=os.str();
}


void printJointParamsPacket(std::string& buf, const JointParamPacket& p){
  ostringstream  os;
  os <<"<table border=\"1\">";
  printAttribute(os,p,seq);
  os <<"<tr><th> num </th><th>parameters</th></tr>"; 
  for(int i=0; i<num_motors; i++){
    os <<"<tr> <td> " << i << "</td><td><table border=\"1\">";
    const JointParams& joint=p.params[i];
    printAttribute(os,joint,kp);
    printAttribute(os,joint,ki);
    printAttribute(os,joint,kd);
    printAttribute(os,joint,max_speed);
    printAttribute(os,joint,min_pwm);
    printAttribute(os,joint,max_pwm);
    printAttribute(os,joint,slope);
    os << "</table></td></tr>";
  }
  os <<"</table>";
  buf=os.str();
}

void printKinematicsParamsPacket(std::string& buf, const DifferentialDriveParamPacket& p){
  ostringstream  os;
  os <<"<table border=\"1\">";
  printAttribute(os,p,seq);
  printAttribute(os,p,baseline);
  printAttribute(os,p,kr);
  printAttribute(os,p,kl);
  printAttribute(os,p,max_translational_velocity);
  printAttribute(os,p,max_translational_acceleration);
  printAttribute(os,p,max_translational_brake);
  printAttribute(os,p,max_rotational_velocity);
  printAttribute(os,p,max_rotational_acceleration);
 
  int right_joint_index=p.right_joint_index;
  int left_joint_index=p.left_joint_index;
  printAttributeNoStruct(os,right_joint_index);
  printAttributeNoStruct(os,left_joint_index);
  os <<"</table>";
  buf=os.str();
}

void printKinematicsStatus(std::string& buf, const DifferentialDriveStatusPacket& p){
  ostringstream  os;
  os <<"<table border=\"1\">";
  printAttribute(os,p,seq);
  printAttribute(os,p,odom_x);
  printAttribute(os,p,odom_y);
  printAttribute(os,p,odom_theta);
  printAttribute(os,p,translational_velocity_measured);
  printAttribute(os,p,rotational_velocity_measured);
  printAttribute(os,p,translational_velocity_desired);
  printAttribute(os,p,rotational_velocity_desired);
  printAttribute(os,p,translational_velocity_adjusted);
  printAttribute(os,p,rotational_velocity_adjusted);
  os <<"</table>";
  buf=os.str();
}

bool run;
unsigned const int BUF_SIZE=1024;
std::string response_string;
std::string system_status_string;
std::string joint_status_string;
std::string kinematic_status_string;
std::string system_params_string;
std::string joint_params_string;
std::string kinematic_params_string;

unsigned char response[10240];
unsigned char* response_begin=response+LWS_SEND_BUFFER_PRE_PADDING;
int response_length=0;

typedef struct libwebsocket * WebSocketConnectionPtr;

typedef std::set<WebSocketConnectionPtr> WebSocketConnectionPtrSet;

WebSocketConnectionPtrSet client_connections;

#ifdef __UBUNTU_16_04__
static int callback_input_command( /*struct libwebsocket_context * this_context,*/
                                   struct libwebsocket *wsi,
                                   enum libwebsocket_callback_reasons reason,
                                   void *user, void *in, size_t len) {
  switch (reason) {
  case LWS_CALLBACK_ESTABLISHED: // just log message that someone is connecting
    printf("connection established\n");
    client_connections.insert(wsi);
    break;

  case LWS_CALLBACK_RECEIVE:  // the funny part
    {
      printf("received data: %s size %d \n", static_cast<unsigned char *>(in), (int) len);
      istringstream is((char*)in);
      std::string tag;
      is >> tag;
      callCommand(tag, is);
    }
    break;

  case LWS_CALLBACK_CLOSED: { // the funny part
    printf("connection closed \n");
    client_connections.erase(wsi);
    break;

  }
  default:
    break;
  }
    
  return 0;
}
#else
static int callback_input_command(struct libwebsocket_context * this_context,
                                   struct libwebsocket *wsi,
                                   enum libwebsocket_callback_reasons reason,
                                   void *user, void *in, size_t len) {
  switch (reason) {
  case LWS_CALLBACK_ESTABLISHED: // just log message that someone is connecting
    printf("connection established\n");
    client_connections.insert(wsi);
    break;

  case LWS_CALLBACK_RECEIVE:  // the funny part
    {
      printf("received data: %s size %d \n", static_cast<unsigned char *>(in), (int) len);
      istringstream is((char*)in);
      std::string tag;
      is >> tag;
      callCommand(tag, is);
    }
    break;

  case LWS_CALLBACK_CLOSED: { // the funny part
    printf("connection closed \n");
    client_connections.erase(wsi);
    break;

  }
  default:
    break;
  }
    
  return 0;
}
#endif

static struct libwebsocket_protocols protocols[] = {
    /* first protocol must always be HTTP handler */
    {
        "http-only",   // name
        callback_http, // callback
        0              // per_session_data_size
    },
    {
        "orazio-robot-protocol", // protocol name - very important!
        callback_input_command,   // callback
        0                          // we don't use any per session data
    },
    {
        NULL, NULL, 0   /* End of list */
    }
};


 int main(int argc, char** argv) {
   int c=1;
   resource_path[0]=0;
   const char* serial_device="/dev/ttyACM0";
   while (c<argc) {
     if (! strcmp(argv[c],"-resource-path")) {
       c++;
       strcpy(resource_path, argv[c]);
     } else if (! strcmp(argv[c], "-serial-device") ) {
       c++;
       serial_device=argv[c];
     } else if (! strcmp(argv[c], "-h") || ! strcmp(argv[c], "-help")) {
       printBanner();
       return 0;
     }
     c++;
   }
   if (! strlen(resource_path)) {
     getcwd(resource_path,PATH_SIZE);
   }
   cerr << "running with parameters" << endl;
   cerr << "resource_path: " << resource_path << endl;
   cerr << "serial_device: " << serial_device << endl;
   

   initCommandMap(&robot_connection,&run);
   

   robot_connection.connect(serial_device);
   if (! robot_connection.isConnected()){
     return 0;
   }
   bool result;
   cout << "querying system params ... ";
   result=robot_connection.queryParams(0);
   cout << result << endl;

   cout << "querying joint params ... ";
   result=robot_connection.queryParams(1);
   cout << result << endl;

   cout << "querying kinematic params ... ";
   result=robot_connection.queryParams(2);
   cout << result << endl;

  
   cerr <<  endl;
   cout << "READY" << endl;
  
   run = true;

   // server url will be http://localhost:9000
   int port = 9000;
   const char *interface = NULL;
   struct libwebsocket_context *context;
   // we're not using ssl
   //const char *cert_path = NULL;
   //const char *key_path = NULL;
   // no special options
   int opts = 0;
    
   struct lws_context_creation_info info;

   memset(&info, 0, sizeof info);
   info.port = port;
   info.iface = interface;
   info.protocols = protocols;
   info.extensions = libwebsocket_get_internal_extensions();
   //if (!use_ssl) {
   info.ssl_cert_filepath = NULL;
   info.ssl_private_key_filepath = NULL;
   //} else {
   //  info.ssl_cert_filepath = LOCAL_RESOURCE_PATH"/libwebsockets-test-server.pem";
   //  info.ssl_private_key_filepath = LOCAL_RESOURCE_PATH"/libwebsockets-test-server.key.pem";
   //}
   info.gid = -1;
   info.uid = -1;
   info.options = opts;

   context = libwebsocket_create_context(&info);    
   if (context == NULL) {
     fprintf(stderr, "libwebsocket init failed\n");
     return -1;
   }
    
   printf("starting websocket server...\n");



   // infinite loop, to end this server send SIGTERM. (CTRL+C)
   int count=0;
   while (run) {
     robot_connection.spinOnce();
     printResponsePacket(response_string,robot_connection.lastResponse());
     printSystemStatusPacket(system_status_string,robot_connection.systemStatus());
     printJointStatusPacket(joint_status_string,robot_connection.jointStatus());
     printKinematicsStatus(kinematic_status_string,robot_connection.kinematicsStatus());
     printSystemParamsPacket(system_params_string,robot_connection.systemParams());
     printJointParamsPacket(joint_params_string,robot_connection.jointParams());
     printKinematicsParamsPacket(kinematic_params_string,robot_connection.kinematicsParams());
 
     ostringstream os;
     switch(mode) {
     case System:
       os << "<p> <table border=\"3\">";
       os << "<tr>";
       os << "<th> System Status  </th>";
       os << "<th> System Params  </th>";
       os << "</tr>";
       os << "<tr>";
       os << "<td valign=\"top\">" << system_status_string << "</td>";
       os << "<td valign=\"top\">" << system_params_string << "</td>";
       os << "</tr>";
       os << "</table> </p>";
       os << "<p>" << response_string <<  "</p>" << endl;
       break;
     case Joints:
       os << "<p> <table border=\"3\">";
       os << "<tr>";
       os << "<th> Joint Status   </th>";
       os << "<th> Joint Params   </th>";
       os << "</tr>";
       os << "<tr>";
       os << "<td valign=\"top\">" << joint_status_string << "</td>";
       os << "<td valign=\"top\">" << joint_params_string << "</td>";
       os << "</tr>";
       os << "</table> </p>";
       os << "<p>" << response_string <<  "</p>" << endl;
       break;
     case Kinematics:
     case Joystick:
       os << "<p> <table border=\"3\">";
       os << "<tr>";
       os << "<th> Joint Status   </th>";
       os << "<th> Kinematic Status  </th>";
       os << "<th> Kinematic Params  </th>";
       os << "</tr>";
       os << "<tr>";
       os << "<td valign=\"top\">" << joint_status_string << "</td>";
       os << "<td valign=\"top\">" << kinematic_status_string << "</td>";
       os << "<td valign=\"top\">" << kinematic_params_string << "</td>";
       os << "</tr>";
       os << "</table> </p>";
       os << "<p>" << response_string <<  "</p>" << endl;
     }
     response_length=os.str().length();
     memcpy(response_begin, os.str().c_str(),response_length+1);
      
     for (WebSocketConnectionPtr wsi: client_connections){
       libwebsocket_write(wsi, response_begin, response_length, LWS_WRITE_TEXT);
     }

     libwebsocket_service(context, 1);
     count++;
   }
    
   libwebsocket_context_destroy(context);
    
   return 0;
 }
