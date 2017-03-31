CXX = g++
CC = gcc
CXXOPTS = -Wall -O3 --std=gnu++11 -I.

#uncomment the line below for ubuntu 16.04
#CXXOPTS+=__UBUNTU_16_04__

CCOPTS = -Wall -O3  --std=gnu99 -I.
LIBS = -lwebsockets -lncurses
OBJS = 	packet_decoder.o \
	packet_encoder.o \
	command_parser.o \
	differential_drive_kinematics.o \
	joint_controller.o \
	orazio_robot_connection.o \

COBJS =	serial.o

APPS= 	orazio_robot_monitor \
	orazio_robot_shell \
	orazio_robot_websocket_server \

.phony:	clean all

all:	$(APPS)

clean:	
	rm -rf *~ *.o $(APPS)

%.o:	%.cpp *.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:	%.c *.h
	$(CC) $(CXXFLAGS) -c -o $@ $<

orazio_robot_monitor:	orazio_robot_monitor.cpp $(OBJS) $(COBJS)
	$(CXX) -o $@ $(CXXOPTS) $^ $(LIBS) -lpthread

orazio_robot_shell:	orazio_robot_shell.cpp $(OBJS) $(COBJS)
	$(CXX) -o $@ $(CXXOPTS) $^ $(LIBS) -lpthread

orazio_robot_websocket_server:	orazio_robot_websocket_server.cpp $(OBJS) $(COBJS)
	$(CXX) -o $@ $(CXXOPTS) $^ $(LIBS)

robot_program:	robot_program.cpp $(OBJS) $(COBJS)
	$(CXX) -o $@ $(CXXOPTS) $^ $(LIBS)
