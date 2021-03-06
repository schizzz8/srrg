# Change these values to the right values for your chip
TARGET_CHIP := 33FJ128MC802
TARGET_FAMILY := dsPIC33F
# or
#TARGET_CHIP := 33EP512GP502
#TARGET_FAMILY := dsPIC33E
# or
#TARGET_CHIP := 24FJ32GB002
#TARGET_FAMILY := PIC24F
# or
#your own values

# Change this to match your XC16 and XC16++ version and path
XC16DIR := /opt/microchip/xc16/v1.25
#XC16DIR := "C:\Program Files (x86)\Microchip\xc16\v1.25"
#XC16DIR := /Applications/microchip/xc16/v1.25

# Example options for the C compiler
# Option -no-legacy-libc is only necessary for xc16 >= v1.25, older versions
# will not understand it and print a warning message, but it will not harm the
# compilation process
CFLAGS := -mcpu=$(TARGET_CHIP) -mno-eds-warn -no-legacy-libc

# Example options for the C++ compiler. Note that -fno-exceptions -fno-rtti
# are always required because this compiler does not support neither exceptions
# nor runtime type identification (RTTI). -D__bool_true_and_false_are_defined is
# only necessary if your code includes stdbool.h
CXXFLAGS := $(CFLAGS)  -fno-exceptions -fno-rtti -D__bool_true_and_false_are_defined -D_DSPIC_

LDSCRIPT := $(XC16DIR)/support/$(TARGET_FAMILY)/gld/p$(TARGET_CHIP).gld
LDFLAGS := --local-stack -p$(TARGET_CHIP) --script $(LDSCRIPT) --heap=512 -L$(XC16DIR)/lib -L$(XC16DIR)/lib/$(TARGET_FAMILY)
LIBS := -lc -lpic30 -lm -lp33FJ128MC802 # List any other library you need here (order matters)

.DEFAULT_GOAL := all
.PHONY: all clean

OBJS := Arduino.o \
comm_globals.o \
differential_drive_kinematics.o \
dspic_robot_main.o \
eeprom.o \
EEPROM.o \
encoder.o \
globals.o \
joint_controller.o \
motor.o \
packet_decoder.o \
packet_encoder.o \
dspic_settings.o

all: dspic_robot.hex

# Rule to compile C source files (using the official C compiler)
%.o: %.c
	$(XC16DIR)/bin/xc16-gcc $(CFLAGS) -c $< -o $@

# Rule to compile C++ source files (using the unofficial C++ compiler)
%.o: %.cpp
	$(XC16DIR)/bin/xc16-g++ $(CXXFLAGS) -c $< -o $@

# Special rule to compile minilibstdc++.cpp, which is located in the
# support-files directory
minilibstdc++.o: $(SUPPORTFILESDIR)/minilibstdc++.cpp
	$(XC16DIR)/bin/xc16-g++ $(CXXFLAGS) -c $< -o $@

# Rule to link together object files created through distinct compiler invocations
dspic_robot.elf: $(OBJS)
	$(XC16DIR)/bin/xc16-ld $(LDFLAGS) $^ $(LIBS) -o $@

# Rule to convert the dspic_roboting ELF file into a HEX file
dspic_robot.hex: dspic_robot.elf
	$(XC16DIR)/bin/xc16-bin2hex $<

clean:
	$(RM) dspic_robot.elf dspic_robot.hex $(OBJS)
