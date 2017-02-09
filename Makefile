# Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
# Qualcomm Technologies Proprietary and Confidential.

CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++

CFLAGS = -Wall -Wno-unused-variable -Wno-unused-but-set-parameter -Wno-unused-but-set-variable
CXXFLAGS = -Wall -Wno-unused-variable -Wno-unused-but-set-parameter -Wno-unused-but-set-variable -std=c++0x

LIB_DIR = ./lib

INCLUDES	= -I ./inc

###libadsprpc.so and libsensor_imu.so  in ./lib will compile warning, in local dir will work well-no warning

LIBS	=	libadsprpc.so \
				libsensor_imu.so \
				$(LIB_DIR)/libadreno_utils.so \
				$(LIB_DIR)/libcamera.so.0 \
				$(LIB_DIR)/libcamparams.so.0 \
				$(LIB_DIR)/libCB.so \
				$(LIB_DIR)/libcrypto.so.1.0.0 \
				$(LIB_DIR)/libEGL_adreno.so \
				$(LIB_DIR)/libGLESv2_adreno.so \
				$(LIB_DIR)/libgsl.so \
				$(LIB_DIR)/libmv.so \
				$(LIB_DIR)/libOpenCL.so \
				$(LIB_DIR)/libsc-a3xx.so \
				$(LIB_DIR)/libsnav_arm.so \
				$(LIB_DIR)/libssl.so.1.0.0 \
				$(LIB_DIR)/libstdc++.so.6 \
				$(LIB_DIR)/libz.so.1 \
				$(LIB_DIR)/libglib-2.0.so.0			
			 
TARGET = snav_proxy snav_test_receive_data_easy snav_waypoint_follow_ex
#TARGET = snav_proxy

all: $(TARGET)

% : %.cpp 
	$(CXX) $(CXXFLAGS) $< -o $@ $(INCLUDES) -lpthread   $(LIBS)	
	
% : %.c 
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDES) -lpthread   $(LIBS)

clean:
	rm -f $(TARGET)

