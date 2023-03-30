# Install script for directory: /home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/argos3/plugins/robots/kheperaiv/control_interface" TYPE FILE FILES
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_camera_sensor.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_encoder_sensor.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_ground_sensor.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_lidar_sensor.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_light_sensor.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_ultrasound_sensor.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_gripper_actuator.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_turret_actuator.h"
    "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/control_interface/ci_kheperaiv_turret_encoder_sensor.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/argos3/plugins/robots/kheperaiv/real_robot" TYPE FILE FILES "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/real_robot/main.cpp")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin__kheperaiv.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin__kheperaiv.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin__kheperaiv.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/argos3" TYPE SHARED_LIBRARY FILES "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/build_simulator/libargos3plugin__kheperaiv.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin__kheperaiv.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin__kheperaiv.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin__kheperaiv.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/midnightpegasus/Documents/WPI/MQP/TheCollectiveMQP/kheperaiv/build_simulator/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
