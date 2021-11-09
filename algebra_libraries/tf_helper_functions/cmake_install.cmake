# Install script for directory: /home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions

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
  include("/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/tf_helper_functions/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf_helper_functions/srv" TYPE FILE FILES
    "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv"
    "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf_helper_functions/cmake" TYPE FILE FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/tf_helper_functions/catkin_generated/installspace/tf_helper_functions-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/include/tf_helper_functions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/share/roseus/ros/tf_helper_functions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/share/common-lisp/ros/tf_helper_functions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/share/gennodejs/ros/tf_helper_functions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/lib/python2.7/dist-packages/tf_helper_functions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/lib/python2.7/dist-packages/tf_helper_functions" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/lib/python2.7/dist-packages/tf_helper_functions" FILES_MATCHING REGEX "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/devel/lib/python2.7/dist-packages/tf_helper_functions/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/tf_helper_functions/catkin_generated/installspace/tf_helper_functions.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf_helper_functions/cmake" TYPE FILE FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/tf_helper_functions/catkin_generated/installspace/tf_helper_functions-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf_helper_functions/cmake" TYPE FILE FILES
    "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/tf_helper_functions/catkin_generated/installspace/tf_helper_functionsConfig.cmake"
    "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/tf_helper_functions/catkin_generated/installspace/tf_helper_functionsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf_helper_functions" TYPE FILE FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tf_helper_functions" TYPE PROGRAM FILES "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/visual_servoing/algebra_libraries/tf_helper_functions/catkin_generated/installspace/tf_listener_broadcaster_node.py")
endif()

