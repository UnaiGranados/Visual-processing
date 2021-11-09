# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tf_helper_functions: 0 messages, 2 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tf_helper_functions_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv" NAME_WE)
add_custom_target(_tf_helper_functions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf_helper_functions" "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv" "geometry_msgs/Vector3:geometry_msgs/TransformStamped:geometry_msgs/Transform:geometry_msgs/Quaternion:std_msgs/Header"
)

get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv" NAME_WE)
add_custom_target(_tf_helper_functions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf_helper_functions" "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv" "geometry_msgs/Vector3:geometry_msgs/TransformStamped:geometry_msgs/Transform:geometry_msgs/Quaternion:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf_helper_functions
)
_generate_srv_cpp(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf_helper_functions
)

### Generating Module File
_generate_module_cpp(tf_helper_functions
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf_helper_functions
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tf_helper_functions_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tf_helper_functions_generate_messages tf_helper_functions_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_cpp _tf_helper_functions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_cpp _tf_helper_functions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf_helper_functions_gencpp)
add_dependencies(tf_helper_functions_gencpp tf_helper_functions_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf_helper_functions_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf_helper_functions
)
_generate_srv_eus(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf_helper_functions
)

### Generating Module File
_generate_module_eus(tf_helper_functions
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf_helper_functions
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tf_helper_functions_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tf_helper_functions_generate_messages tf_helper_functions_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_eus _tf_helper_functions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_eus _tf_helper_functions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf_helper_functions_geneus)
add_dependencies(tf_helper_functions_geneus tf_helper_functions_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf_helper_functions_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf_helper_functions
)
_generate_srv_lisp(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf_helper_functions
)

### Generating Module File
_generate_module_lisp(tf_helper_functions
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf_helper_functions
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tf_helper_functions_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tf_helper_functions_generate_messages tf_helper_functions_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_lisp _tf_helper_functions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_lisp _tf_helper_functions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf_helper_functions_genlisp)
add_dependencies(tf_helper_functions_genlisp tf_helper_functions_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf_helper_functions_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf_helper_functions
)
_generate_srv_nodejs(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf_helper_functions
)

### Generating Module File
_generate_module_nodejs(tf_helper_functions
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf_helper_functions
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tf_helper_functions_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tf_helper_functions_generate_messages tf_helper_functions_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_nodejs _tf_helper_functions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_nodejs _tf_helper_functions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf_helper_functions_gennodejs)
add_dependencies(tf_helper_functions_gennodejs tf_helper_functions_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf_helper_functions_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions
)
_generate_srv_py(tf_helper_functions
  "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions
)

### Generating Module File
_generate_module_py(tf_helper_functions
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tf_helper_functions_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tf_helper_functions_generate_messages tf_helper_functions_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/StaticTransformPublisher.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_py _tf_helper_functions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tecnalia/workspace/fanuc_3D_cam_ws/src/algebra_libraries/tf_helper_functions/srv/TFEcho.srv" NAME_WE)
add_dependencies(tf_helper_functions_generate_messages_py _tf_helper_functions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf_helper_functions_genpy)
add_dependencies(tf_helper_functions_genpy tf_helper_functions_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf_helper_functions_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf_helper_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf_helper_functions
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tf_helper_functions_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf_helper_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf_helper_functions
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tf_helper_functions_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf_helper_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf_helper_functions
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tf_helper_functions_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf_helper_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf_helper_functions
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tf_helper_functions_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf_helper_functions/.+/__init__.pyc?$"
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tf_helper_functions_generate_messages_py geometry_msgs_generate_messages_py)
endif()
