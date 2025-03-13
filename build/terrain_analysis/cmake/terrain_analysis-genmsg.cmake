# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "terrain_analysis: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iterrain_analysis:/home/zf/pc_analysis/src/terrain_analysis/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(terrain_analysis_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg" NAME_WE)
add_custom_target(_terrain_analysis_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "terrain_analysis" "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg" "terrain_analysis/CustomPoint:std_msgs/Header"
)

get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg" NAME_WE)
add_custom_target(_terrain_analysis_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "terrain_analysis" "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/terrain_analysis
)
_generate_msg_cpp(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/terrain_analysis
)

### Generating Services

### Generating Module File
_generate_module_cpp(terrain_analysis
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/terrain_analysis
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(terrain_analysis_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(terrain_analysis_generate_messages terrain_analysis_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_cpp _terrain_analysis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_cpp _terrain_analysis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(terrain_analysis_gencpp)
add_dependencies(terrain_analysis_gencpp terrain_analysis_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS terrain_analysis_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/terrain_analysis
)
_generate_msg_eus(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/terrain_analysis
)

### Generating Services

### Generating Module File
_generate_module_eus(terrain_analysis
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/terrain_analysis
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(terrain_analysis_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(terrain_analysis_generate_messages terrain_analysis_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_eus _terrain_analysis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_eus _terrain_analysis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(terrain_analysis_geneus)
add_dependencies(terrain_analysis_geneus terrain_analysis_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS terrain_analysis_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/terrain_analysis
)
_generate_msg_lisp(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/terrain_analysis
)

### Generating Services

### Generating Module File
_generate_module_lisp(terrain_analysis
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/terrain_analysis
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(terrain_analysis_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(terrain_analysis_generate_messages terrain_analysis_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_lisp _terrain_analysis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_lisp _terrain_analysis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(terrain_analysis_genlisp)
add_dependencies(terrain_analysis_genlisp terrain_analysis_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS terrain_analysis_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/terrain_analysis
)
_generate_msg_nodejs(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/terrain_analysis
)

### Generating Services

### Generating Module File
_generate_module_nodejs(terrain_analysis
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/terrain_analysis
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(terrain_analysis_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(terrain_analysis_generate_messages terrain_analysis_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_nodejs _terrain_analysis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_nodejs _terrain_analysis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(terrain_analysis_gennodejs)
add_dependencies(terrain_analysis_gennodejs terrain_analysis_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS terrain_analysis_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/terrain_analysis
)
_generate_msg_py(terrain_analysis
  "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/terrain_analysis
)

### Generating Services

### Generating Module File
_generate_module_py(terrain_analysis
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/terrain_analysis
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(terrain_analysis_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(terrain_analysis_generate_messages terrain_analysis_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomMsg.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_py _terrain_analysis_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zf/pc_analysis/src/terrain_analysis/msg/CustomPoint.msg" NAME_WE)
add_dependencies(terrain_analysis_generate_messages_py _terrain_analysis_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(terrain_analysis_genpy)
add_dependencies(terrain_analysis_genpy terrain_analysis_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS terrain_analysis_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/terrain_analysis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/terrain_analysis
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(terrain_analysis_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(terrain_analysis_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(terrain_analysis_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/terrain_analysis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/terrain_analysis
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(terrain_analysis_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(terrain_analysis_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(terrain_analysis_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/terrain_analysis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/terrain_analysis
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(terrain_analysis_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(terrain_analysis_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(terrain_analysis_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/terrain_analysis)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/terrain_analysis
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(terrain_analysis_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(terrain_analysis_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(terrain_analysis_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/terrain_analysis)
  install(CODE "execute_process(COMMAND \"/home/zf/anaconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/terrain_analysis\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/terrain_analysis
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(terrain_analysis_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(terrain_analysis_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(terrain_analysis_generate_messages_py sensor_msgs_generate_messages_py)
endif()
