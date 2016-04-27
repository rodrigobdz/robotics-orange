# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "create_fundamentals: 1 messages, 5 services")

set(MSG_I_FLAGS "-Icreate_fundamentals:/home/orange/catkin_ws/src/create_fundamentals/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(create_fundamentals_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg" NAME_WE)
add_custom_target(_create_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "create_fundamentals" "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv" NAME_WE)
add_custom_target(_create_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "create_fundamentals" "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv" ""
)

get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv" NAME_WE)
add_custom_target(_create_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "create_fundamentals" "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv" ""
)

get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv" NAME_WE)
add_custom_target(_create_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "create_fundamentals" "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv" ""
)

get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv" NAME_WE)
add_custom_target(_create_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "create_fundamentals" "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv" ""
)

get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv" NAME_WE)
add_custom_target(_create_fundamentals_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "create_fundamentals" "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
)

### Generating Services
_generate_srv_cpp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_cpp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_cpp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_cpp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_cpp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
)

### Generating Module File
_generate_module_cpp(create_fundamentals
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(create_fundamentals_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(create_fundamentals_generate_messages create_fundamentals_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_cpp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_cpp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_cpp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_cpp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_cpp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_cpp _create_fundamentals_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(create_fundamentals_gencpp)
add_dependencies(create_fundamentals_gencpp create_fundamentals_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS create_fundamentals_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
)

### Generating Services
_generate_srv_lisp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_lisp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_lisp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_lisp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
)
_generate_srv_lisp(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
)

### Generating Module File
_generate_module_lisp(create_fundamentals
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(create_fundamentals_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(create_fundamentals_generate_messages create_fundamentals_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_lisp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_lisp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_lisp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_lisp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_lisp _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_lisp _create_fundamentals_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(create_fundamentals_genlisp)
add_dependencies(create_fundamentals_genlisp create_fundamentals_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS create_fundamentals_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
)

### Generating Services
_generate_srv_py(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
)
_generate_srv_py(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
)
_generate_srv_py(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
)
_generate_srv_py(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
)
_generate_srv_py(create_fundamentals
  "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
)

### Generating Module File
_generate_module_py(create_fundamentals
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(create_fundamentals_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(create_fundamentals_generate_messages create_fundamentals_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/msg/SensorPacket.msg" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_py _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/Leds.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_py _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/PlaySong.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_py _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/DiffDrive.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_py _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/ResetEncoders.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_py _create_fundamentals_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/orange/catkin_ws/src/create_fundamentals/srv/StoreSong.srv" NAME_WE)
add_dependencies(create_fundamentals_generate_messages_py _create_fundamentals_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(create_fundamentals_genpy)
add_dependencies(create_fundamentals_genpy create_fundamentals_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS create_fundamentals_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/create_fundamentals
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(create_fundamentals_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(create_fundamentals_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/create_fundamentals
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(create_fundamentals_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(create_fundamentals_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/create_fundamentals
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(create_fundamentals_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(create_fundamentals_generate_messages_py sensor_msgs_generate_messages_py)
