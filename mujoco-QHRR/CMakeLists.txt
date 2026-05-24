cmake_minimum_required(VERSION 3.16)
project(QHRR0 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco)
set(ONNX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime)

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

add_executable(daemon
    app/daemon/main.cpp
    app/daemon/daemon.cpp
)
target_include_directories(daemon PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(daemon PRIVATE
    yaml-cpp
    pthread
)

add_executable(task_controller
    app/task_controller/main.cpp
    app/task_controller/task_controller.cpp
    app/task_controller/onnx_runner.cpp
    app/task_controller/rnn_onnx_runner.cpp
)
target_include_directories(task_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ONNX_DIR}/include
)
target_link_directories(task_controller PRIVATE
    ${ONNX_DIR}/lib
)
target_link_libraries(task_controller PRIVATE
    yaml-cpp
    Eigen3::Eigen
    onnxruntime
    pthread
    dl
    m
)
set_target_properties(task_controller PROPERTIES
    BUILD_RPATH "${ONNX_DIR}/lib"
)

add_executable(aux_reader
    app/aux_reader/main.cpp
    app/aux_reader/joystick.cpp
)
target_include_directories(aux_reader PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(aux_reader PRIVATE
    pthread
)

add_executable(shm_logger
    app/shm_logger/main.cpp
)
target_include_directories(shm_logger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(shm_logger PRIVATE
    pthread
)

add_executable(mujoco_simulate
    third_party/mujoco/simulate/main.cc
    third_party/mujoco/simulate/glfw_adapter.cc
    third_party/mujoco/simulate/glfw_dispatch.cc
    third_party/mujoco/simulate/platform_ui_adapter.cc
    third_party/mujoco/simulate/simulate.cc
    third_party/mujoco/simulate/lodepng.cpp
    third_party/mujoco/simulate/mujoco_can/mujoco_can_bridge.cc
    third_party/mujoco/simulate/mujoco_can/socket_can_adapter.cc
    third_party/mujoco/simulate/mujoco_can/actuator_firmware_base.cc
    third_party/mujoco/simulate/mujoco_can/spg_firmware.cc
    third_party/mujoco/simulate/mujoco_can/e2box_imu_firmware.cc
)
target_include_directories(mujoco_simulate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_DIR}/include
    ${MUJOCO_DIR}/simulate
    ${MUJOCO_DIR}/simulate/mujoco_can
    ${GLFW_INCLUDE_DIRS}
)
target_link_directories(mujoco_simulate PRIVATE
    ${MUJOCO_DIR}/lib
)
target_link_libraries(mujoco_simulate PRIVATE
    ${GLFW_LIBRARIES}
    mujoco
    yaml-cpp
    GL
    pthread
    dl
    m
)
set_target_properties(mujoco_simulate PROPERTIES
    BUILD_RPATH "${MUJOCO_DIR}/lib"
)
target_compile_definitions(mujoco_simulate PRIVATE
    MUJOCO_SIM_CONFIG_DIR="${MUJOCO_DIR}/simulate/configs"
)