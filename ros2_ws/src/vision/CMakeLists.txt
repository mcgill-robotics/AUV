cmake_minimum_required(VERSION 3.8)

project(vision)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # C++ only: nvcc forwards these to its generated host code and -Wpedantic floods the log.
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall> $<$<COMPILE_LANGUAGE:CXX>:-Wextra> $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

set(ROS_DEPS
    rclcpp
    auv_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    vision_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)
set(CPP_LIBS
    OpenCV
    eigen3_cmake_module 
    Eigen3
)

# Find Dependencies
foreach(dep IN LISTS ROS_DEPS)
  find_package(${dep} REQUIRED)
endforeach()
foreach(dep IN LISTS CPP_LIBS)
  find_package(${dep} REQUIRED)
endforeach()


# ---Object Map library---
add_library(object_map_lib SHARED
    ${PROJECT_NAME}/object_map/object_tracker.cpp
    ${PROJECT_NAME}/external/hungarian.cpp
    ${PROJECT_NAME}/external/kalman.cpp
)


target_include_directories(object_map_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(object_map_lib ${CPP_LIBS})

# ---Detection geometry library (Snell's law + depth modes, CUDA-free)---
add_library(detection_geometry_lib SHARED
    ${PROJECT_NAME}/object_detection/detection_geometry.cpp
)
target_include_directories(detection_geometry_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(detection_geometry_lib ${CPP_LIBS})

# ---RF-DETR TensorRT engine (only on machines with CUDA + TensorRT)---
# CPU-only dev containers skip this block so `colcon build` keeps working there.
include(CheckLanguage)
check_language(CUDA)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/rf-detr-cpp)
if(CMAKE_CUDA_COMPILER)
  find_package(TensorRT QUIET)
endif()

if(CMAKE_CUDA_COMPILER AND TensorRT_FOUND)
  set(VISION_HAS_RFDETR ON)
  # rf-detr-cpp's default arch list includes sm_101, which CUDA 13 renamed to sm_110.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
      set(CMAKE_CUDA_ARCHITECTURES 87)  # Jetson Orin
    else()
      set(CMAKE_CUDA_ARCHITECTURES "75;86;89;120")
    endif()
  endif()
  set(RFDETR_BUILD_SHARED OFF CACHE BOOL "" FORCE)
  set(RFDETR_BUILD_APPS ON CACHE BOOL "" FORCE)  # rfdetr_build compiles ONNX -> engine on the target GPU
  add_subdirectory(external/rf-detr-cpp EXCLUDE_FROM_ALL)
  add_custom_target(vision_rfdetr_tools ALL DEPENDS rfdetr_build)
  # Third-party code: silence its warnings (deprecated TensorRT 10 APIs etc.) so ours stay visible.
  target_compile_options(rfdetr PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-w>)
  target_compile_options(rfdetr_build PRIVATE -w)

  add_library(rfdetr_engine_lib SHARED
      ${PROJECT_NAME}/object_detection/rfdetr_engine.cpp
  )
  target_include_directories(rfdetr_engine_lib PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_link_libraries(rfdetr_engine_lib PUBLIC detection_geometry_lib PRIVATE rfdetr::rfdetr)

  add_executable(rfdetr_detect_image ${PROJECT_NAME}/object_detection/rfdetr_detect_image.cpp)
  target_link_libraries(rfdetr_detect_image rfdetr_engine_lib ${OpenCV_LIBS})

  # Front camera node. Without the ZED SDK it still builds, but only supports has_zed_sdk:=false.
  find_package(cv_bridge REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ZED 5 QUIET)
  find_package(CUDAToolkit REQUIRED)  # CUDA::cudart; rf-detr-cpp's own find is scoped to its subdirectory

  add_executable(front_cam_object_detection
      ${PROJECT_NAME}/front_cam_object_detection_main.cpp
      ${PROJECT_NAME}/object_detection/front_cam_object_detection.cpp
  )
  target_include_directories(front_cam_object_detection PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  )
  target_link_libraries(front_cam_object_detection rfdetr_engine_lib detection_geometry_lib)
  ament_target_dependencies(front_cam_object_detection ${ROS_DEPS} ${CPP_LIBS} cv_bridge ament_index_cpp)
  if(ZED_FOUND)
    target_compile_definitions(front_cam_object_detection PRIVATE ZED_SDK_AVAILABLE)
    target_include_directories(front_cam_object_detection SYSTEM PRIVATE ${ZED_INCLUDE_DIRS})
    target_link_directories(front_cam_object_detection PRIVATE ${ZED_LIBRARY_DIR})
    target_link_libraries(front_cam_object_detection ${ZED_LIBRARIES} CUDA::cudart)
    # libsl_zed.so needs Jetson driver libs (libcuda, Argus, nvbufsurface) that only exist on the device;
    # the CI image lacks them, so let them resolve at runtime.
    target_link_options(front_cam_object_detection PRIVATE "LINKER:--allow-shlib-undefined")
    message(STATUS "vision: front_cam_object_detection built with ZED SDK ${ZED_VERSION}")
  else()
    message(STATUS "vision: ZED SDK not found, front_cam_object_detection supports has_zed_sdk:=false only")
  endif()
  message(STATUS "vision: RF-DETR C++ detection enabled (CUDA archs: ${CMAKE_CUDA_ARCHITECTURES})")
else()
  set(VISION_HAS_RFDETR OFF)
  message(STATUS "vision: CUDA/TensorRT not found, skipping RF-DETR C++ detection nodes")
endif()

# ---ROS2 nodes---
# C++ nodes
add_executable(object_map 
    ${PROJECT_NAME}/object_map/object_map.cpp
    include/${PROJECT_NAME}/object_map.hpp
    ${PROJECT_NAME}/object_map_main.cpp
)

target_include_directories(object_map PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_map object_map_lib)
ament_target_dependencies(object_map ${ROS_DEPS} ${CPP_LIBS})

# Python nodes
set (PYTHON_NODE_SOURCES
    ${PROJECT_NAME}/down_image_enhancement.py
    ${PROJECT_NAME}/down_cam_object_detection.py
    ${PROJECT_NAME}/front_image_enhancement.py
    ${PROJECT_NAME}/front_cam_object_detection.py
)

# --- Installation ---
# Object Map static library
install(TARGETS object_map_lib detection_geometry_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
if(VISION_HAS_RFDETR)
  install(TARGETS rfdetr_engine_lib
      LIBRARY DESTINATION lib
  )
  install(TARGETS rfdetr_build rfdetr_detect_image front_cam_object_detection DESTINATION lib/${PROJECT_NAME})
endif()
# Object Map node executable
install(TARGETS object_map 
    DESTINATION lib/${PROJECT_NAME}
)
# Python nodes
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ${PYTHON_NODE_SOURCES} DESTINATION lib/${PROJECT_NAME})
# Launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# configuration files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
# Model files: Compute relative path from install share to source models folder, and automatically symlink it
file(RELATIVE_PATH REL_PATH_TO_MODELS 
     "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}" 
     "${CMAKE_CURRENT_SOURCE_DIR}/models")

install(CODE "
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
  file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models\")
  file(CREATE_LINK \"${REL_PATH_TO_MODELS}\" \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models\" SYMBOLIC)
  message(STATUS \"Created relative symlink for models: share/${PROJECT_NAME}/models -> ${REL_PATH_TO_MODELS}\")
")

# Headers
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detection_geometry test/test_detection_geometry.cpp)
  target_link_libraries(test_detection_geometry detection_geometry_lib)
endif()

ament_package()
