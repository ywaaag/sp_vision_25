cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

find_path(
    TENSORRT_INCLUDE_DIR
    NAMES NvInfer.h
    HINTS /usr/include /usr/include/x86_64-linux-gnu /usr/local/include
)
find_library(
    TENSORRT_NVINFER_LIBRARY
    NAMES nvinfer
    HINTS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
)
find_library(
    TENSORRT_NVINFER_PLUGIN_LIBRARY
    NAMES nvinfer_plugin
    HINTS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
)
find_library(
    TENSORRT_CUDART_LIBRARY
    NAMES cudart
    HINTS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/cuda/lib64
)
find_path(
    CUDA_RUNTIME_INCLUDE_DIR
    NAMES cuda_runtime_api.h
    HINTS /usr/include /usr/local/cuda/include
)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov5_trt.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)

if(TENSORRT_INCLUDE_DIR AND CUDA_RUNTIME_INCLUDE_DIR AND TENSORRT_NVINFER_LIBRARY AND TENSORRT_CUDART_LIBRARY)
    message(STATUS "TensorRT found, enabling YOLOV5TRT backend")
    target_compile_definitions(auto_aim PRIVATE AUTO_AIM_ENABLE_TENSORRT=1)
    target_include_directories(auto_aim PRIVATE ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
    target_link_libraries(auto_aim ${TENSORRT_NVINFER_LIBRARY} ${TENSORRT_CUDART_LIBRARY})
    if(TENSORRT_NVINFER_PLUGIN_LIBRARY)
        target_link_libraries(auto_aim ${TENSORRT_NVINFER_PLUGIN_LIBRARY})
    endif()
else()
    message(WARNING "TensorRT not found, YOLOV5TRT backend will throw at runtime if selected")
endif()
