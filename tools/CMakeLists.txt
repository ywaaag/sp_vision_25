cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
    check_sum.cpp
)

target_link_libraries(tools PUBLIC yaml-cpp)

if(PahoMqttCpp_FOUND)
    add_library(mqtt_bridge STATIC
        mqtt_bridge.cpp
    )
    target_include_directories(mqtt_bridge PUBLIC
        ${PROJECT_SOURCE_DIR}
    )
    target_link_libraries(mqtt_bridge PUBLIC
        nlohmann_json::nlohmann_json
    )

    if(TARGET PahoMqttCpp::paho-mqttpp3)
        target_link_libraries(mqtt_bridge PUBLIC PahoMqttCpp::paho-mqttpp3)
    elseif(TARGET paho-mqttpp3)
        target_link_libraries(mqtt_bridge PUBLIC paho-mqttpp3)
    else()
        target_link_libraries(mqtt_bridge PUBLIC paho-mqttpp3 paho-mqtt3as)
    endif()
endif()
