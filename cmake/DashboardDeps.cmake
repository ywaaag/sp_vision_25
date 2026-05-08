# Optional dependency probe for the MQTT Dashboard C++ bridge.
# This file must not make dashboard dependencies required.

find_package(OpenSSL QUIET)
if(OpenSSL_FOUND)
    find_package(PahoMqttCpp QUIET)
else()
    set(PahoMqttCpp_FOUND OFF)
endif()

if(PahoMqttCpp_FOUND)
    message(STATUS "Dashboard optional dependency: PahoMqttCpp found.")
else()
    message(STATUS "Dashboard optional dependency: PahoMqttCpp not found; future MqttBridge targets will need it.")
endif()
