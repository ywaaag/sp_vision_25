add_library(auto_buff_fyt OBJECT
    rune_detector.cpp
    buff_detector.cpp
    buff_solver.cpp
)

target_link_libraries(auto_buff_fyt openvino::runtime)
