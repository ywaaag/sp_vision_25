add_library(auto_aim_ekfpnp OBJECT
    planner.cpp
    target.cpp
    tracker.cpp
)

target_link_libraries(auto_aim_ekfpnp
    PUBLIC
        auto_aim
        omniperception
        tools
        io
)
