
add_library(${PROJECT_NAME}.common
        timer/timer.cc
        io_utils.cc
        global_flags.cc
        )

target_link_libraries(${PROJECT_NAME}.common
        ${third_party_libs}
        )

