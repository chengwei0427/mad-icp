add_executable(${PROJECT_NAME} mad_icp.cpp  )
target_link_libraries(${PROJECT_NAME} 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         tools
                         odometry
                         ${third_party_libs}
                        )

add_executable(bin_runner bin_runner.cpp)
target_link_libraries(bin_runner
            odometry
            yaml-cpp
            )                      
