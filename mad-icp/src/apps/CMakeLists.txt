
add_executable(${PROJECT_NAME}_mad mad_icp_bag.cpp  )
target_link_libraries(${PROJECT_NAME}_mad 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         tools
                         odometry
                         ${third_party_libs}
                        )

