add_executable(hw3cpp hw3.cpp)
message(${OpenRAVE_LIBRARIES})
target_link_libraries(hw3cpp ${OpenRAVE_LIBRARIES} rrtconnect)
set_target_properties(hw3cpp PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
