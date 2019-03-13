cmake_minimum_required (VERSION 2.6.0)
project(rrtconnect)
find_package(OpenRAVE 0.9 REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(rrtconnect SHARED rrtconnect.cpp)
set_target_properties(rrtconnect PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(rrtconnect ${OpenRAVE_LIBRARIES})
install(TARGETS rrtconnect DESTINATION .)
