cmake_minimum_required(VERSION 2.8)
project(multitarget_tracker_ros)

unset(CMAKE_CXX_FLAGS CACHE)

#find catkin packages
set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge  
  image_transport
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(OpenMP)
if (OPENMP_FOUND)
    list(APPEND CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra --std=gnu++14" CACHE STRING COMPILE_FLAGS FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native -mtune=native --fast-math -ffast-math -funroll-loops -Wall -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -march=native -mtune=native -Wall -DDEBUG" CACHE STRING COMPILE_FLAGS FORCE)
elseif (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /W4 -DGTL_STATIC" CACHE STRING COMPILE_FLAGS FORCE)
endif()


#set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# ----------------------------------------------------------------------------
# assume FindOpenCV.cmake file locate in CMAKE_MODULE_PATH 
# ----------------------------------------------------------------------------
set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)
#set(CMAKE_MODULE_PATH /usr/local/opencv3/share/OpenCV/OpenCVConfig.cmake)
FIND_PACKAGE(OpenCV 3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OpenCV
  INCLUDE_DIRS src
)

message(STATUS "=============${OpenCV_INCLUDE_DIRS}")
message(STATUS "--------------${OpenCV_LIBS}")

include_directories(src include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
		      src/Detector
              src/Detector/vibe_src
              src/Detector/Subsense
              src/Tracker
              src/Tracker/HungarianAlg
              src/Tracker/graph
              src/Tracker/graph/GTL/include)

# ----------------------------------------------------------------------------
# set source files
# ----------------------------------------------------------------------------
  set(folder_source
             src/Detector/BaseDetector.cpp
             src/Detector/MotionDetector.cpp
             src/Detector/BackgroundSubtract.cpp
             src/Detector/vibe_src/vibe.cpp
             src/Detector/Subsense/BackgroundSubtractorLBSP.cpp
             src/Detector/Subsense/BackgroundSubtractorLOBSTER.cpp
             src/Detector/Subsense/BackgroundSubtractorSuBSENSE.cpp
             src/Detector/Subsense/LBSP.cpp
             src/Detector/FaceDetector.cpp
             src/Detector/PedestrianDetector.cpp
             src/Detector/pedestrians/c4-pedestrian-detector.cpp
             src/Detector/SSDMobileNetDetector.cpp
             src/Detector/YoloDetector.cpp

             src/Tracker/Ctracker.cpp
             src/Tracker/track.cpp
             src/Tracker/HungarianAlg/HungarianAlg.cpp
             src/Tracker/LocalTracker.cpp
             src/Tracker/Kalman.cpp
)

  set(graph_source
             src/Tracker/graph/tokenise.cpp
#             Tracker/graph/script.cpp
#             Tracker/graph/rings.cpp
             src/Tracker/graph/mytree.cpp
             src/Tracker/graph/mygraph.cpp
             src/Tracker/graph/mwbmatching.cpp
             src/Tracker/graph/mincut.cpp
             src/Tracker/graph/gport.cpp
#             Tracker/graph/gml2nestedsql.cpp
#             Tracker/graph/gml2dot.cpp
#             Tracker/graph/components.cpp
             src/Tracker/graph/fheap.c
             )

  set(gtl_source
             src/Tracker/graph/GTL/src/bellman_ford.cpp
             src/Tracker/graph/GTL/src/bfs.cpp
             src/Tracker/graph/GTL/src/biconnectivity.cpp
             src/Tracker/graph/GTL/src/bid_dijkstra.cpp
             src/Tracker/graph/GTL/src/components.cpp
             src/Tracker/graph/GTL/src/debug.cpp
             src/Tracker/graph/GTL/src/dfs.cpp
             src/Tracker/graph/GTL/src/dijkstra.cpp
             src/Tracker/graph/GTL/src/edge.cpp
             src/Tracker/graph/GTL/src/embedding.cpp
             src/Tracker/graph/GTL/src/fm_partition.cpp
             src/Tracker/graph/GTL/src/gml_parser.cpp
             src/Tracker/graph/GTL/src/gml_scanner.cpp
             src/Tracker/graph/GTL/src/graph.cpp
             src/Tracker/graph/GTL/src/maxflow_ff.cpp
             src/Tracker/graph/GTL/src/maxflow_pp.cpp
             src/Tracker/graph/GTL/src/maxflow_sap.cpp
             src/Tracker/graph/GTL/src/min_tree.cpp
             src/Tracker/graph/GTL/src/node.cpp
             src/Tracker/graph/GTL/src/planarity.cpp
             src/Tracker/graph/GTL/src/pq_node.cpp
             src/Tracker/graph/GTL/src/pq_tree.cpp
             src/Tracker/graph/GTL/src/ratio_cut_partition.cpp
             src/Tracker/graph/GTL/src/st_number.cpp
             src/Tracker/graph/GTL/src/topsort.cpp
             )


  SOURCE_GROUP("Source Files" FILES ${folder_source})
  SOURCE_GROUP("Header Files" FILES ${folder_headers})

  SOURCE_GROUP("graph" FILES ${graph_source} ${graph_header})
  SOURCE_GROUP("GTL" FILES ${gtl_source} ${gtl_header})

include(CheckIncludeFileCXX)
check_include_file_cxx(opencv2/bgsegm.hpp HAVE_OPENCV_CONTRIB)
if(HAVE_OPENCV_CONTRIB)
    add_definitions(-DHAVE_OPENCV_CONTRIB)

    option(USE_OCV_BGFG "Should use the bgfg algorithms from opencv_contrib?" ON)
    option(USE_OCV_KCF "Should use the KCF tracker from opencv_contrib for collisions resolving?" ON)
    option(USE_OCV_UKF "Should use the Unscented Kalman Filterr from opencv_contrib?" ON)

else(HAVE_OPENCV_CONTRIB)

    option(USE_OCV_BGFG "Should use the bgfg algorithms from opencv_contrib?" OFF)
    option(USE_OCV_KCF "Should use the KCF tracker from opencv_contrib for collisions resolving?" OFF)
    option(USE_OCV_UKF "Should use the Unscented Kalman Filterr from opencv_contrib?" OFF)
endif(HAVE_OPENCV_CONTRIB)

if(USE_OCV_UKF)
    add_definitions(-DUSE_OCV_UKF)
endif(USE_OCV_UKF)

if(USE_OCV_KCF)
    add_definitions(-DUSE_OCV_KCF)
endif(USE_OCV_KCF)

if(USE_OCV_BGFG)
    add_definitions(-DUSE_OCV_BGFG)
endif(USE_OCV_BGFG)

# ----------------------------------------------------------------------------  
# Build
# ----------------------------------------------------------------------------
ADD_EXECUTABLE(multitarget_tracker_ros src/main.cpp ${folder_source} ${graph_source}  ${gtl_source})
TARGET_LINK_LIBRARIES(multitarget_tracker_ros ${OpenCV_LIBS})

ADD_EXECUTABLE(multitarget_tracker_node src/multitarget_tracker_node.cpp ${folder_source} ${graph_source}  ${gtl_source})
TARGET_LINK_LIBRARIES(multitarget_tracker_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})