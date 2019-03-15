cmake_minimum_required(VERSION 2.8)
project(orbSLAMReconstructor)

# -------------------basic infomation -------------------------

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# -----------------------------Cmake Modules-------------------------
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# -------------------------- Library -------------------------------
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# pcl
find_package( PCL REQUIRED COMPONENT common io ) 
include_directories( ${PCL_INCLUDE_DIRS} ) 
add_definitions( ${PCL_DEFINITIONS} )

# mesh
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(CGAL REQUIRED)

# --------------------- include -----------------------------
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${GMPXX_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${CGAL_INCLUDE_DIRS}
external/glm/glm/
external/
include/mesh/core/
include/mesh/core/utilities/
include/mesh/example/
)

# -------------------- add library --------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc

src/mesh/core/CameraPointsCollection.cpp
src/mesh/core/ConfigParser.cpp
src/mesh/core/Delaunay3DCellInfo.cpp
src/mesh/core/Delaunay3DVertexInfo.cpp
src/mesh/core/ManifoldManager.cpp
src/mesh/core/OutputManager.cpp
src/mesh/core/ReconstructFromSfMData.cpp
src/mesh/core/ReconstructFromSLAMData.cpp
src/mesh/core/TriangulationManager.cpp

src/mesh/core/utilities/Chronometer.cpp
src/mesh/core/utilities/Logger.cpp
src/mesh/core/utilities/utilities.cpp
)

# -----------------------  link library ---------------------------
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so

${GMPXX_LIBRARIES}
${GMP_LIBRARIES}
${LAPACK_LIBRARIES}
${CGAL_LIBRARY}
${Boost_LIBRARIES}
${ASSIMP_LIBRARIES} 
${PCL_LIBRARIES}
)

# ---------------------- copy from mesh ------------------------

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable(stereo_mesh Examples/Stereo/stereo_mesh.cpp )
target_link_libraries(stereo_mesh ${PROJECT_NAME} )





