PROJECT(voxelizer)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_C_FLAGS "-Wall -O3 -g")
set(CMAKE_CXX_FLAGS "-Wall -O3 -g")

find_package(OpenGL REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Gui REQUIRED) 
find_package(Qt5OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

find_package(catkin COMPONENTS 
     glow)

if(catkin_FOUND)
  message("Found catkin")
  catkin_package(
    CATKIN_DEPENDS
      glow
    DEPENDS
      Boost
  )
endif()

include_directories(${catkin_INCLUDE_DIRS} src/ ${QT5_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} /usr/include/eigen3 /usr/local/include)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # needs to be activated for qt generated files in build directory.
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3  -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-UDEBUG_GL -UDEBUG -DNDEBUG -g2 ${CMAKE_CXX_FLAGS}")

# since now everything resides in "bin", we have to copy some stuff.
  

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

configure_file(assets/labels.xml ${CMAKE_SOURCE_DIR}/bin/labels.xml COPYONLY)
configure_file(assets/settings.cfg ${CMAKE_SOURCE_DIR}/bin/settings.cfg COPYONLY)

QT5_WRAP_UI(UI_HDRS 
  src/VoxelizerMainFrame.ui)

set(VIZ_SHADER_SRC ${CMAKE_BINARY_DIR}/visualization_shaders.cpp)

COMPILE_SHADERS(${VIZ_SHADER_SRC}
  src/shaders/color.glsl
  
  src/shaders/empty.vert
  src/shaders/empty.frag
  src/shaders/quad.geom
  src/shaders/passthrough.frag
  
  src/shaders/draw_pose.geom
  src/shaders/draw_voxels.vert
  src/shaders/draw_voxels.geom
  src/shaders/draw_points.vert
  src/shaders/blinnphong.frag
)

add_executable(voxelizer 
  ${UI_HDRS}
  ${VIZ_SHADER_SRC}
  
  src/data/label_utils.cpp
  src/data/kitti_utils.cpp
  src/data/VoxelGrid.cpp
  src/rv/string_utils.cpp
  src/rv/Stopwatch.cpp
  src/data/misc.cpp
  src/voxelizer.cpp
  src/widget/Mainframe.h
  src/widget/Viewport.h
  src/widget/Mainframe.cpp
  src/widget/Viewport.cpp
  src/widget/KittiReader.cpp
  src/data/voxelize_utils.cpp
)

add_executable(gen_data 
  src/data/label_utils.cpp
  src/data/kitti_utils.cpp
  src/data/VoxelGrid.cpp
  src/rv/string_utils.cpp
  src/rv/Stopwatch.cpp
  src/data/misc.cpp
  src/gen_data.cpp

  src/widget/KittiReader.cpp
  src/data/voxelize_utils.cpp
)

target_link_libraries(voxelizer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} pthread Qt5::Xml Qt5::OpenGL Qt5::Widgets matio)
target_link_libraries(gen_data ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} pthread Qt5::Xml Qt5::OpenGL Qt5::Widgets matio)

