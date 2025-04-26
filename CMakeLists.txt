cmake_minimum_required(VERSION 3.0.2)
project(heart_beat_proj)

add_compile_options(-std=c++17)

# Trova i pacchetti ROS necessari
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  geometry_msgs
  std_msgs
  tf
  tf2_ros
  tf2_eigen
  image_transport
  cv_bridge
)

# Trova Eigen3
find_package(Eigen3 REQUIRED)

# Configura il pacchetto catkin
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

# Includi le directory dei pacchetti richiesti
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  /home/corbe/wavelib/header# Percorso ai file header di WaveLib
  include
)

# Aggiungi le librerie da collegare
set(LIBRARIES
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  /home/corbe/wavelib/build/Bin/libwavelib.a# Percorso alla libreria statica di WaveLib
)

# Aggiungi l'eseguibile
add_executable(opticalflowpose src/main.cpp src/OpticalFlowPose.cpp src/OpticalFlow.cpp src/Triangulation.cpp src/Visualizer.cpp src/Feature_extractor.cpp src/Wavelets.cpp src/data.cpp)

# Collega le librerie necessarie all'eseguibile
target_link_libraries(opticalflowpose ${LIBRARIES})
