cmake_minimum_required(VERSION 2.8.3)
project(orne_waypoints_editor_rviz_plugins)
# set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11")
# set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2")
# set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2")
# set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2")
find_package(catkin REQUIRED COMPONENTS rviz std_srvs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/save_trigger_panel.h
)

set(SOURCE_FILES
  src/save_trigger_panel.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

