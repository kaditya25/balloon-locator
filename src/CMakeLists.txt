set(TARGET lab5)

find_package( OpenCV 4.0.1 REQUIRED )
find_package (Eigen3 3.3 NO_MODULE)

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( ${TARGET} main.cc )
target_link_libraries( ${TARGET} ${OpenCV_LIBS} Eigen3::Eigen)

set_target_properties( ${TARGET}
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/exe"
)

