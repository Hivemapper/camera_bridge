cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(nlohmann_json REQUIRED)

add_custom_target(VersionCpp ${CMAKE_COMMAND} -DVERSION_SHA=${VERSION_SHA} -P ${CMAKE_CURRENT_LIST_DIR}/version.cmake)
set_source_files_properties(version.cpp PROPERTIES GENERATED 1)

add_library(libcamera_app libcamera_app.cpp post_processor.cpp version.cpp options.cpp)
add_dependencies(libcamera_app VersionCpp)

set_target_properties(libcamera_app PROPERTIES PREFIX "" IMPORT_PREFIX "")
target_link_libraries(libcamera_app pthread ${LIBCAMERA_LINK_LIBRARIES} ${Boost_LIBRARIES} ${nlohmann_json_LIBRARIES} post_processing_stages)

install(TARGETS libcamera_app LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
