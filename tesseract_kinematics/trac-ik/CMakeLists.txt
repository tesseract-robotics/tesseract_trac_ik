find_package(trac_ik_lib REQUIRED)
# When integrating into tesseract_kinematics remove next line
find_package(tesseract_kinematics REQUIRED)

# When integrating into tesseract_kinematics add this to tesseract_kinematics-config.cmake.in
# if(@TESSERACT_BUILD_TRACIK@)
#   find_dependency(trac_ik_lib)
# endif()
# And add this to package.xml
# <depend>trac_ik_lib</depend>

add_library(${PROJECT_NAME}_trac-ik src/trac-ik_inv_kin_chain.cpp)
target_link_libraries(
  ${PROJECT_NAME}_trac-ik
  # When integrating into tesseract_kinematics restore lines below and remove tesseract_kinematics_core and
  # tesseract_kinematics_kdl PUBLIC ${PROJECT_NAME}_core ${PROJECT_NAME}_kdl
  PUBLIC tesseract::tesseract_kinematics_core
         tesseract::tesseract_kinematics_kdl
         Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_scene_graph
         trac_ik_lib::trac_ik
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_trac-ik PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trac-ik PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trac-ik PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trac-ik ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trac-ik PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_trac-ik
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_trac-ik PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Add trac-ik kinematics factory
add_library(${PROJECT_NAME}_trac-ik_factory src/trac-ik_factory.cpp)
target_link_libraries(
  ${PROJECT_NAME}_trac-ik_factory
  PUBLIC ${PROJECT_NAME}_trac-ik
         tesseract::tesseract_scene_graph
         tesseract::tesseract_common
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_trac-ik_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trac-ik_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trac-ik_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trac-ik_factory ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trac-ik_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_trac-ik_factory
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_trac-ik_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                         "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find this factory by default
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "${PROJECT_NAME}_trac-ik_factory" PARENT_SCOPE)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT trac-ik
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

configure_component(
  COMPONENT trac-ik
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_trac-ik ${PROJECT_NAME}_trac-ik_factory
  DEPENDENCIES trac_ik_lib "tesseract_kinematics COMPONENTS core")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT trac-ik
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics Trac-IK implementation"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "trac_ik_lib"
    WINDOWS_DEPENDS "trac_ik_lib")
endif()
