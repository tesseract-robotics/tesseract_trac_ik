find_package(
  tesseract REQUIRED
  COMPONENTS common
             kinematics
             kinematics_kdl
             scene_graph
             state_solver
             urdf)
find_package(Eigen3 REQUIRED)

find_gtest()

macro(add_trac_ik_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(
    ${test_name}
    PRIVATE GTest::GTest
            GTest::Main
            ${PROJECT_NAME}_trac-ik
            ${PROJECT_NAME}_trac-ik_factory
            tesseract::kinematics
            tesseract::kinematics_kdl
            tesseract::scene_graph
            tesseract::state_solver
            tesseract::urdf
            tesseract::common
            console_bridge::console_bridge
            Eigen3::Eigen)
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${test_name} PRIVATE ${TESSERACT_COMPILE_DEFINITIONS_PUBLIC})
  # Location of the built plugin library, so the plugin loader can find it without an install step
  target_compile_definitions(${test_name} PRIVATE PLUGIN_DIR="$<TARGET_FILE_DIR:${PROJECT_NAME}_trac-ik_factory>")
  target_clang_tidy(${test_name} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${test_name} PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${test_name}
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${test_name})
  add_dependencies(${test_name} ${PROJECT_NAME}_trac-ik ${PROJECT_NAME}_trac-ik_factory)
endmacro()

add_trac_ik_gtest(${PROJECT_NAME}_kinematics_unit trac-ik_kinematics_unit.cpp)
add_trac_ik_gtest(${PROJECT_NAME}_factory_unit trac-ik_factory_unit.cpp)
