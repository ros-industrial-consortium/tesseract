find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(LAPACK REQUIRED) # Requried for ikfast

if(NOT TARGET GTest::GTest)
  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest.so")
    endif()
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "gtest_main.lib")
    else()
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest_main.so")
    endif()
  endif()
endif()

include(GoogleTest)

add_library(abb_irb2400_ikfast_kinematics abb_irb2400_ikfast_solver.cpp abb_irb2400_ikfast_kinematics.cpp)
target_link_libraries(abb_irb2400_ikfast_kinematics PUBLIC ${PROJECT_NAME}_ikfast ${LAPACK_LIBRARIES})
target_compile_definitions(
  abb_irb2400_ikfast_kinematics
  PUBLIC -DIKFAST_NO_MAIN
         -DIKFAST_CLIBRARY
         -DIKFAST_HAS_LIBRARY
         ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_options(abb_irb2400_ikfast_kinematics PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_options(abb_irb2400_ikfast_kinematics PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_include_directories(abb_irb2400_ikfast_kinematics PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")
target_include_directories(abb_irb2400_ikfast_kinematics SYSTEM PUBLIC ${LAPACK_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_dependencies(abb_irb2400_ikfast_kinematics ${PROJECT_NAME}_ikfast)

add_executable(${PROJECT_NAME}_core_unit kinematics_core_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_core
          ${PROJECT_NAME}_kdl
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_core_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_core_unit)
add_dependencies(${PROJECT_NAME}_core_unit ${PROJECT_NAME}_core)
add_dependencies(run_tests ${PROJECT_NAME}_core_unit)

add_executable(${PROJECT_NAME}_kdl_unit kdl_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_kdl_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_kdl_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_kdl_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_kdl_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_kdl_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_kdl_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_kdl_unit)
add_dependencies(${PROJECT_NAME}_kdl_unit ${PROJECT_NAME}_kdl)
add_dependencies(run_tests ${PROJECT_NAME}_kdl_unit)

add_executable(${PROJECT_NAME}_opw_unit opw_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_opw_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_opw
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_opw_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_opw_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_opw_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_opw_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_opw_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_opw_unit)
add_dependencies(${PROJECT_NAME}_opw_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_opw)
add_dependencies(run_tests ${PROJECT_NAME}_opw_unit)

add_executable(${PROJECT_NAME}_ikfast_unit ikfast_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ikfast_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_ikfast
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph
          abb_irb2400_ikfast_kinematics)
target_compile_options(${PROJECT_NAME}_ikfast_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ikfast_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ikfast_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS}
                  ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ikfast_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ikfast_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_ikfast_unit)
add_dependencies(${PROJECT_NAME}_ikfast_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_ikfast)
add_dependencies(run_tests ${PROJECT_NAME}_ikfast_unit)

add_executable(${PROJECT_NAME}_rop_unit rop_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_rop_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_opw
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_rop_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_rop_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_rop_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_rop_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_rop_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_rop_unit)
add_dependencies(${PROJECT_NAME}_rop_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_opw)
add_dependencies(run_tests ${PROJECT_NAME}_rop_unit)

add_executable(${PROJECT_NAME}_rep_unit rep_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_rep_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_opw
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_rep_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_rep_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_rep_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_rep_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_rep_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_rep_unit)
add_dependencies(${PROJECT_NAME}_rep_unit ${PROJECT_NAME}_kdl ${PROJECT_NAME}_opw)
add_dependencies(run_tests ${PROJECT_NAME}_rep_unit)

add_executable(${PROJECT_NAME}_ur_unit ur_kinematics_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ur_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}_kdl
          ${PROJECT_NAME}_ur
          tesseract::tesseract_support
          tesseract::tesseract_urdf
          tesseract::tesseract_scene_graph)
target_compile_options(${PROJECT_NAME}_ur_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ur_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ur_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ur_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ur_unit
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_ur_unit)
add_dependencies(${PROJECT_NAME}_ur_unit ${PROJECT_NAME}_kdl)
add_dependencies(run_tests ${PROJECT_NAME}_ur_unit)
