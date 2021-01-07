cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(cmake_common_scripts REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(descartes_light REQUIRED)
find_package(descartes_samplers REQUIRED)
find_package(ompl REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(trajopt REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(cmake_common_scripts REQUIRED)
# serialization was required because ompl does not include find_dependency for its required dependencies
find_package(Boost REQUIRED COMPONENTS serialization system filesystem program_options)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if (NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

initialize_code_coverage()
set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/* /*/include/descartes_light/* /*/include/opw_kinematics/* /*/include/trajopt/* /*/include/trajopt_sco/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# Create interface for core
add_library(${PROJECT_NAME}_core src/core/utils.cpp)
target_link_libraries(${PROJECT_NAME}_core PUBLIC tesseract::tesseract_environment_core tesseract::tesseract_common tesseract::tesseract_command_language trajopt::trajopt console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_core ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

# Descartes Planner
add_library(${PROJECT_NAME}_descartes
  src/descartes/descartes_motion_planner.cpp
  src/descartes/descartes_collision.cpp
  src/descartes/descartes_collision_edge_evaluator.cpp
  src/descartes/descartes_robot_sampler.cpp
  src/descartes/descartes_motion_planner_status_category.cpp
  src/descartes/serialize.cpp
  src/descartes/deserialize.cpp
  src/descartes/descartes_utils.cpp
  src/descartes/profile/descartes_default_plan_profile.cpp)
target_link_libraries(${PROJECT_NAME}_descartes PUBLIC ${PROJECT_NAME}_core descartes::descartes_light descartes::descartes_samplers ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_descartes PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_descartes PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_descartes PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_descartes ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_descartes PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_descartes ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_descartes PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_descartes SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS})

# OMPL Freespace Planning Interface
set(OMPL_SRC src/ompl/ompl_motion_planner.cpp
  src/ompl/continuous_motion_validator.cpp
  src/ompl/discrete_motion_validator.cpp
  src/ompl/weighted_real_vector_state_sampler.cpp
  src/ompl/ompl_motion_planner_status_category.cpp
  src/ompl/ompl_planner_configurator.cpp
  src/ompl/ompl_problem.cpp
  src/ompl/profile/ompl_default_plan_profile.cpp
  src/ompl/problem_generators/default_problem_generator.cpp
  src/ompl/utils.cpp
  src/ompl/state_collision_validator.cpp
  src/ompl/compound_state_validator.cpp
  src/ompl/serialize.cpp
  src/ompl/deserialize.cpp)

#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  list(APPEND OMPL_SRC src/ompl/config/ompl_planner_constrained_config.cpp)
#endif()

message(AUTHOR_WARNING "OMPL INCLUDE DIRS: ${OMPL_INCLUDE_DIRS}")
add_library(${PROJECT_NAME}_ompl ${OMPL_SRC})
target_link_libraries(${PROJECT_NAME}_ompl PUBLIC ${PROJECT_NAME}_core ${OMPL_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_ompl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ompl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ompl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ompl ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ompl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_ompl ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_ompl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ompl SYSTEM PUBLIC
    ${OMPL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})
if(OMPL_VERSION VERSION_LESS "1.4.0")
    target_compile_definitions(${PROJECT_NAME}_ompl PUBLIC OMPL_LESS_1_4_0=ON)
endif()

# Simple Planner
add_library(${PROJECT_NAME}_simple
  src/simple/simple_motion_planner.cpp
  src/simple/profile/simple_planner_default_lvs_plan_profile.cpp
  src/simple/profile/simple_planner_default_plan_profile.cpp
  src/simple/profile/simple_planner_interpolation_plan_profile.cpp
  src/simple/step_generators/fixed_size_assign_position.cpp
  src/simple/step_generators/fixed_size_interpolation.cpp
  src/simple/step_generators/lvs_interpolation.cpp)
target_link_libraries(${PROJECT_NAME}_simple PUBLIC ${Boost_LIBRARIES} ${PROJECT_NAME}_core)
target_compile_options(${PROJECT_NAME}_simple PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_simple PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_simple PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_simple ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_simple PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_simple ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_simple PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_simple SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

# Trajopt Planner
add_library(${PROJECT_NAME}_trajopt
  src/trajopt/trajopt_collision_config.cpp
  src/trajopt/trajopt_motion_planner.cpp
  src/trajopt/trajopt_utils.cpp
  src/trajopt/profile/trajopt_default_plan_profile.cpp
  src/trajopt/profile/trajopt_default_composite_profile.cpp
  src/trajopt/profile/trajopt_default_solver_profile.cpp
  src/trajopt/problem_generators/default_problem_generator.cpp
  src/trajopt/serialize.cpp
  src/trajopt/deserialize.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt PUBLIC ${Boost_LIBRARIES} ${PROJECT_NAME}_core trajopt::trajopt trajopt::trajopt_utils trajopt::trajopt_sco)
target_compile_options(${PROJECT_NAME}_trajopt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trajopt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trajopt ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trajopt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_trajopt ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_trajopt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajopt SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

# Trajopt_ifopt Planner
add_library(${PROJECT_NAME}_trajopt_ifopt SHARED
  src/trajopt_ifopt/trajopt_ifopt_motion_planner.cpp
  src/trajopt_ifopt/trajopt_ifopt_utils.cpp
  src/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.cpp
  src/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.cpp
  src/trajopt_ifopt/problem_generators/default_problem_generator.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt_ifopt PUBLIC ${Boost_LIBRARIES} ${PROJECT_NAME}_core trajopt::trajopt_ifopt trajopt::trajopt_sqp)
target_compile_options(${PROJECT_NAME}_trajopt_ifopt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trajopt_ifopt ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trajopt_ifopt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_trajopt_ifopt ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_trajopt_ifopt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajopt_ifopt SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

configure_package(NAMESPACE tesseract
                  TARGETS ${PROJECT_NAME}_core
                          ${PROJECT_NAME}_descartes
                          ${PROJECT_NAME}_ompl
                          ${PROJECT_NAME}_simple
                          ${PROJECT_NAME}_trajopt
                          ${PROJECT_NAME}_trajopt_ifopt
                  )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN ".svn" EXCLUDE
)

add_subdirectory(examples)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()
