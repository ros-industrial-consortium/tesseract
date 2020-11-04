#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>

#include <tesseract_process_managers/process_input.h>
#include <tesseract_process_managers/process_managers/raster_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_global_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_only_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_only_global_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_dt_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_dt_process_manager.h>
#include <tesseract_process_managers/process_generators/seed_min_length_process_generator.h>
#include <tesseract_process_managers/taskflows/cartesian_taskflow.h>
#include <tesseract_process_managers/taskflows/freespace_taskflow.h>
#include <tesseract_process_managers/taskflows/descartes_taskflow.h>
#include <tesseract_process_managers/taskflows/trajopt_taskflow.h>

#include "raster_example_program.h"
#include "raster_dt_example_program.h"
#include "raster_waad_example_program.h"
#include "raster_waad_dt_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract;
using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_planning;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class TesseractProcessManagerUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  ManipulatorInfo manip;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";
    manip.working_frame = "base_link";
  }
};

TEST_F(TesseractProcessManagerUnit, SeedMinLengthProcessGeneratorTest)
{
  tesseract_planning::CompositeInstruction program = freespaceExampleProgramABB();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  // Define the Process Input
  auto cur_state = tesseract_ptr_->getEnvironment()->getCurrentState();
  CompositeInstruction seed = generateSeed(program, cur_state, tesseract_ptr_);

  Instruction program_instruction = program;
  Instruction seed_instruction = seed;

  long current_length = getMoveInstructionCount(seed);
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed_instruction);

  SeedMinLengthProcessGenerator smlpg(current_length);
  EXPECT_TRUE(smlpg.generateConditionalTask(input)() == 1);
  long final_length = getMoveInstructionCount(*(input.getResults()->cast_const<CompositeInstruction>()));
  EXPECT_TRUE(final_length == current_length);

  SeedMinLengthProcessGenerator smlpg2(2 * current_length);
  EXPECT_TRUE(smlpg2.generateConditionalTask(input)() == 1);
  long final_length2 = getMoveInstructionCount(*(input.getResults()->cast_const<CompositeInstruction>()));
  EXPECT_TRUE(final_length2 >= (2 * current_length));

  seed_instruction = seed;
  ProcessInput input2(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed_instruction);

  SeedMinLengthProcessGenerator smlpg3(3 * current_length);
  EXPECT_TRUE(smlpg3.generateConditionalTask(input2)() == 1);
  long final_length3 = getMoveInstructionCount(*(input2.getResults()->cast_const<CompositeInstruction>()));
  EXPECT_TRUE(final_length3 >= (3 * current_length));
}

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[process_profile] = std::make_shared<SimplePlannerDefaultPlanProfile>();
  interpolator->plan_profiles[freespace_profile] = std::make_shared<SimplePlannerDefaultPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionCount(request.instructions);
  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerDefaultLVSPlanProfileTest)
{
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[process_profile] = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  interpolator->plan_profiles[freespace_profile] = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(168, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerDefaultPlanProfileTest)
{
  CompositeInstruction program = freespaceExampleProgramABB(DEFAULT_PROFILE_KEY, DEFAULT_PROFILE_KEY);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[DEFAULT_PROFILE_KEY] = std::make_shared<SimplePlannerDefaultPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionCount(request.instructions);
  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerDefaultLVSPlanProfileTest)
{
  CompositeInstruction program = freespaceExampleProgramABB(DEFAULT_PROFILE_KEY, DEFAULT_PROFILE_KEY);
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  interpolator->plan_profiles[DEFAULT_PROFILE_KEY] = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(33, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterProcessManagerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                      std::move(transition_taskflow_generator),
                                      std::move(raster_taskflow_generator),
                                      1);

  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterProcessManagerDefaultLVSPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                      std::move(transition_taskflow_generator),
                                      std::move(raster_taskflow_generator),
                                      1);

  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterGlobalProcessManagerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Create taskflows
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultPlanProfile>();
  tesseract_planning::DescartesTaskflowParams descartes_params;
  descartes_params.enable_simple_planner = true;
  descartes_params.enable_post_contact_discrete_check = false;
  descartes_params.enable_post_contact_continuous_check = false;
  descartes_params.enable_time_parameterization = false;
  descartes_params.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  descartes_params.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  auto global_taskflow_generator = createDescartesTaskflow(descartes_params);

  FreespaceTaskflowParams fparams;
  fparams.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  fparams.enable_simple_planner = false;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  TrajOptTaskflowParams cparams;
  cparams.enable_simple_planner = false;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createTrajOptTaskflow(cparams);
  RasterGlobalProcessManager raster_manager(std::move(global_taskflow_generator),
                                            std::move(freespace_taskflow_generator),
                                            std::move(transition_taskflow_generator),
                                            std::move(raster_taskflow_generator),
                                            1);

  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterGlobalProcessManagerDefaultLVSPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Create taskflows
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  tesseract_planning::DescartesTaskflowParams descartes_params;
  descartes_params.enable_simple_planner = true;
  descartes_params.enable_post_contact_discrete_check = false;
  descartes_params.enable_post_contact_continuous_check = false;
  descartes_params.enable_time_parameterization = false;
  descartes_params.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  descartes_params.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  auto global_taskflow_generator = createDescartesTaskflow(descartes_params);

  FreespaceTaskflowParams fparams;
  fparams.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  fparams.enable_simple_planner = false;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  TrajOptTaskflowParams cparams;
  cparams.enable_simple_planner = false;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createTrajOptTaskflow(cparams);
  RasterGlobalProcessManager raster_manager(std::move(global_taskflow_generator),
                                            std::move(freespace_taskflow_generator),
                                            std::move(transition_taskflow_generator),
                                            std::move(raster_taskflow_generator),
                                            1);

  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyProcessManagerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Creat Taskflows
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterOnlyProcessManager raster_manager(
      std::move(transition_taskflow_generator), std::move(raster_taskflow_generator), 1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyProcessManagerDefaultLVSPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Creat Taskflows
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterOnlyProcessManager raster_manager(
      std::move(transition_taskflow_generator), std::move(raster_taskflow_generator), 1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyGlobalProcessManagerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Create taskflows
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultPlanProfile>();
  tesseract_planning::DescartesTaskflowParams descartes_params;
  descartes_params.enable_simple_planner = true;
  descartes_params.enable_post_contact_discrete_check = false;
  descartes_params.enable_post_contact_continuous_check = false;
  descartes_params.enable_time_parameterization = false;
  descartes_params.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  descartes_params.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto global_taskflow_generator = createDescartesTaskflow(descartes_params);

  FreespaceTaskflowParams fparams;
  fparams.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  fparams.enable_simple_planner = false;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  TrajOptTaskflowParams cparams;
  cparams.enable_simple_planner = false;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createTrajOptTaskflow(cparams);
  RasterOnlyGlobalProcessManager raster_manager(std::move(global_taskflow_generator),
                                                std::move(transition_taskflow_generator),
                                                std::move(raster_taskflow_generator),
                                                1);

  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterOnlyGlobalProcessManagerDefaultLVSPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterOnlyExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Create taskflows
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  tesseract_planning::DescartesTaskflowParams descartes_params;
  descartes_params.enable_simple_planner = true;
  descartes_params.enable_post_contact_discrete_check = false;
  descartes_params.enable_post_contact_continuous_check = false;
  descartes_params.enable_time_parameterization = false;
  descartes_params.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  descartes_params.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto global_taskflow_generator = createDescartesTaskflow(descartes_params);

  FreespaceTaskflowParams fparams;
  fparams.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  fparams.enable_simple_planner = false;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  TrajOptTaskflowParams cparams;
  cparams.enable_simple_planner = false;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createTrajOptTaskflow(cparams);
  RasterOnlyGlobalProcessManager raster_manager(std::move(global_taskflow_generator),
                                                std::move(transition_taskflow_generator),
                                                std::move(raster_taskflow_generator),
                                                1);

  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterDTProcessManagerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterDTExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterDTProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                        std::move(transition_taskflow_generator),
                                        std::move(raster_taskflow_generator),
                                        1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterDTProcessManagerDefaultLVSPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string process_profile = "PROCESS";

  CompositeInstruction program = rasterDTExampleProgram(freespace_profile, process_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterDTProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                        std::move(transition_taskflow_generator),
                                        std::move(raster_taskflow_generator),
                                        1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADProcessManagerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";
  CompositeInstruction program =
      rasterWAADExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterWAADProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                          std::move(transition_taskflow_generator),
                                          std::move(raster_taskflow_generator),
                                          1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADProcessManagerDefaultLVSPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";
  CompositeInstruction program =
      rasterWAADExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterWAADProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                          std::move(transition_taskflow_generator),
                                          std::move(raster_taskflow_generator),
                                          1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADDTProcessManagerDefaultPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADDTExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterWAADDTProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                            std::move(transition_taskflow_generator),
                                            std::move(raster_taskflow_generator),
                                            1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADDTProcessManagerDefaultLVSPlanProfileTest)
{
  // Define the program
  std::string freespace_profile = DEFAULT_PROFILE_KEY;
  std::string approach_profile = "APPROACH";
  std::string process_profile = "PROCESS";
  std::string departure_profile = "DEPARTURE";

  CompositeInstruction program =
      rasterWAADDTExampleProgram(freespace_profile, approach_profile, process_profile, departure_profile);
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto default_simple_plan_profile = std::make_shared<SimplePlannerDefaultLVSPlanProfile>();
  FreespaceTaskflowParams fparams;
  fparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  fparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;
  CartesianTaskflowParams cparams;
  cparams.simple_plan_profiles[freespace_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[approach_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[process_profile] = default_simple_plan_profile;
  cparams.simple_plan_profiles[departure_profile] = default_simple_plan_profile;

  auto freespace_taskflow_generator = createFreespaceTaskflow(fparams);
  auto transition_taskflow_generator = createFreespaceTaskflow(fparams);
  auto raster_taskflow_generator = createCartesianTaskflow(cparams);
  RasterWAADDTProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                            std::move(transition_taskflow_generator),
                                            std::move(raster_taskflow_generator),
                                            1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
