
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/tesseract.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_process_managers/process_generators/random_process_generator.h>
#include <tesseract_process_managers/taskflow_generators/sequential_failure_tree_taskflow.h>
#include <tesseract_process_managers/examples/freespace_example_program.h>
#include <tesseract_process_managers/process_managers/freespace_process_manager.h>

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

int main()
{
  // --------------------
  // Perform setup
  // --------------------
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  tesseract::Tesseract::Ptr tesseract = std::make_shared<tesseract::Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  tesseract->init(urdf_path, srdf_path, locator);
  auto fwd_kin = tesseract->getFwdKinematicsManager()->getFwdKinematicSolver("manipulator");
  auto inv_kin = tesseract->getInvKinematicsManager()->getInvKinematicSolver("manipulator");

  // --------------------
  // Define the program
  // --------------------
  CompositeInstruction program = freespaceExampleProgram();
  const Instruction program_instruction{ program };
  Instruction seed = CompositeInstruction();

  // --------------------
  // Print Diagnostics
  // --------------------
  program_instruction.print("Program: ");

  // --------------------
  // Define the Process Input
  // --------------------
  ProcessInput input(tesseract, program_instruction, seed);
  std::cout << "Input size: " << input.size() << std::endl;

  // --------------------
  // Initialize Freespace Manager
  // --------------------
  FreespaceProcessManager freespace_manager;
  freespace_manager.init(input);

  // --------------------
  // Solve
  // --------------------
  freespace_manager.execute();

  std::cout << "Execution Complete" << std::endl;

  return 0;
}
