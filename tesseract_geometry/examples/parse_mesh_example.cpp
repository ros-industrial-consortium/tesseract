#include <console_bridge/console.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/mesh_parser.h>
#include <iostream>

using namespace tesseract_geometry;

int main(int /*argc*/, char** /*argv*/)
{
  // documentation:start:1: Create meshes
  std::string mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.dae";
  std::vector<Mesh::Ptr> meshes = createMeshFromPath<Mesh>(mesh_file);
  // documentation:end:1: Create meshes

  // documentation:start:2: Print mesh information
  CONSOLE_BRIDGE_logInform("Number of meshes: %f", meshes.size());
  CONSOLE_BRIDGE_logInform("Mesh #1 Triangle Count: %f", meshes[0]->getTriangleCount());
  CONSOLE_BRIDGE_logInform("Mesh #1 Triangle Count: %f", meshes[0]->getVerticeCount());
  CONSOLE_BRIDGE_logInform("Mesh #2 Triangle Count: %f", meshes[1]->getTriangleCount());
  CONSOLE_BRIDGE_logInform("Mesh #2 Triangle Count: %f", meshes[1]->getVerticeCount());
  // documentation:end:2: Print mesh information
}
