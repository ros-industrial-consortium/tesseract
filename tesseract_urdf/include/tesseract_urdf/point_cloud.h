/**
 * @file point_cloud.h
 * @brief Parse PCL point cloud to octree from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_URDF_POINT_CLOUD_H
#define TESSERACT_URDF_POINT_CLOUD_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <exception>
#include <tesseract_common/utils.h>
#include <tinyxml2.h>
#include <pcl/io/pcd_io.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/octree.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>

namespace tesseract_urdf
{
inline tesseract_geometry::Octree::Ptr parsePointCloud(const tinyxml2::XMLElement* xml_element,
                                                       const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                                       tesseract_geometry::Octree::SubType shape_type,
                                                       const bool prune,
                                                       const int /*version*/)
{
  std::string filename;
  if (tesseract_common::QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("PointCloud: Missing or failed parsing attribute 'filename'!"));

  double resolution;
  if (xml_element->QueryDoubleAttribute("resolution", &resolution) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("PointCloud: Missing or failed parsing point_cloud attribute "
                                              "'resolution'!"));

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tesseract_common::Resource::Ptr located_resource = locator->locateResource(filename);
  if (!located_resource || !located_resource->isFile())
  {
    // TODO: Handle point clouds that are not files
    CONSOLE_BRIDGE_logError("Point clouds can only be loaded from file");
    std::throw_with_nested(std::runtime_error("PointCloud: Unable to locate resource '" + filename + "'!"));
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(located_resource->getFilePath(), *cloud) == -1)
    std::throw_with_nested(std::runtime_error("PointCloud: Failed to import point cloud from '" + filename + "'!"));

  if (cloud->points.empty())
    std::throw_with_nested(std::runtime_error("PointCloud: Imported point cloud from '" + filename + "' is empty!"));

  auto geom = std::make_shared<tesseract_geometry::Octree>(*cloud, resolution, shape_type, prune);
  if (geom == nullptr)
    std::throw_with_nested(std::runtime_error("PointCloud: Failed to create Tesseract Octree Geometry from point "
                                              "cloud!"));

  return geom;
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_POINT_CLOUD_H
