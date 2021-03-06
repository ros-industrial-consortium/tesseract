/**
 * @file geometry.h
 * @brief Parse geometry from xml string
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
#ifndef TESSERACT_URDF_GEOMETRY_H
#define TESSERACT_URDF_GEOMETRY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tinyxml2
{
class XMLElement;
}
namespace tesseract_scene_graph
{
class ResourceLocator;
}
namespace tesseract_geometry
{
class Geometry;
}

namespace tesseract_urdf
{
/**
 * @brief Parse xml element geometry
 * @param xml_element The xml element
 * @param locator The Tesseract resource locator
 * @param visual Indicate if visual
 * @param version The version number
 * @return A Tesseract Geometry
 */
std::vector<std::shared_ptr<tesseract_geometry::Geometry>>
parseGeometry(const tinyxml2::XMLElement* xml_element,
              const std::shared_ptr<tesseract_scene_graph::ResourceLocator>& locator,
              bool visual,
              int version);

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_GEOMETRY_H
