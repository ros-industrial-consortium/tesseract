/**
 * @file sdf_mesh.h
 * @brief Tesseract SDF Mesh Geometry
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_GEOMETRY_SDF_MESH_H
#define TESSERACT_GEOMETRY_SDF_MESH_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>
#include <tesseract_common/types.h>

namespace tesseract_geometry
{
class SDFMesh : public Geometry
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SDFMesh>;
  using ConstPtr = std::shared_ptr<const SDFMesh>;

  SDFMesh(const std::shared_ptr<const tesseract_common::VectorVector3d>& vertices,
          const std::shared_ptr<const Eigen::VectorXi>& triangles,
          std::string file_path = "",
          Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
    : Geometry(GeometryType::SDF_MESH), vertices_(vertices), triangles_(triangles), file_path_(file_path), scale_(scale)
  {
    vertice_count_ = static_cast<int>(vertices->size());

    triangle_count_ = 0;
    for (int i = 0; i < triangles_->size(); ++i)
    {
      ++triangle_count_;
      int num_verts = (*triangles_)(i);
      i += num_verts;
      assert(num_verts == 3);
    }
  }

  SDFMesh(const std::shared_ptr<const tesseract_common::VectorVector3d>& vertices,
          const std::shared_ptr<const Eigen::VectorXi>& triangles,
          int triangle_count,
          std::string file_path = "",
          Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
    : Geometry(GeometryType::SDF_MESH)
    , vertices_(vertices)
    , triangles_(triangles)
    , triangle_count_(triangle_count)
    , file_path_(file_path)
    , scale_(scale)
  {
    vertice_count_ = static_cast<int>(vertices->size());
    assert((triangle_count * 4) == triangles_->size());
  }

  ~SDFMesh() override = default;

  const std::shared_ptr<const tesseract_common::VectorVector3d>& getVertices() const { return vertices_; }
  const std::shared_ptr<const Eigen::VectorXi>& getTriangles() const { return triangles_; }

  int getVerticeCount() const { return vertice_count_; }
  int getTriangleCount() const { return triangle_count_; }

  /**
   * @brief Get the path to file used to generate the mesh
   *
   * Note: If empty, assume it was manually generated.
   *
   * @return Absolute path to the mesh file
   */
  const std::string& getFilePath() const { return file_path_; }

  /**
   * @brief Get the scale applied to file used to generate the mesh
   * @return The scale x, y, z
   */
  const Eigen::Vector3d& getScale() const { return scale_; }

  Geometry::Ptr clone() const override
  {
    return SDFMesh::Ptr(new SDFMesh(vertices_, triangles_, triangle_count_, file_path_, scale_));
  }

private:
  std::shared_ptr<const tesseract_common::VectorVector3d> vertices_;
  std::shared_ptr<const Eigen::VectorXi> triangles_;

  int vertice_count_;
  int triangle_count_;
  std::string file_path_;
  Eigen::Vector3d scale_;
};
}  // namespace tesseract_geometry
#endif
