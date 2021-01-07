/**
 * @file joint_waypoint.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
class JointWaypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  JointWaypoint() = default;

  // This constructs a Joint Waypoint from xml element
  JointWaypoint(const tinyxml2::XMLElement& xml_element)
  {
    const tinyxml2::XMLElement* names_element = xml_element.FirstChildElement("Names");
    const tinyxml2::XMLElement* position_element = xml_element.FirstChildElement("Position");
    if (!names_element)
      throw std::runtime_error("JointWaypoint: Must have Names element.");

    if (!position_element)
      throw std::runtime_error("JointWaypoint: Must have Position element.");

    std::vector<std::string> names_tokens, position_tokens;
    std::string names_string;
    tinyxml2::XMLError status = tesseract_common::QueryStringText(names_element, names_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypoint: Error parsing Names string");

    std::string position_string;
    status = tesseract_common::QueryStringText(position_element, position_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypoint: Error parsing Position string");

    boost::split(names_tokens, names_string, boost::is_any_of(" "), boost::token_compress_on);
    boost::split(position_tokens, position_string, boost::is_any_of(" "), boost::token_compress_on);

    if (names_tokens.empty() || position_tokens.empty())
      throw std::runtime_error("JointWaypoint: Names or Position elements are empty.");

    if (names_tokens.size() != position_tokens.size())
      throw std::runtime_error("JointWaypoint: Names and Position are not the same size.");

    if (!tesseract_common::isNumeric(position_tokens))
      throw std::runtime_error("JointWaypoint: Positions are not all numeric values.");

    joint_names = names_tokens;
    waypoint.resize(static_cast<long>(names_tokens.size()));
    for (long i = 0; i < static_cast<long>(position_tokens.size()); ++i)
      tesseract_common::toNumeric<double>(position_tokens[static_cast<std::size_t>(i)], waypoint[i]);
  }

  int getType() const { return static_cast<int>(WaypointType::JOINT_WAYPOINT); }

  void print(const std::string& prefix = "") const
  {
    std::cout << prefix << "Joint WP: " << this->transpose() << std::endl;
  };

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const
  {
    Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
    tinyxml2::XMLElement* xml_waypoint = doc.NewElement("Waypoint");
    xml_waypoint->SetAttribute("type", std::to_string(getType()).c_str());

    tinyxml2::XMLElement* xml_joint_waypoint = doc.NewElement("JointWaypoint");
    tinyxml2::XMLElement* xml_joint_names = doc.NewElement("Names");
    if (!joint_names.empty())
    {
      std::string jn = joint_names[0];
      for (std::size_t i = 1; i < joint_names.size(); ++i)
        jn += " " + joint_names[i];

      xml_joint_names->SetText(jn.c_str());
    }
    xml_joint_waypoint->InsertEndChild(xml_joint_names);

    std::stringstream position_string;
    position_string << waypoint.format(eigen_format);

    tinyxml2::XMLElement* xml_joint_position = doc.NewElement("Position");
    xml_joint_position->SetText(position_string.str().c_str());
    xml_joint_waypoint->InsertEndChild(xml_joint_position);

    xml_waypoint->InsertEndChild(xml_joint_waypoint);

    return xml_waypoint;
  }

#ifndef SWIG

  /////////////////////
  // Eigen Container //
  /////////////////////

  using ConstTransposeReturnType = Eigen::VectorXd::ConstTransposeReturnType;

  ////////////////////////
  // Eigen Constructors //
  ////////////////////////

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template <typename OtherDerived>
  JointWaypoint(std::vector<std::string> joint_names, const Eigen::MatrixBase<OtherDerived>& other)
    : waypoint(other), joint_names(std::move(joint_names))
  {
    if (static_cast<Eigen::Index>(this->joint_names.size()) != this->waypoint.rows())
      throw std::runtime_error("JointWaypoint: joint_names is not the same size as position!");
  }

  JointWaypoint(std::vector<std::string> joint_names, std::initializer_list<double> l)
    : joint_names(std::move(joint_names))
  {
    waypoint.resize(static_cast<Eigen::Index>(l.size()));
    Eigen::Index i = 0;
    for (auto& v : l)
      waypoint(i++) = v;

    if (static_cast<Eigen::Index>(this->joint_names.size()) != this->waypoint.rows())
      throw std::runtime_error("JointWaypoint: joint_names is not the same size as position!");
  }

  ///////////////////
  // Eigen Methods //
  ///////////////////

  /** @returns true if two are approximate */
  inline Eigen::Index size() const { return waypoint.size(); }
  /** @returns norm of vector */
  inline double norm() const { return waypoint.norm(); }
  /** @returns true if two are approximate */
  inline bool isApprox(const Eigen::VectorXd& other, double prec = 1e-12) { return waypoint.isApprox(other, prec); }
  /** @returns the transpose of the joint positions */
  inline ConstTransposeReturnType transpose() const { return waypoint.transpose(); }

  /////////////////////
  // Eigen Operators //
  /////////////////////

  // This method allows you to assign Eigen expressions to MyVectorType
  template <typename OtherDerived>
  inline JointWaypoint& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = other;
    return *this;
  }

  template <typename OtherDerived>
  inline JointWaypoint& operator*=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    waypoint = waypoint * other;
    return *this;
  }

  inline JointWaypoint& operator=(std::initializer_list<double> l)
  {
    waypoint.resize(static_cast<Eigen::Index>(l.size()));
    Eigen::Index i = 0;
    for (auto& v : l)
      waypoint(i++) = v;

    return *this;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator*(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint * other;
    return jwp;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator-(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint - other;
    return jwp;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator+(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint + other;
    return jwp;
  }

  inline JointWaypoint operator-(const JointWaypoint& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint - other.waypoint;
    return jwp;
  }

  template <typename OtherDerived>
  inline JointWaypoint operator+(const JointWaypoint& other) const
  {
    JointWaypoint jwp = *this;
    jwp.waypoint = jwp.waypoint + other.waypoint;
    return jwp;
  }

  inline Eigen::CommaInitializer<Eigen::VectorXd> operator<<(const double& s)
  {
    return Eigen::CommaInitializer<Eigen::VectorXd>(waypoint, s);
  }

  inline double& operator[](Eigen::Index i) { return waypoint[i]; }
  inline double operator[](Eigen::Index i) const { return waypoint[i]; }
  inline double& operator()(Eigen::Index i) { return waypoint(i); }
  inline double operator()(Eigen::Index i) const { return waypoint(i); }

  //////////////////////////
  // Implicit Conversions //
  //////////////////////////

  /** @return Implicit Conversions to read-only Eigen::VectorXd */
  inline operator const Eigen::VectorXd&() const { return waypoint; }

  /** @return Implicit Conversions to writable Eigen::VectorXd */
  inline operator Eigen::VectorXd&() { return waypoint; }

  /** @return Implicit Conversions to read-only Eigen::VectorXd */
  inline operator Eigen::Ref<const Eigen::VectorXd>() const { return Eigen::Ref<const Eigen::VectorXd>(waypoint); }

  /** @return Implicit Conversions to writable Eigen::VectorXd */
  inline operator Eigen::Ref<Eigen::VectorXd>() { return Eigen::Ref<Eigen::VectorXd>(waypoint); }

  //////////////////////////////////
  // Joint Waypoint Container //
  //////////////////////////////////

#endif  // SWIG

  Eigen::VectorXd waypoint;
  std::vector<std::string> joint_names;
  /** @brief Joint distance below waypoint that is allowed. Each element should be <= 0 */
  Eigen::VectorXd lower_tolerance;
  /** @brief Joint distance above waypoint that is allowed. Each element should be >= 0 */
  Eigen::VectorXd upper_tolerance;

  bool isToleranced() const
  {
    // Check if they are empty
    if (lower_tolerance.size() == 0 || upper_tolerance.size() == 0)
      return false;

    // Check if they are close to 0
    Eigen::VectorXd range = upper_tolerance - lower_tolerance;
    double sum = range.sum();
    return (sum < 1e-5);
  }
};
}  // namespace tesseract_planning

#ifdef SWIG
%extend tesseract_planning::JointWaypoint {
  JointWaypoint(std::vector<std::string> joint_names, const Eigen::VectorXd& other)
  {
    return new tesseract_planning::JointWaypoint(joint_names, other);
  }
}

%tesseract_command_language_add_waypoint_type(JointWaypoint)
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
