/**
 * @file utils.cpp
 * @brief Common Tesseract Utility Functions
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <type_traits>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>

namespace tesseract_common
{
// Implicit Instantiation
template bool toNumeric<double>(const std::string&, double&);
template bool toNumeric<float>(const std::string&, float&);
template bool toNumeric<int>(const std::string&, int&);
template bool toNumeric<long>(const std::string&, long&);
template bool isIdentical<std::string>(const std::vector<std::string>&, const std::vector<std::string>&, bool);
template bool isIdentical<Eigen::Index>(const std::vector<Eigen::Index>&, const std::vector<Eigen::Index>&, bool);

// Similar to rethrow_if_nested
// but does nothing instead of calling std::terminate
// when std::nested_exception is nullptr.
template <typename E>
std::enable_if_t<!std::is_polymorphic<E>::value> my_rethrow_if_nested(const E&)
{
}

template <typename E>
std::enable_if_t<std::is_polymorphic<E>::value> my_rethrow_if_nested(const E& e)
{
  const auto* p = dynamic_cast<const std::nested_exception*>(std::addressof(e));
  if (p && p->nested_ptr())
    p->rethrow_nested();
}

Eigen::Vector4d computeRandomColor()
{
  Eigen::Vector4d c;
  c.setZero();
  c[3] = 1;
  while (almostEqualRelativeAndAbs(c[0], c[1]) || almostEqualRelativeAndAbs(c[2], c[1]) ||
         almostEqualRelativeAndAbs(c[2], c[0]))
  {
    c[0] = (rand() % 100) / 100.0;
    c[1] = (rand() % 100) / 100.0;
    c[2] = (rand() % 100) / 100.0;
  }
  return c;
}

void printNestedException(const std::exception& e, int level)
{
  std::cerr << std::string(static_cast<unsigned>(2 * level), ' ') << "exception: " << e.what() << std::endl;
  try
  {
    my_rethrow_if_nested(e);
  }
  catch (const std::exception& e)
  {
    printNestedException(e, level + 1);
  }
  catch (...)
  {
  }
}

std::string getTempPath() { return fs::temp_directory_path().string() + std::string(1, fs::path::preferred_separator); }

bool isNumeric(const std::string& s)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  double out;
  ss >> out;

  return !(ss.fail() || !ss.eof());
}

bool isNumeric(const std::vector<std::string>& sv)
{
  for (const auto& s : sv)
  {
    if (!isNumeric(s))
      return false;
  }

  return true;
}

Eigen::VectorXd generateRandomNumber(const Eigen::Ref<const Eigen::MatrixX2d>& limits)
{
  Eigen::VectorXd joint_values;
  joint_values.resize(limits.rows());
  for (long i = 0; i < limits.rows(); ++i)
  {
    std::uniform_real_distribution<double> sample(limits(i, 0), limits(i, 1));
    joint_values(i) = sample(mersenne);
  }
  return joint_values;
}

void ltrim(std::string& s) { s.erase(0, s.find_first_not_of(" \n\r\t\f\v")); }

void rtrim(std::string& s) { s.erase(s.find_last_not_of(" \n\r\t\f\v") + 1); }

void trim(std::string& s)
{
  ltrim(s);
  rtrim(s);
}

std::string getTimestampString()
{
  std::ostringstream oss;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  return oss.str();
}

void reorder(Eigen::Ref<Eigen::VectorXd> v, std::vector<Eigen::Index> order)
{
  assert(v.rows() == static_cast<Eigen::Index>(order.size()));
  // for all elements to put in place
  for (std::size_t i = 0; i < order.size() - 1; ++i)
  {
    if (order[i] == static_cast<Eigen::Index>(i))
      continue;

    size_t j;
    for (j = i + 1; j < order.size(); ++j)
    {
      if (order[j] == static_cast<Eigen::Index>(i))
        break;
    }
    std::swap(v[static_cast<Eigen::Index>(i)], v[order[i]]);
    std::swap(order[i], order[j]);
  }
}

tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLElement* xml_element, std::string& value)
{
  if (xml_element->Value() == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  value = std::string(xml_element->Value());
  trim(value);
  return tinyxml2::XML_SUCCESS;
}

tinyxml2::XMLError QueryStringText(const tinyxml2::XMLElement* xml_element, std::string& text)
{
  if (xml_element->GetText() == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  text = std::string(xml_element->GetText());
  trim(text);
  return tinyxml2::XML_SUCCESS;
}

tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLAttribute* xml_attribute, std::string& value)
{
  if (xml_attribute->Value() == nullptr)
    return tinyxml2::XML_WRONG_ATTRIBUTE_TYPE;

  value = std::string(xml_attribute->Value());
  trim(value);
  return tinyxml2::XML_SUCCESS;
}

tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string& value)
{
  const tinyxml2::XMLAttribute* attribute = xml_element->FindAttribute(name);
  if (attribute == nullptr)
    return tinyxml2::XML_NO_ATTRIBUTE;

  return QueryStringValue(attribute, value);
}

std::string StringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string default_value)
{
  std::string str = std::move(default_value);
  QueryStringAttribute(xml_element, name, str);
  return str;
}

tinyxml2::XMLError QueryStringAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                std::string& value)
{
  tinyxml2::XMLError status = QueryStringAttribute(xml_element, name, value);

  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_NO_ATTRIBUTE)
  {
    CONSOLE_BRIDGE_logError("Missing %s required attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute type '%s'", xml_element->Name(), name);
  }

  return status;
}

tinyxml2::XMLError QueryDoubleAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                double& value)
{
  tinyxml2::XMLError status = xml_element->QueryDoubleAttribute(name, &value);

  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_NO_ATTRIBUTE)
  {
    CONSOLE_BRIDGE_logError("Missing %s required attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute type '%s'", xml_element->Name(), name);
  }

  return status;
}

tinyxml2::XMLError QueryIntAttributeRequired(const tinyxml2::XMLElement* xml_element, const char* name, int& value)
{
  tinyxml2::XMLError status = xml_element->QueryIntAttribute(name, &value);

  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_NO_ATTRIBUTE)
  {
    CONSOLE_BRIDGE_logError("Missing %s required attribute '%s'", xml_element->Name(), name);
  }
  else if (status == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    CONSOLE_BRIDGE_logError("Invalid %s attribute type '%s'", xml_element->Name(), name);
  }

  return status;
}

bool almostEqualRelativeAndAbs(double a, double b, double max_diff, double max_rel_diff)
{
  double diff = std::fabs(a - b);
  if (diff <= max_diff)
    return true;

  a = std::fabs(a);
  b = std::fabs(b);
  double largest = (b > a) ? b : a;

  return (diff <= largest * max_rel_diff);
}

bool almostEqualRelativeAndAbs(const Eigen::Ref<const Eigen::VectorXd>& v1,
                               const Eigen::Ref<const Eigen::VectorXd>& v2,
                               double max_diff,
                               double max_rel_diff)
{
  if (v1.size() == 0 && v2.size() == 0)
    return true;
  if (v1.size() != v2.size())
    return false;

  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> a1 = v1.array();
  Eigen::ArrayWrapper<const Eigen::Ref<const Eigen::VectorXd>> a2 = v2.array();

  auto diff_abs = (a1 - a2).abs();
  double diff = diff_abs.maxCoeff();
  if (diff <= max_diff)
    return true;

  return (diff_abs <= (max_rel_diff * a1.abs().max(a2.abs()))).all();
}

}  // namespace tesseract_common
