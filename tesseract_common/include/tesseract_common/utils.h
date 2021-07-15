/**
 * @file utils.h
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
#ifndef TESSERACT_COMMON_UTILS_H
#define TESSERACT_COMMON_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <random>
#include <iomanip>
#include <Eigen/Geometry>
#include <iostream>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_common
{
#if __cplusplus < 201703L
/** @brief Random number generator */
static std::mt19937 mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) };
#else
/** @brief Random number generator */
inline std::mt19937 mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) };
#endif

/**
 * @brief This computes a random color with alpha set to 1
 * @return A random color
 */
Eigen::Vector4d computeRandomColor();

/**
 * @brief Print a nested exception
 * @param e The exception to print
 * @param level The exception level which controls the indentation
 */
void printNestedException(const std::exception& e, int level = 0);

/**
 * @brief Get the host temp directory path
 * @return The host temp directory path
 */
std::string getTempPath();

/**
 * @brief Determine if a string is a number
 * @param s The string to evaluate
 * @return True if numeric, otherwise false.
 */
bool isNumeric(const std::string& s);

/**
 * @brief Determine if each string in vector is a number
 * @param sv The vector of strings to evaluate
 * @return True if all strings are numeric, otherwise false.
 */
bool isNumeric(const std::vector<std::string>& sv);

/**
 * @brief Given a set of limits it will generate a vector of randon numbers between the limit.
 * @param limits The limits to generated numbers based on.
 * @return A vector of random numbers between the provided limits.
 */
Eigen::VectorXd generateRandomNumber(const Eigen::Ref<const Eigen::MatrixX2d>& limits);

/**
 * @brief Left trim string
 * @param s The string to left trim
 */
void ltrim(std::string& s);

/**
 * @brief Right trim string
 * @param s The string to right trim
 */
void rtrim(std::string& s);

/**
 * @brief Trim left and right of string
 * @param s The string to trim
 */
void trim(std::string& s);

/**
 * @brief Check if two vector of strings are identical
 * @param vec1 Vector strings
 * @param vec2 Vector strings
 * @param ordered If true order is relavent, othwise if false order is not relavent
 */
template <typename T>
bool isIdentical(const std::vector<T>& vec1, const std::vector<T>& vec2, bool ordered = true)
{
  if (ordered)
    return std::equal(vec1.begin(), vec1.end(), vec2.begin());

  std::vector<T> v1 = vec1;
  std::vector<T> v2 = vec2;
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  return std::equal(v1.begin(), v1.end(), v2.begin());
}

/**
 * @brief Get Timestamp string
 * @return Timestamp string
 */
std::string getTimestampString();

/**
 * @brief Reorder Eigen::VectorXd implace given index list
 * @param v The vector to reorder
 * @param order A vector of index which define the new order
 */
void reorder(Eigen::Ref<Eigen::VectorXd> v, std::vector<Eigen::Index> order);

/**
 * @brief Query a string value from xml element
 * @param xml_element The xml element to query string from
 * @param value The value to update from the xml element
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE
 */
tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLElement* xml_element, std::string& value);

/**
 * @brief Query a string Text from xml element
 * @param xml_element The xml element to query string from
 * @param test The value to update from the xml element
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE
 */
tinyxml2::XMLError QueryStringText(const tinyxml2::XMLElement* xml_element, std::string& text);

/**
 * @brief Query a string value from xml attribute
 * @param xml_attribute The xml attribute to query string from
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLAttribute* xml_attribute, std::string& value);

/**
 * @brief Query a string attribute from an xml element
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string& value);

/**
 * @brief Get string attribute if exist. If it does not exist it returns the default value.
 * @param xml_element The xml element to attempt to parse attribute
 * @param name The name of the attribute
 * @param default_value The default value to return if attribute does not exist
 * @return Parsed string attribute or default value if attribute does not exist.
 */
std::string StringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string default_value);

/**
 * @brief Query a string attribute from an xml element and print error log
 *
 * This is the same QueryStringAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryStringAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                std::string& value);

/**
 * @brief Query a double attribute from an xml element and print error log
 *
 * This is the same QueryDoubleAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryDoubleAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                double& value);

/**
 * @brief Query a int attribute from an xml element and print error log
 *
 * This is the same QueryIntAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryIntAttributeRequired(const tinyxml2::XMLElement* xml_element, const char* name, int& value);

/**
 * @brief Check if two double are relatively equal
 * @details The max_diff is for handling when comparing numbers near zero
 * @param a Double
 * @param b Double
 * @param max_diff The max diff when comparing std::abs(a - b) <= max_diff, if true they are considered equal
 * @param max_rel_diff The max relative diff std::abs(a - b) <= largest * max_rel_diff, if true considered equal. The
 * largest is the largest of the absolute values of a and b.
 * @return True if they are relativily equal, otherwise false.
 */
bool almostEqualRelativeAndAbs(double a,
                               double b,
                               double max_diff = 1e-6,
                               double max_rel_diff = std::numeric_limits<double>::epsilon());

/**
 * @brief Check if two Eigen VectorXd are relatively and absolute equal
 * @details
 *    This is a vectorized implementation of the function above.
 *    The max_diff is for handling when comparing numbers near zero
 * @param a A vector of double
 * @param b A vector of double
 * @param max_diff The max diff when comparing max(abs(a - b)) <= max_diff, if true they are considered equal
 * @param max_rel_diff The max relative diff abs(a - b) <= largest * max_rel_diff, if true considered equal. The
 * largest is the largest of the absolute values of a and b.
 * @return True if they are relativily equal, otherwise false.
 */
bool almostEqualRelativeAndAbs(const Eigen::Ref<const Eigen::VectorXd>& v1,
                               const Eigen::Ref<const Eigen::VectorXd>& v2,
                               double max_diff = 1e-6,
                               double max_rel_diff = std::numeric_limits<double>::epsilon());

/**
 * @brief Convert a string to a numeric value type
 * @param s The string to be converted
 * @param value The value to be loaded with coverted string
 * @return True if successful, otherwise false
 */
template <typename FloatType>
inline bool toNumeric(const std::string& s, FloatType& value)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  FloatType out;
  ss >> out;

  if (ss.fail() || !ss.eof())
    return false;

  value = out;
  return true;
}

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_UTILS_H
