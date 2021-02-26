/**
 * @file utils.h
 * @brief Common Tesseract Utilities for Interpolating
 *
 * @date January 7, 2020
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
#ifndef TESSERACT_COMMON_INTERPOLATORS_H
#define TESSERACT_COMMON_INTERPOLATORS_H

#include <tesseract_common/macros.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/** @brief Uses Eigen/Splines to fit a 3rd order B-Spline to the input data and provides an operator for retrieving
 * intermediate points
 *
 * Note: B-Spline may be lower order than requested for short input vectors */
class SplineFunction
{
public:
  /**
   * @brief SplineFunction Constructor for interpolating while applying restrictions on the derivates of some points
   *
   * Example: Start/end with 0 velocity,
   * Derivatives is a VectorXd of length 2 filled with zeros;
   * Indices is then a VectorXi of length 2 filled with 0 and x_vec.size()-1
   *
   * **Note: There is a known bug in Eigen as of 1/6/2020. This will give incorrect results unless you patch
   * SplineFitting.h.** See stackoverflow.com/questions/48382939 and https://gitlab.com/libeigen/eigen/merge_requests/41
   * @param x_vec Independent variable. Probably time for most motion planning problems
   * @param y_vec Dependent variable. Probably joint values for most motion planning problems.
   * @param derivatives This is a vector of derivative values.
   * @param indices This is a vector of indices of where those derivatives are applied
   */
  SplineFunction(const Eigen::Ref<Eigen::VectorXd>& x_vec,
                 const Eigen::Ref<Eigen::VectorXd>& y_vec,
                 const Eigen::Ref<Eigen::VectorXd>& derivatives,
                 const Eigen::Ref<Eigen::VectorXi>& indices)
    : x_min_(x_vec.minCoeff())
    , x_max_(x_vec.maxCoeff())
    ,
    // Scale x values to [0:1] and curve fit with a 3rd order B-Spline.
    spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::InterpolateWithDerivatives(
        y_vec.transpose(),
        derivatives.transpose(),
        indices,
        std::min<unsigned>(static_cast<unsigned>(x_vec.rows()) - 1, 3),
        scaledValues(x_vec)))
  {
    assert(derivatives.size() == indices.size());
    assert(!(x_vec.array().isNaN().any()));
    assert(!(x_vec.array().isInf().any()));
    assert(!(y_vec.array().isNaN().any()));
    assert(!(y_vec.array().isInf().any()));
  }

  /**
   * @brief Constructor for interpolating with no restrictions on derivatives
   * @param x_vec Independent variable. Probably time for most motion planning problems
   * @param y_vec Dependent variable. Probably joint values for most motion planning problems.
   */
  SplineFunction(const Eigen::Ref<Eigen::VectorXd>& x_vec, const Eigen::Ref<Eigen::VectorXd>& y_vec)
    : x_min_(x_vec.minCoeff())
    , x_max_(x_vec.maxCoeff())
    ,
    // Scale x values to [0:1] and curve fit with a 3rd order B-Spline.
    spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y_vec.transpose(),
                                                                        std::min<long>(x_vec.rows() - 1, 3),
                                                                        scaledValues(x_vec)))
  {
    assert(!(x_vec.array().isNaN().any()));
    assert(!(x_vec.array().isInf().any()));
    assert(!(y_vec.array().isNaN().any()));
    assert(!(y_vec.array().isInf().any()));
  }

  /** @brief Returns the y value at point x in the spline */
  double operator()(double x) const
  {
    // x values need to be scaled to [0:1] in extraction as well.
    return spline_(scaledValue(x))(0);
  }

  /** @brief Returns a vector of interpolated y values for each x in the input vector */
  Eigen::VectorXd operator()(const Eigen::Ref<Eigen::VectorXd>& x_vec) const
  {
    return x_vec.unaryExpr([this](double x) { return spline_(scaledValue(x))(0); });
  }

private:
  /** @brief Scales value to [0:1] based on x_min and x_max */
  double scaledValue(double x) const { return (x - x_min_) / (x_max_ - x_min_); }

  /** @brief Scales vector such that each value is [0:1] based on x_min and x_max
   *
   * It is a requirement for Interpolate that x be on the interval [0:1] */
  Eigen::RowVectorXd scaledValues(const Eigen::Ref<Eigen::VectorXd>& x_vec) const
  {
    return x_vec.unaryExpr([this](double x) { return scaledValue(x); }).transpose();
  }

  /** @brief Minimum value in x_vec. Used to scale inputs to [0:1] */
  double x_min_;
  /** @brief Maximum value in x_vec. Used to scale inputs to [0:1] */
  double x_max_;

  /** @brief One dimensional Spline that is used to interpolate the data
   *
   * Note: operator(double) returns an array of points. In this case the 0th element is the y value*/
  Eigen::Spline<double, 1> spline_;
};

/**
 * @brief Interpolates each column in a TrajArray using a cubic spline. The resuls are an evenly spaced trajectory with
 * result_length size.
 *
 * Note: While the spline will hit these points, the resulting TrajArray is not guaranteed to include the original
 * points unless result_length is of size n * (input_traj.rows() - 1) + 1
 * @param input_traj Input TrajArray. Each column will be interpolated with a cubic spline.
 * @param result_length Number of rows in the resulting TrajArray. For best results this should be size n *
 * (input_traj.rows() - 1) + 1
 * @return Resulting TrajArray of size results_length x input_traj.cols()
 */
inline TrajArray interpolateCubicSpline(const Eigen::Ref<TrajArray>& input_traj, const int& result_length)
{
  TrajArray results(result_length, input_traj.cols());
  for (long ind = 0; ind < input_traj.cols(); ind++)
  {
    // Fit the spline to the input data
    Eigen::VectorXd t_in =
        Eigen::VectorXd::LinSpaced(input_traj.rows(), 0.0, static_cast<double>(input_traj.rows() - 1));
    Eigen::VectorXd y_in = input_traj.col(ind);
    SplineFunction spline(t_in, y_in);

    // Extract evenly spaced points from spline
    Eigen::VectorXd t_out = Eigen::VectorXd::LinSpaced(result_length, 0.0, static_cast<double>(input_traj.rows() - 1));
    Eigen::VectorXd y_out = spline(t_out);
    results.col(ind) = y_out;
  }

  return results;
}

}  // namespace tesseract_common

#endif
