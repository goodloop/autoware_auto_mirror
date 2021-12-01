// Copyright 2018 Forrest
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef UTILIZATION__INTERPOLATION__CUBIC_SPLINE_HPP_
#define UTILIZATION__INTERPOLATION__CUBIC_SPLINE_HPP_


#include <common/types.hpp>

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Eigen/Eigen"

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{

using std::int64_t;

static std::vector<double> vec_diff(const std::vector<double> & input)
{
  std::vector<double> output;
  for (unsigned int i = 1; i < input.size(); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
}

static std::vector<double> cum_sum(const std::vector<double> & input)
{
  std::vector<double> output;
  double temp = 0;
  for (unsigned int i = 0; i < input.size(); i++) {
    temp += input[i];
    output.push_back(temp);
  }
  return output;
}

class Spline
{
public:
  std::vector<double> x;
  std::vector<double> y;
  size_t nx;
  std::vector<double> h;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  // Eigen::VectorXf c;
  std::vector<double> d;

  Spline() {}
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(const std::vector<double> & x_, const std::vector<double> & y_)
  : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_)
  {
    Eigen::MatrixXd A = calc_A();
    Eigen::VectorXd B = calc_B();
    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
    double * c_pointer = c_eigen.data();
    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (size_t i = 0; i < nx - 1; i++) {
      d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
      b.push_back((a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
  }

  double calc(double t)
  {
    if (t < x.front() || t > x.back()) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    size_t seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
  }

  double calc(double t, double s)
  {
    if (t < 0 || t > s) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    size_t seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
  }

  double calc_d(double t)
  {
    if (t < x.front() || t > x.back()) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    size_t seg_id = bisect(t, 0, nx - 1);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double calc_d(double t, double s)
  {
    if (t < 0 || t > s) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    size_t seg_id = bisect(t, 0, nx - 1);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double calc_dd(double t)
  {
    if (t < x.front() || t > x.back()) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    size_t seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

  double calc_dd(double t, double s)
  {
    if (t < 0.0 || t > s) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    size_t seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

private:
  Eigen::MatrixXd calc_A()
  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<int64_t>(nx), static_cast<int64_t>(nx));
    A(0, 0) = 1;
    for (size_t i = 0; i < nx - 1; i++) {
      if (i != nx - 2) {
        A(static_cast<int64_t>(i + 1), static_cast<int64_t>(i + 1)) = 2 * (h[i] + h[i + 1]);
      }
      A(static_cast<int64_t>(i + 1), static_cast<int64_t>(i)) = h[i];
      A(static_cast<int64_t>(i), static_cast<int64_t>(i + 1)) = h[i];
    }
    A(0, 1) = 0.0;
    A(static_cast<int64_t>(nx - 1), static_cast<int64_t>(nx - 2)) = 0.0;
    A(static_cast<int64_t>(nx - 1), static_cast<int64_t>(nx - 1)) = 1.0;
    return A;
  }
  Eigen::VectorXd calc_B()
  {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(static_cast<int64_t>(nx));
    for (size_t i = 0; i < nx - 2; i++) {
      B(static_cast<int64_t>(i + 1)) =
        3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
  }

  size_t bisect(double t, size_t start, size_t end)
  {
    size_t mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1) {
      return mid;
    } else if (t > x[mid]) {
      return bisect(t, mid, end);
    } else {
      return bisect(t, start, mid);
    }
  }
};

class Spline2D
{
public:
  Spline sx;
  Spline sy;
  std::vector<double> s;

  Spline2D(const std::vector<double> & x, const std::vector<double> & y)
  {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
    max_s_value_ = *std::max_element(s.begin(), s.end());
  }

  std::array<double, 2> calc_position(double s_t)
  {
    double x = sx.calc(s_t, max_s_value_);
    double y = sy.calc(s_t, max_s_value_);
    return {{x, y}};
  }

  double calc_curvature(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double ddx = sx.calc_dd(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    double ddy = sy.calc_dd(s_t, max_s_value_);
    return (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
  }

  double calc_yaw(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    return std::atan2(dy, dx);
  }

private:
  static std::vector<double> calc_s(const std::vector<double> & x, const std::vector<double> & y)
  {
    std::vector<double> ds;
    std::vector<double> out_s{0};
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    std::vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  }
  double max_s_value_;
};

class Spline3D
{
public:
  Spline sx;
  Spline sy;
  Spline sv;
  std::vector<double> s;

  Spline3D(
    const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & v)
  {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
    sv = Spline(s, v);
    max_s_value_ = *std::max_element(s.begin(), s.end());
  }

  std::array<double, 3> calc_trajectory_point(double s_t)
  {
    double x = sx.calc(s_t, max_s_value_);
    double y = sy.calc(s_t, max_s_value_);
    double v = sv.calc(s_t, max_s_value_);
    return {{x, y, v}};
  }

  double calc_curvature(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double ddx = sx.calc_dd(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    double ddy = sy.calc_dd(s_t, max_s_value_);
    return (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
  }

  double calc_yaw(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    return std::atan2(dy, dx);
  }

private:
  static std::vector<double> calc_s(const std::vector<double> & x, const std::vector<double> & y)
  {
    std::vector<double> ds;
    std::vector<double> out_s{0};
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    std::vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  }
  double max_s_value_;
};

class Spline4D
{
public:
  Spline sx;
  Spline sy;
  Spline sz;
  Spline sv;
  std::vector<double> s;

  Spline4D(
    const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<double> & z,
    const std::vector<double> & v)
  {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
    sz = Spline(s, z);
    sv = Spline(s, v);
    max_s_value_ = *std::max_element(s.begin(), s.end());
  }

  std::array<double, 4> calc_trajectory_point(double s_t)
  {
    double x = sx.calc(s_t, max_s_value_);
    double y = sy.calc(s_t, max_s_value_);
    double z = sz.calc(s_t, max_s_value_);
    double v = sv.calc(s_t, max_s_value_);
    return {{x, y, z, v}};
  }

  double calc_curvature(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double ddx = sx.calc_dd(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    double ddy = sy.calc_dd(s_t, max_s_value_);
    return (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
  }

  double calc_yaw(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    return std::atan2(dy, dx);
  }

private:
  static std::vector<double> calc_s(const std::vector<double> & x, const std::vector<double> & y)
  {
    std::vector<double> ds;
    std::vector<double> out_s{0};
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    std::vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  }
  double max_s_value_;
};
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#endif  // UTILIZATION__INTERPOLATION__CUBIC_SPLINE_HPP_
