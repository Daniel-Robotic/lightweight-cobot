#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace iiwa_planning {
struct Limit {
  double lower, upper, velocity, acceleration, jerk;
};
struct Sample {
  double time;
  std::vector<double> position, velocity, acceleration;
};
// Normalized time u=t/T; exactly the quintic used for JTC
// position/velocity/acceleration points.
inline std::vector<double> quintic(double p0, double p1, double v0, double v1,
                                   double a0, double a1, double t) {
  const double d = p1 - p0, v = v0 * t, w = v1 * t, a = a0 * t * t,
               b = a1 * t * t;
  return {p0,
          v,
          a / 2,
          10 * d - 6 * v - 4 * w - 1.5 * a + .5 * b,
          -15 * d + 8 * v + 7 * w + 1.5 * a - b,
          6 * d - 3 * v - 3 * w - .5 * a + .5 * b};
}
inline double choose(unsigned n, unsigned k) {
  double x = 1;
  for (unsigned i = 1; i <= k; ++i)
    x = x * (n + 1 - i) / i;
  return x;
}
// Convex hull bound in Bernstein basis, tightened by de Casteljau subdivision.
// Unlike sampling, it also bounds extrema between evaluation points.
inline std::pair<double, double> hull(const std::vector<double> &b,
                                      unsigned depth) {
  if (!depth)
    return {*std::min_element(b.begin(), b.end()),
            *std::max_element(b.begin(), b.end())};
  auto v = b;
  std::vector<double> left(b.size()), right(b.size());
  left[0] = v[0];
  right.back() = v.back();
  for (size_t k = 1; k < b.size(); ++k) {
    for (size_t i = 0; i < b.size() - k; ++i)
      v[i] = (v[i] + v[i + 1]) / 2;
    left[k] = v[0];
    right[b.size() - 1 - k] = v[b.size() - 1 - k];
  }
  const auto l = hull(left, depth - 1), r = hull(right, depth - 1);
  return {std::min(l.first, r.first), std::max(l.second, r.second)};
}
inline std::pair<double, double> bounds(const std::vector<double> &power) {
  std::vector<double> b(power.size(), 0);
  const unsigned n = power.size() - 1;
  for (unsigned i = 0; i <= n; ++i)
    for (unsigned k = 0; k <= i; ++k)
      b[i] += power[k] * choose(i, k) / choose(n, k);
  return hull(b, 5);
}
inline double limit_splines(std::vector<Sample> &points,
                            const std::vector<Limit> &limits) {
  if (points.size() < 2 || limits.empty())
    throw std::runtime_error("Need at least two trajectory points");
  for (const auto &l : limits) {
    for (double x : {l.lower, l.upper, l.velocity, l.acceleration, l.jerk})
      if (!std::isfinite(x))
        throw std::runtime_error("Non-finite joint limit");
    if (l.lower >= l.upper || l.velocity <= 0 || l.acceleration <= 0 ||
        l.jerk <= 0)
      throw std::runtime_error("Invalid joint limits");
  }
  for (const auto &p : points) {
    if (!std::isfinite(p.time) || p.time < 0 || p.time > 2147483647.0)
      throw std::runtime_error("Invalid trajectory time");
    for (const auto *values : {&p.position, &p.velocity, &p.acceleration}) {
      if (values->size() != limits.size())
        throw std::runtime_error("Incomplete trajectory derivatives");
      for (double x : *values)
        if (!std::isfinite(x))
          throw std::runtime_error("Non-finite trajectory");
    }
  }
  if (std::abs(points.front().time) > 1e-9)
    throw std::runtime_error("Trajectory must start at t=0");
  for (const auto *p : {&points.front(), &points.back()}) {
    for (double v : p->velocity)
      if (std::abs(v) > 1e-9)
        throw std::runtime_error(
            "Only stationary trajectory boundaries are supported");
    for (double a : p->acceleration)
      if (std::abs(a) > 1e-9)
        throw std::runtime_error(
            "Boundary acceleration must be zero for bounded jerk");
  }
  double scale = 1;
  for (size_t i = 1; i < points.size(); ++i) {
    const auto &p = points[i - 1];
    const auto &q = points[i];
    const double dt = q.time - p.time;
    if (dt < 1e-6)
      throw std::runtime_error(
          "Trajectory times must strictly increase (>= 1 us)");
    for (size_t j = 0; j < limits.size(); ++j) {
      auto c = quintic(p.position[j], q.position[j], p.velocity[j],
                       q.velocity[j], p.acceleration[j], q.acceleration[j], dt);
      auto position = bounds(c);
      const auto &l = limits[j];
      if (position.first < l.lower - 1e-10 || position.second > l.upper + 1e-10)
        throw std::runtime_error(
            "Interpolated position exceeds joint limit at joint index " +
            std::to_string(j));
      for (unsigned order = 1; order <= 3; ++order) {
        std::vector<double> derivative(c.size() - 1);
        for (size_t k = 1; k < c.size(); ++k)
          derivative[k - 1] = k * c[k] / dt;
        c = derivative;
        const auto b = bounds(c);
        const double peak = std::max(std::abs(b.first), std::abs(b.second));
        const double cap =
            order == 1 ? l.velocity : (order == 2 ? l.acceleration : l.jerk);
        const double ratio = peak / cap;
        if (!std::isfinite(ratio))
          throw std::runtime_error("Unrepresentable trajectory derivatives");
        scale = std::max(scale, std::pow(ratio, 1.0 / order));
      }
    }
  }
  if (scale > 1.0 + 1e-9) {
    scale *= 1.000001; // Margin for conversion of timestamps to nanoseconds.
    for (auto &p : points) {
      p.time *= scale;
      if (!std::isfinite(p.time) || p.time > 2147483647.0)
        throw std::runtime_error(
            "Trajectory duration cannot be represented by ROS Duration");
      for (double &v : p.velocity)
        v /= scale;
      for (double &a : p.acceleration)
        a /= scale * scale;
    }
  } else
    scale = 1;
  return scale;
}
} // namespace iiwa_planning
