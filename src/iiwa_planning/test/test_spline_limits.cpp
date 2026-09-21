#include "iiwa_planning/spline_limits.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace iiwa_planning;
int main() {
  // Rest-to-rest quintic: endpoint acceleration is zero, interior peak > 5.
  std::vector<Sample> points{{0, {0}, {0}, {0}}, {1, {1}, {0}, {0}}};
  std::vector<Limit> limits{{-2, 2, 2, 1, 2}};
  const double scale = limit_splines(points, limits);
  assert(scale > 3.0);
  assert(std::abs(points.back().position[0] - 1) < 1e-12);
  assert(limit_splines(points, limits) <= 1.000001);
  // Consistent time scaling keeps the geometric quintic unchanged.
  auto p = quintic(0, 1, 0, 0, 0, 0, 1);
  auto q = quintic(0, 1, 0, 0, 0, 0, points.back().time);
  for (int i = 0; i <= 5; ++i)
    assert(std::abs(p[i] - q[i]) < 1e-10);
  bool bad = false;
  try {
    std::vector<Sample> s{{0, {0}, {0}, {0}}, {0, {1}, {0}, {0}}};
    limit_splines(s, limits);
  } catch (const std::exception &) {
    bad = true;
  }
  assert(bad);
  bad = false;
  // Positions are in range but large endpoint velocities overshoot between
  // them.
  try {
    std::vector<Sample> s{{0, {0}, {0}, {0}},
                          {1, {0}, {12}, {0}},
                          {2, {0}, {-12}, {0}},
                          {3, {0}, {0}, {0}}};
    limit_splines(s, limits);
  } catch (const std::exception &) {
    bad = true;
  }
  assert(bad);
  bad = false;
  try {
    std::vector<Sample> s{{0, {NAN}, {0}, {0}}, {1, {0}, {0}, {0}}};
    limit_splines(s, limits);
  } catch (const std::exception &) {
    bad = true;
  }
  assert(bad);
  // Non-rest boundaries must not slip past the finite-segment jerk check.
  bad = false;
  try {
    std::vector<Sample> s{{0, {0}, {0}, {1}}, {1, {1}, {0}, {0}}};
    limit_splines(s, limits);
  } catch (const std::exception &) {
    bad = true;
  }
  assert(bad);
  std::cout << "spline limits OK\n";
}
