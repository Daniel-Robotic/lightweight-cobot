#include "iiwa_planning/spline_limits.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
PYBIND11_MODULE(_iiwa_spline_limits, m) {
  m.def("retime", [](const std::vector<double> &t,
                     const std::vector<std::vector<double>> &q,
                     const std::vector<std::vector<double>> &v,
                     const std::vector<std::vector<double>> &a,
                     const std::vector<std::array<double, 5>> &caps) {
    if (q.size() != t.size() || v.size() != t.size() || a.size() != t.size())
      throw std::runtime_error("Point array sizes differ");
    std::vector<iiwa_planning::Sample> points;
    for (size_t i = 0; i < t.size(); ++i)
      points.push_back({t[i], q[i], v[i], a[i]});
    std::vector<iiwa_planning::Limit> limits;
    for (const auto &c : caps)
      limits.push_back({c[0], c[1], c[2], c[3], c[4]});
    const double scale = iiwa_planning::limit_splines(points, limits);
    std::vector<double> times;
    std::vector<std::vector<double>> positions, velocities, accelerations;
    for (const auto &p : points) {
      times.push_back(p.time);
      positions.push_back(p.position);
      velocities.push_back(p.velocity);
      accelerations.push_back(p.acceleration);
    }
    return py::make_tuple(times, positions, velocities, accelerations, scale);
  });
}
