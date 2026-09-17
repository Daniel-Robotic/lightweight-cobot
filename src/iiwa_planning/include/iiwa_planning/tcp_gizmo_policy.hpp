#pragma once
#include <cmath>
#include <cstdint>
#include <map>
#include <string>

namespace iiwa_planning {
inline bool requiresGizmoMotion(double translation_error, double angular_error) {
  return translation_error >= 1e-5 || angular_error >= 1e-4;
}

// Pure arbitration state. The ROS node serializes all access. Steady time is
// used for leases/commands; simulation time is used for trajectory deadlines.
class GizmoPolicy {
public:
  std::uint64_t epoch() const { return epoch_; }
  bool hasTarget() const { return target_; }
  void invalidate() { target_ = false; ++epoch_; }
  bool accept(std::uint64_t epoch, double now, double timeout) {
    if (epoch != epoch_) return false;
    target_ = true; target_deadline_ = now + timeout; return true;
  }
  bool expired(double now) const { return target_ && now >= target_deadline_; }
  bool acquire(const std::string& owner, double ttl, double now) {
    if (owner.empty() || !std::isfinite(ttl) || ttl <= 0. || ttl > 60.) return false;
    const auto it = leases_.find(owner);
    if (it == leases_.end() || it->second <= now) invalidate();
    leases_[owner] = now + ttl; settling_ = true; return true;
  }
  void release(const std::string& owner) { leases_.erase(owner); }
  void normal(double deadline) { invalidate(); normal_deadline_ = deadline; settling_ = true; }
  void action(bool active) {
    if (active && !action_active_) { invalidate(); settling_ = true; }
    action_active_ = active;
  }
  bool allowed(double steady, double sim, bool settled) {
    for (auto it = leases_.begin(); it != leases_.end();) {
      if (it->second <= steady) it = leases_.erase(it); else ++it;
    }
    if (!leases_.empty() || action_active_ || sim < normal_deadline_) return false;
    if (settling_ && !settled) return false;
    settling_ = false; return true;
  }
private:
  std::uint64_t epoch_{0};
  bool target_{false}, action_active_{false}, settling_{false};
  double target_deadline_{0.}, normal_deadline_{0.};
  std::map<std::string, double> leases_;
};
// A controller action owns the low slot from submission through its terminal
// result. Releasing the mouse or requesting high-priority motion must not
// release this slot before the already dispatched bounded step finishes.
class LowGoalBarrier {
public:
  bool idle() const { return !pending_; }
  bool begin() {
    if (pending_) return false;
    pending_ = true; return true;
  }
  void accepted(bool value) { if (!value) terminal(); }
  void terminal() { pending_ = false; }
private:
  bool pending_{false};
};
}  // namespace iiwa_planning
