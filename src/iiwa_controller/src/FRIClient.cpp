#include "iiwa_controller/FRIClient.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace iiwa_controller
{
using namespace KUKA::FRI;

FRIClient::FRIClient()
{
  lower_.fill(-std::numeric_limits<double>::infinity());
  upper_.fill(std::numeric_limits<double>::infinity());
  max_velocity_.fill(std::numeric_limits<double>::infinity());
}

void FRIClient::setLimits(const Joints & lower, const Joints & upper, const Joints & velocity)
{
  lower_ = lower;
  upper_ = upper;
  max_velocity_ = velocity;
}

void FRIClient::captureData(bool commanding)
{
  const auto & state = robotState();
  std::copy_n(state.getMeasuredJointPosition(), N_JOINTS, current_.measured_pos.begin());
  std::copy_n(state.getMeasuredTorque(), N_JOINTS, current_.measured_tau.begin());
  std::copy_n(state.getExternalTorque(), N_JOINTS, current_.external_tau.begin());
  current_.sample_time = state.getSampleTime();
  current_.session = state.getSessionState();
  current_.quality = state.getConnectionQuality();
  current_.safety = state.getSafetyState();
  current_.drives = state.getDriveState();
  current_.operation = state.getOperationMode();
  current_.control = state.getControlMode();
  current_.command_mode = commanding ? state.getClientCommandMode() : NO_COMMAND_MODE;
  current_.overlay = commanding ? state.getOverlayType() : NO_OVERLAY;
  current_.tracking_performance = commanding ? state.getTrackingPerformance() : 0.0;
  current_.ipo_valid = commanding;
  if (commanding) {
    std::copy_n(state.getIpoJointPosition(), N_JOINTS, current_.ipo_pos.begin());
  }
  const auto sec = state.getTimestampSec();
  const auto nsec = state.getTimestampNanoSec();
  if (current_.valid && (sec < current_.time_stamp_sec ||
    (sec == current_.time_stamp_sec && nsec <= current_.time_stamp_nano_sec)))
  {
    throw std::runtime_error("FRI timestamp did not advance");
  }
  if (nsec >= 1000000000 || !std::isfinite(current_.sample_time) || current_.sample_time <= 0.0) {
    throw std::runtime_error("Invalid FRI timestamp/sample period");
  }
  for (size_t i = 0; i < N_JOINTS; ++i) {
    if (!std::isfinite(current_.measured_pos[i]) || !std::isfinite(current_.measured_tau[i]) ||
      !std::isfinite(current_.external_tau[i]) ||
      (commanding && !std::isfinite(current_.ipo_pos[i])))
    {
      throw std::runtime_error("Non-finite FRI telemetry");
    }
  }
  current_.time_stamp_sec = sec;
  current_.time_stamp_nano_sec = nsec;
  current_.received_at = std::chrono::steady_clock::now();
  current_.valid = true;
}

void FRIClient::publishState()
{
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (lock.owns_lock()) {snapshot_ = current_;}
}

void FRIClient::validateCommanding() const
{
  if (current_.command_mode != POSITION || current_.overlay != JOINT ||
    (current_.control != POSITION_CONTROL_MODE && current_.control != JOINT_IMP_CONTROL_MODE &&
    current_.control != CART_IMP_CONTROL_MODE))
  {
    throw std::runtime_error("Expected FRI POSITION command mode with JOINT overlay");
  }
  if (current_.quality < GOOD || current_.safety != NORMAL_OPERATION ||
    current_.drives != ACTIVE)
  {
    throw std::runtime_error("FRI commanding unavailable: quality, safety or drives");
  }
}

void FRIClient::monitor()
{
  captureData(false);
  LBRClient::monitor();
  initialized_ = false;
  publishState();
}

void FRIClient::waitForCommand()
{
  captureData(true);
  validateCommanding();
  LBRClient::waitForCommand();  // SDK 1.16 mirrors IPO, not measured position.
  sent_pos_ = target_pos_ = current_.ipo_pos;
  initialized_ = true;
  last_command_at_ = current_.received_at;
  publishState();
}

void FRIClient::command()
{
  captureData(true);
  validateCommanding();
  if (!initialized_) {
    sent_pos_ = target_pos_ = current_.ipo_pos;
    last_command_at_ = current_.received_at;
    initialized_ = true;
  }
  {
    std::unique_lock<std::mutex> lock(command_mutex_, std::try_to_lock);
    if (lock.owns_lock() && requested_ && requested_at_ > last_command_at_) {
      target_pos_ = requested_pos_;
      last_command_at_ = requested_at_;
    }
  }
  // Local watchdog: a stalled ros2_control loop must not leave a moving target active.
  if (current_.received_at - last_command_at_ > std::chrono::milliseconds(100)) {
    throw std::runtime_error("FRI command watchdog expired");
  }
  const double dt = current_.sample_time;
  for (size_t i = 0; i < N_JOINTS; ++i) {
    if (!std::isfinite(target_pos_[i]) || target_pos_[i] < lower_[i] ||
      target_pos_[i] > upper_[i])
    {
      throw std::runtime_error("FRI position command outside configured limits");
    }
    const double step = target_pos_[i] - sent_pos_[i];
    sent_pos_[i] += std::clamp(step, -max_velocity_[i] * dt, max_velocity_[i] * dt);
  }
  robotCommand().setJointPosition(sent_pos_.data());
  publishState();
}

void FRIClient::onStateChange(ESessionState oldState, ESessionState newState)
{
  session_state_.store(newState);
  if (oldState == COMMANDING_ACTIVE && newState != COMMANDING_ACTIVE) {
    throw std::runtime_error("FRI left COMMANDING_ACTIVE; explicit recovery required");
  }
}

void FRIClient::setTargetJointPositions(const Joints & q)
{
  for (size_t i = 0; i < N_JOINTS; ++i) {
    if (!std::isfinite(q[i]) || q[i] < lower_[i] || q[i] > upper_[i]) {
      throw std::invalid_argument("Invalid FRI position command");
    }
  }
  std::unique_lock<std::mutex> lock(command_mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    requested_pos_ = q;
    requested_at_ = std::chrono::steady_clock::now();
    requested_ = true;
  }
}

IIWAStateSnapshot FRIClient::getStateSnapshot() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return snapshot_;
}

bool FRIClient::tryGetStateSnapshot(IIWAStateSnapshot & snapshot) const
{
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {return false;}
  snapshot = snapshot_;
  return true;
}

bool FRIClient::isCommandingActive() const
{
  return getSessionState() == COMMANDING_ACTIVE;
}

ESessionState FRIClient::getSessionState() const
{
  return session_state_.load();
}
}  // namespace iiwa_controller
