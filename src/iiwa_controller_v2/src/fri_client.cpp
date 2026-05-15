#include "iiwa_controller_v2/fri_client.hpp"

#include <cmath>
#include <cstring>

#include "rclcpp/rclcpp.hpp"

namespace iiwa_controller_v2
{

static const char * sessionStateName(KUKA::FRI::ESessionState s)
{
  switch (s) {
    case KUKA::FRI::IDLE:             return "IDLE";
    case KUKA::FRI::MONITORING_WAIT:  return "MONITORING_WAIT";
    case KUKA::FRI::MONITORING_READY: return "MONITORING_READY";
    case KUKA::FRI::COMMANDING_WAIT:  return "COMMANDING_WAIT";
    case KUKA::FRI::COMMANDING_ACTIVE:return "COMMANDING_ACTIVE";
    default:                          return "UNKNOWN";
  }
}

FRIClient::FRIClient(CommandMode mode, double joint_position_tau)
: cmd_mode_(mode), joint_position_tau_(joint_position_tau)
{
  target_pos_.fill(0.0);
  target_tau_.fill(0.0);
  filtered_pos_.fill(0.0);
}

void FRIClient::captureMonitoringData()
{
  // getIpoJointPosition() throws in Monitor states — do NOT call it here.
  std::memcpy(snapshot_.measured_pos.data(),
              robotState().getMeasuredJointPosition(), N_JOINTS * sizeof(double));
  std::memcpy(snapshot_.measured_tau.data(),
              robotState().getMeasuredTorque(), N_JOINTS * sizeof(double));
  std::memcpy(snapshot_.commanded_tau.data(),
              robotState().getCommandedTorque(), N_JOINTS * sizeof(double));
  std::memcpy(snapshot_.external_tau.data(),
              robotState().getExternalTorque(), N_JOINTS * sizeof(double));
  snapshot_.sample_time        = robotState().getSampleTime();
  snapshot_.connection_quality = robotState().getConnectionQuality();
  snapshot_.session_state      = robotState().getSessionState();
  snapshot_.ipo_valid          = false;
  snapshot_.time_stamp_sec     = robotState().getTimestampSec();
  snapshot_.time_stamp_nano_sec= robotState().getTimestampNanoSec();
}

void FRIClient::captureCommandingData()
{
  captureMonitoringData();
  std::memcpy(snapshot_.ipo_pos.data(),
              robotState().getIpoJointPosition(), N_JOINTS * sizeof(double));
  snapshot_.ipo_valid = true;
  // Open-loop: expose the filtered position as "measured" so JTC sees no error.
  snapshot_.measured_pos = filtered_pos_;
}

void FRIClient::monitor()
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  captureMonitoringData();
}

void FRIClient::waitForCommand()
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  captureCommandingData();

  // Initialise both target and filter from IPO to avoid a step on the first command cycle.
  std::memcpy(target_pos_.data(),   snapshot_.ipo_pos.data(), N_JOINTS * sizeof(double));
  std::memcpy(filtered_pos_.data(), snapshot_.ipo_pos.data(), N_JOINTS * sizeof(double));

  robotCommand().setJointPosition(filtered_pos_.data());

  if (cmd_mode_ == CommandMode::TORQUE) {
    target_tau_.fill(0.0);
    robotCommand().setTorque(target_tau_.data());
  }
}

void FRIClient::command()
{
  std::lock_guard<std::mutex> lk(data_mutex_);

  // EMA filter applied BEFORE snapshot so measured_pos = what we actually sent this cycle.
  const double dt    = robotState().getSampleTime();
  const double alpha = (joint_position_tau_ > 0.0) ? dt / (joint_position_tau_ + dt) : 1.0;
  for (std::size_t i = 0; i < N_JOINTS; ++i) {
    filtered_pos_[i] = alpha * target_pos_[i] + (1.0 - alpha) * filtered_pos_[i];
  }

  robotCommand().setJointPosition(filtered_pos_.data());

  if (cmd_mode_ == CommandMode::TORQUE) {
    robotCommand().setTorque(target_tau_.data());
  }

  captureCommandingData();
}

void FRIClient::onStateChange(
  KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
{
  session_state_.store(newState, std::memory_order_relaxed);

  RCLCPP_INFO(
    rclcpp::get_logger("iiwa_controller_v2"),
    "FRI state change: %s → %s", sessionStateName(oldState), sessionStateName(newState));

  // Safety: zero torques whenever we leave COMMANDING states.
  if (newState == KUKA::FRI::IDLE ||
      newState == KUKA::FRI::MONITORING_WAIT ||
      newState == KUKA::FRI::MONITORING_READY)
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    target_tau_.fill(0.0);
    RCLCPP_WARN(
      rclcpp::get_logger("iiwa_controller_v2"),
      "FRI left commanding — torque targets reset to zero");
  }
}

void FRIClient::setTargetJointPositions(const std::array<double, N_JOINTS> & q)
{
  for (const auto v : q) {
    if (!std::isfinite(v)) { return; }
  }
  std::lock_guard<std::mutex> lk(data_mutex_);
  target_pos_ = q;
}

void FRIClient::setTargetJointTorques(const std::array<double, N_JOINTS> & tau)
{
  for (const auto v : tau) {
    if (!std::isfinite(v)) { return; }
  }
  std::lock_guard<std::mutex> lk(data_mutex_);
  target_tau_ = tau;
}

IIWAStateSnapshot FRIClient::getStateSnapshot() const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  return snapshot_;
}

KUKA::FRI::ESessionState FRIClient::getSessionState() const
{
  return session_state_.load(std::memory_order_relaxed);
}

}  // namespace iiwa_controller_v2
