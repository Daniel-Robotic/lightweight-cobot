#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

namespace iiwa_controller
{
struct IIWAStateSnapshot
{
  std::array<double, 7> measured_pos{};  // encoder position [rad], never a command
  std::array<double, 7> measured_tau{};  // measured torque [Nm]
  std::array<double, 7> external_tau{};  // estimated external torque [Nm]
  std::array<double, 7> ipo_pos{};       // interpolator position [rad]
  std::array<double, 7> command_pos{};   // position sent in the current FRI reply [rad]
  double sample_time{0.0};
  KUKA::FRI::ESessionState session{KUKA::FRI::IDLE};
  KUKA::FRI::EConnectionQuality quality{KUKA::FRI::POOR};
  KUKA::FRI::ESafetyState safety{KUKA::FRI::NORMAL_OPERATION};
  KUKA::FRI::EDriveState drives{KUKA::FRI::OFF};
  KUKA::FRI::EOperationMode operation{KUKA::FRI::TEST_MODE_1};
  KUKA::FRI::EControlMode control{KUKA::FRI::NO_CONTROL};
  KUKA::FRI::EClientCommandMode command_mode{KUKA::FRI::NO_COMMAND_MODE};
  KUKA::FRI::EOverlayType overlay{KUKA::FRI::NO_OVERLAY};
  double tracking_performance{0.0};
  bool valid{false};
  bool ipo_valid{false};
  unsigned int time_stamp_sec{0};
  unsigned int time_stamp_nano_sec{0};
  std::chrono::steady_clock::time_point received_at{};
};

class FRIClient : public KUKA::FRI::LBRClient
{
public:
  static constexpr size_t N_JOINTS = KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
  using Joints = std::array<double, N_JOINTS>;
  FRIClient();
  void monitor() override;
  void waitForCommand() override;
  void command() override;
  void onStateChange(KUKA::FRI::ESessionState oldState,
    KUKA::FRI::ESessionState newState) override;

  // Configure before starting the FRI thread; limits come from the robot URDF.
  void setLimits(const Joints & lower, const Joints & upper, const Joints & velocity);
  void setExpectedSampleTime(double seconds) {expected_sample_time_ = seconds;}
  void setTargetJointPositions(const Joints & q);
  IIWAStateSnapshot getStateSnapshot() const;  // lifecycle/non-RT use
  bool tryGetStateSnapshot(IIWAStateSnapshot & snapshot) const;
  bool isCommandingActive() const;
  KUKA::FRI::ESessionState getSessionState() const;

private:
  std::atomic<KUKA::FRI::ESessionState> session_state_{KUKA::FRI::IDLE};
  mutable std::mutex state_mutex_;
  std::mutex command_mutex_;
  Joints target_pos_{};
  Joints requested_pos_{};
  Joints sent_pos_{};
  Joints lower_, upper_, max_velocity_;
  bool initialized_{false};
  bool requested_{false};
  double expected_sample_time_{0.0};
  std::chrono::steady_clock::time_point requested_at_{};
  std::chrono::steady_clock::time_point last_command_at_{};
  IIWAStateSnapshot snapshot_{};  // published under state_mutex_
  IIWAStateSnapshot current_{};   // owned exclusively by FRI thread
  void captureData(bool commanding);
  void publishState();
  void validateCommanding() const;
};
}  // namespace iiwa_controller
