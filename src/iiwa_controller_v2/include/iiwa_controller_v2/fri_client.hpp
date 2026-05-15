#pragma once

#include <array>
#include <atomic>
#include <mutex>

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

namespace iiwa_controller_v2
{

enum class CommandMode
{
  POSITION,
  TORQUE
};

// Atomic snapshot of robot state, captured each FRI cycle and consumed by read().
struct IIWAStateSnapshot
{
  std::array<double, 7> measured_pos{};    // open-loop: = filtered_pos_ in COMMANDING_ACTIVE
  std::array<double, 7> ipo_pos{};         // interpolator position (valid only in COMMANDING)
  std::array<double, 7> measured_tau{};    // measured joint torques [Nm]
  std::array<double, 7> commanded_tau{};   // torques sent to robot in previous cycle [Nm]
  std::array<double, 7> external_tau{};    // estimated external torques (gravity-compensated) [Nm]
  double sample_time{0.005};
  KUKA::FRI::EConnectionQuality connection_quality{KUKA::FRI::POOR};
  KUKA::FRI::ESessionState session_state{KUKA::FRI::IDLE};
  bool ipo_valid{false};
  unsigned int time_stamp_sec{0};
  unsigned int time_stamp_nano_sec{0};
};

class FRIClient : public KUKA::FRI::LBRClient
{
public:
  static constexpr size_t N_JOINTS = 7;

  explicit FRIClient(CommandMode mode = CommandMode::POSITION, double joint_position_tau = 0.04);
  ~FRIClient() override = default;

  // FRI SDK callbacks — called from friThreadFunc via ClientApplication::step()
  void monitor() override;
  void waitForCommand() override;
  void command() override;
  void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) override;

  // Thread-safe API for the ros2_control loop
  void setTargetJointPositions(const std::array<double, N_JOINTS> & q);
  void setTargetJointTorques(const std::array<double, N_JOINTS> & tau);
  IIWAStateSnapshot getStateSnapshot() const;
  KUKA::FRI::ESessionState getSessionState() const;

private:
  CommandMode cmd_mode_;
  double joint_position_tau_;
  std::atomic<KUKA::FRI::ESessionState> session_state_{KUKA::FRI::IDLE};

  mutable std::mutex data_mutex_;
  std::array<double, N_JOINTS> target_pos_{};
  std::array<double, N_JOINTS> target_tau_{};
  // Filtered position actually sent to the robot; initialised from IPO in waitForCommand().
  std::array<double, N_JOINTS> filtered_pos_{};
  IIWAStateSnapshot snapshot_{};

  // Monitor state: getIpoJointPosition() is NOT available here.
  void captureMonitoringData();
  // COMMANDING states: IPO position is available and snapshot_.measured_pos = filtered_pos_.
  void captureCommandingData();
};

}  // namespace iiwa_controller_v2
