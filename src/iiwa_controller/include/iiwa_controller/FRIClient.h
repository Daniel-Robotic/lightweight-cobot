#pragma once

#include <array>
#include <atomic>
#include <mutex>

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

namespace iiwa_controller
{

enum class CommandMode
{
  POSITION,
  TORQUE
};

// Снимок состояния робота захватывается атомарно за один lock в FRI-потоке
// и так же за один lock читается из read() в потоке управления.
struct IIWAStateSnapshot
{
  std::array<double, 7> measured_pos{}; // измеренные позиции суставов [рад]
  std::array<double, 7> measured_tau{}; // измеренные моменты [Нм]
  std::array<double, 7> external_tau{}; // внешние моменты без компенсации модели [Нм]
  std::array<double, 7> ipo_pos{};      // позиция интерполятора [рад], только в Commanding
  double sample_time{0.005};            // период цикла FRI [с]
  KUKA::FRI::EConnectionQuality quality{KUKA::FRI::POOR};
  bool ipo_valid{false};                // в Monitor-режиме IPO недоступна
};

class FRIClient : public KUKA::FRI::LBRClient
{
public:
  static constexpr size_t N_JOINTS = 7;

  explicit FRIClient(CommandMode mode = CommandMode::POSITION);
  ~FRIClient() override = default;

  // Коллбэки FRI SDK, вызываются из friThreadFunc через ClientApplication::step()
  void monitor() override;
  void waitForCommand() override;
  void command() override;
  void onStateChange(
    KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) override;

  // Потокобезопасное API для ros2_control, вызывается из read() и write()
  void setTargetJointPositions(const std::array<double, N_JOINTS> & q);
  void setTargetJointTorques(const std::array<double, N_JOINTS> & tau);
  IIWAStateSnapshot getStateSnapshot() const;
  bool isCommandingActive() const;
  KUKA::FRI::ESessionState getSessionState() const;

private:
  CommandMode cmd_mode_;
  std::atomic<KUKA::FRI::ESessionState> session_state_{KUKA::FRI::IDLE};

  mutable std::mutex data_mutex_;
  std::array<double, N_JOINTS> target_pos_{};
  std::array<double, N_JOINTS> target_tau_{};
  IIWAStateSnapshot snapshot_{};

  // Обновить snapshot_ без поля ipo_pos (в Monitor-режиме getIpoJointPosition() недоступна)
  void captureMonitoringData();
  // Обновить snapshot_ вместе с ipo_pos (в Commanding-режиме)
  void captureCommandingData();
};

}  // namespace iiwa_controller
