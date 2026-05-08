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

// Атомарный снимок всего FRI-состояния — захватывается за один lock в FRI-потоке,
// читается из ros2_control read() за один lock.
struct IIWAStateSnapshot
{
  std::array<double, 7> measured_pos{};  // Измеренные позиции [рад]
  std::array<double, 7> measured_tau{};  // Измеренные моменты [Нм]
  std::array<double, 7> external_tau{};  // Внешние моменты (без модели робота) [Нм]
  std::array<double, 7> ipo_pos{};       // IPO-позиция интерполятора [рад] (только в Commanding)
  double sample_time{0.005};             // Период цикла FRI [с]
  KUKA::FRI::EConnectionQuality quality{KUKA::FRI::POOR};
  bool ipo_valid{false};  // IPO недоступна в Monitor-режиме
};

class FRIClient : public KUKA::FRI::LBRClient
{
public:
  static constexpr size_t N_JOINTS = 7;

  explicit FRIClient(CommandMode mode = CommandMode::POSITION);
  ~FRIClient() override = default;

  // Callbacks ClientApplication::step() → вызываются из FRI-потока
  void monitor() override;
  void waitForCommand() override;
  void command() override;
  void onStateChange(
    KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) override;

  // Thread-safe API для ros2_control (вызывается из read/write в control-потоке)
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

  // Обновить snapshot_ без IPO (Monitor-режим, где getIpoJointPosition() бросает исключение)
  void captureMonitoringData();
  // Обновить snapshot_ с IPO (Commanding-режим)
  void captureCommandingData();
};

}  // namespace iiwa_controller
