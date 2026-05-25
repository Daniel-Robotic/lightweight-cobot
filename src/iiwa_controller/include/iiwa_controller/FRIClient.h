#pragma once

#include <array>
#include <atomic>
#include <mutex>

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

namespace iiwa_controller
{


// Снимок состояния робота захватывается атомарно за один lock в FRI-потоке
// и так же за один lock читается из read() в потоке управления.
struct IIWAStateSnapshot
{
  std::array<double, 7> measured_pos{}; // в Commanding = filtered_pos_ (open-loop)
  std::array<double, 7> measured_tau{}; // измеренные моменты [Нм]
  std::array<double, 7> external_tau{}; // внешние моменты без компенсации модели [Нм]
  std::array<double, 7> ipo_pos{};      // позиция интерполятора [рад], только в Commanding
  double sample_time{0.005};            // период цикла FRI [с]
  KUKA::FRI::EConnectionQuality quality{KUKA::FRI::POOR};
  bool ipo_valid{false};                // в Monitor-режиме IPO недоступна
  unsigned int time_stamp_sec{0};       // Unix-время пакета [с]
  unsigned int time_stamp_nano_sec{0};  // наносекундная часть [нс]
};

class FRIClient : public KUKA::FRI::LBRClient
{
public:
  static constexpr size_t N_JOINTS = 7;

  // joint_position_tau — постоянная времени экспоненциального фильтра позиций [с].
  // Аналог joint_position_tau из lbr_fri_ros2_stack (по умолчанию 0.04 с = 40 мс).
  // Сглаживает скачки команд перед отправкой роботу → убирает писк и стук суставов.
  explicit FRIClient(double joint_position_tau = 0.04);
  ~FRIClient() override = default;

  // Коллбэки FRI SDK, вызываются из friThreadFunc через ClientApplication::step()
  void monitor() override;
  void waitForCommand() override;
  void command() override;
  void onStateChange(
    KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) override;

  // Потокобезопасное API для ros2_control, вызывается из read() и write()
  void setTargetJointPositions(const std::array<double, N_JOINTS> & q);
  IIWAStateSnapshot getStateSnapshot() const;
  bool isCommandingActive() const;
  KUKA::FRI::ESessionState getSessionState() const;

private:
  double joint_position_tau_;
  std::atomic<KUKA::FRI::ESessionState> session_state_{KUKA::FRI::IDLE};

  mutable std::mutex data_mutex_;
  std::array<double, N_JOINTS> target_pos_{};
  // Сглаженная позиция, которую реально отправляем роботу.
  // Инициализируется IPO-позицией в waitForCommand(), чтобы не было скачка при старте.
  std::array<double, N_JOINTS> filtered_pos_{};
  IIWAStateSnapshot snapshot_{};

  // Обновить snapshot_ без поля ipo_pos (в Monitor-режиме getIpoJointPosition() недоступна)
  void captureMonitoringData();
  // Обновить snapshot_ вместе с ipo_pos (в Commanding-режиме)
  void captureCommandingData();
};

}  // namespace iiwa_controller
