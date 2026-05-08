#include "iiwa_controller/FRIClient.h"

#include <cmath>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

namespace iiwa_controller
{

static const char * friStateName(KUKA::FRI::ESessionState s)
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

FRIClient::FRIClient(CommandMode mode) : cmd_mode_(mode)
{
  target_pos_.fill(0.0);
  target_tau_.fill(0.0);
}

// Вызывается ТОЛЬКО из Monitor-состояний.
// getIpoJointPosition() в Monitor-режиме бросает FRIException → не вызываем.
void FRIClient::captureMonitoringData()
{
  std::memcpy(
    snapshot_.measured_pos.data(),
    robotState().getMeasuredJointPosition(), N_JOINTS * sizeof(double));
  std::memcpy(
    snapshot_.measured_tau.data(),
    robotState().getMeasuredTorque(), N_JOINTS * sizeof(double));
  std::memcpy(
    snapshot_.external_tau.data(),
    robotState().getExternalTorque(), N_JOINTS * sizeof(double));
  snapshot_.sample_time = robotState().getSampleTime();
  snapshot_.quality     = robotState().getConnectionQuality();
  snapshot_.ipo_valid   = false;
}

// Вызывается из Commanding-состояний (COMMANDING_WAIT и COMMANDING_ACTIVE).
// getIpoJointPosition() здесь доступна.
void FRIClient::captureCommandingData()
{
  captureMonitoringData();
  std::memcpy(
    snapshot_.ipo_pos.data(),
    robotState().getIpoJointPosition(), N_JOINTS * sizeof(double));
  snapshot_.ipo_valid = true;
}

// MONITORING_WAIT / MONITORING_READY
void FRIClient::monitor()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  captureMonitoringData();
}

// COMMANDING_WAIT
// FRI-документация §6.2.2: клиент ОБЯЗАН отправлять команды в каждом цикле.
// Переход COMMANDING_WAIT → COMMANDING_ACTIVE происходит когда:
//   |IPO_position[j] - commanded_position[j]| < 0.001 рад  (для всех j)
// КРИТИЧЕСКИ ВАЖНО: эхировать IPO-позицию, а не measured-позицию!
// Если эхировать measured, статическое отклонение от IPO заблокирует переход.
void FRIClient::waitForCommand()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  captureCommandingData();

  // Инициализируем целевую позицию IPO-позицией.
  // ros2_control::write() перезапишет её в следующем цикле командой контроллера.
  // Важно: до первого write() мы должны эхировать IPO, а не 0.
  std::memcpy(target_pos_.data(), snapshot_.ipo_pos.data(), N_JOINTS * sizeof(double));

  robotCommand().setJointPosition(target_pos_.data());

  if (cmd_mode_ == CommandMode::TORQUE) {
    // В COMMANDING_WAIT момент обнуляем — контроллер ещё не синхронизирован
    target_tau_.fill(0.0);
    robotCommand().setTorque(target_tau_.data());
  }
}

// COMMANDING_ACTIVE — основной цикл управления
void FRIClient::command()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  captureCommandingData();

  robotCommand().setJointPosition(target_pos_.data());

  if (cmd_mode_ == CommandMode::TORQUE) {
    // TORQUE-режим: позиция = feedforward удержания, момент = дополнительный overlay.
    // Кука ограничивает: отклонение позиции > ±10° → CommandInvalidException.
    robotCommand().setTorque(target_tau_.data());
  }
}

void FRIClient::onStateChange(
  KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
{
  session_state_.store(newState, std::memory_order_relaxed);

  RCLCPP_INFO(
    rclcpp::get_logger("FRIClient"),
    "[FRI] %s → %s", friStateName(oldState), friStateName(newState));

  if (newState == KUKA::FRI::IDLE || newState == KUKA::FRI::MONITORING_WAIT) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_tau_.fill(0.0);
    RCLCPP_WARN(
      rclcpp::get_logger("FRIClient"),
      "[FRI] Сессия неактивна — моменты обнулены для безопасности");
  }
}

// Thread-safe сеттеры/геттеры

void FRIClient::setTargetJointPositions(const std::array<double, N_JOINTS> & q)
{
  // Защита: командный интерфейс может содержать NaN до первой команды контроллера.
  // Отправка NaN в COMMANDING_ACTIVE → немедленный CK_COMPOUND_RETURN_ERROR.
  for (const auto & v : q) {
    if (!std::isfinite(v)) {
      return;
    }
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  target_pos_ = q;
}

void FRIClient::setTargetJointTorques(const std::array<double, N_JOINTS> & tau)
{
  for (const auto & v : tau) {
    if (!std::isfinite(v)) {
      return;
    }
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  target_tau_ = tau;
}

IIWAStateSnapshot FRIClient::getStateSnapshot() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return snapshot_;
}

bool FRIClient::isCommandingActive() const
{
  return session_state_.load(std::memory_order_relaxed) == KUKA::FRI::COMMANDING_ACTIVE;
}

KUKA::FRI::ESessionState FRIClient::getSessionState() const
{
  return session_state_.load(std::memory_order_relaxed);
}

}  // namespace iiwa_controller
