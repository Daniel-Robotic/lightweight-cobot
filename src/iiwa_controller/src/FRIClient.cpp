#include "iiwa_controller/FRIClient.h"

#include <cmath>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

namespace iiwa_controller
{

static const char * friStateName(KUKA::FRI::ESessionState s)
{
  switch (s) {
    case KUKA::FRI::IDLE: return "IDLE";
    case KUKA::FRI::MONITORING_WAIT: return "MONITORING_WAIT";
    case KUKA::FRI::MONITORING_READY: return "MONITORING_READY";
    case KUKA::FRI::COMMANDING_WAIT: return "COMMANDING_WAIT";
    case KUKA::FRI::COMMANDING_ACTIVE: return "COMMANDING_ACTIVE";
    default: return "UNKNOWN";
  }
}

FRIClient::FRIClient(CommandMode mode) : cmd_mode_(mode)
{
  target_pos_.fill(0.0);
  target_tau_.fill(0.0);
}

// Вызывается только в Monitor-состояниях.
// В Monitor-режиме getIpoJointPosition() бросает FRIException, поэтому здесь не зовём.
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
  snapshot_.quality = robotState().getConnectionQuality();
  snapshot_.ipo_valid = false;
}

// Вызывается из Commanding-состояний (COMMANDING_WAIT и COMMANDING_ACTIVE).
// В отличие от Monitor, здесь getIpoJointPosition() доступна.
void FRIClient::captureCommandingData()
{
  captureMonitoringData();
  std::memcpy(
    snapshot_.ipo_pos.data(),
    robotState().getIpoJointPosition(), N_JOINTS * sizeof(double));
  snapshot_.ipo_valid = true;
}

// Вызывается в MONITORING_WAIT и MONITORING_READY
void FRIClient::monitor()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  captureMonitoringData();
}

// Вызывается в COMMANDING_WAIT.
// По документации FRI (п. 6.2.2) клиент должен отправлять команды в каждом цикле.
// Переход в COMMANDING_ACTIVE происходит только когда разница между commanded_position
// и IPO_position меньше 0.001 рад для всех суставов.
// Важно эхировать именно IPO-позицию, не measured. Если взять measured,
// статическое отклонение не даст выполниться этому условию.
void FRIClient::waitForCommand()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  captureCommandingData();

  // Инициализируем цель IPO-позицией, иначе до первого write() будем посылать нули.
  std::memcpy(target_pos_.data(), snapshot_.ipo_pos.data(), N_JOINTS * sizeof(double));

  robotCommand().setJointPosition(target_pos_.data());

  if (cmd_mode_ == CommandMode::TORQUE) {
    // Пока контроллер не синхронизирован, момент держим на нуле
    target_tau_.fill(0.0);
    robotCommand().setTorque(target_tau_.data());
  }
}

// Вызывается в COMMANDING_ACTIVE, основной цикл управления
void FRIClient::command()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  captureCommandingData();

  robotCommand().setJointPosition(target_pos_.data());

  if (cmd_mode_ == CommandMode::TORQUE) {
    // В режиме TORQUE позиция работает как feedforward удержания, момент добавляется поверх.
    // Кука выбрасывает CommandInvalidException если отклонение позиции превышает 10 градусов.
    robotCommand().setTorque(target_tau_.data());
  }
}

void FRIClient::onStateChange(
  KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
{
  session_state_.store(newState, std::memory_order_relaxed);

  RCLCPP_INFO(
    rclcpp::get_logger("FRIClient"),
    "FRI смена состояния: %s, теперь %s", friStateName(oldState), friStateName(newState));

  if (newState == KUKA::FRI::IDLE ||
    newState == KUKA::FRI::MONITORING_WAIT ||
    newState == KUKA::FRI::MONITORING_READY)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_tau_.fill(0.0);
    RCLCPP_WARN(
      rclcpp::get_logger("FRIClient"),
      "FRI сессия неактивна, моменты обнулены");
  }
}

void FRIClient::setTargetJointPositions(const std::array<double, N_JOINTS> & q)
{
  // До первой команды контроллера интерфейс содержит NaN.
  // Если отправить NaN роботу в COMMANDING_ACTIVE, получим CK_COMPOUND_RETURN_ERROR.
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
