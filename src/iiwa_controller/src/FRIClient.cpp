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

FRIClient::FRIClient(double joint_position_tau)
: joint_position_tau_(joint_position_tau)
{
  target_pos_.fill(0.0);
  filtered_pos_.fill(0.0);
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
  snapshot_.time_stamp_sec = robotState().getTimestampSec();
  snapshot_.time_stamp_nano_sec = robotState().getTimestampNanoSec();
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
  // Open-loop: JTC видит filtered_pos_ как «измеренную» позицию — как в lbr_fri_ros2_stack.
  // Благодаря этому JTC не видит расхождения и не генерирует коррекций.
  snapshot_.measured_pos = filtered_pos_;
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

  // Инициализируем цель и фильтр IPO-позицией.
  // Фильтр стартует с IPO — это гарантирует нулевой скачок при переходе в COMMANDING_ACTIVE.
  std::memcpy(target_pos_.data(), snapshot_.ipo_pos.data(), N_JOINTS * sizeof(double));
  std::memcpy(filtered_pos_.data(), snapshot_.ipo_pos.data(), N_JOINTS * sizeof(double));

  robotCommand().setJointPosition(filtered_pos_.data());
}

// Вызывается в COMMANDING_ACTIVE, основной цикл управления
void FRIClient::command()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // EMA-фильтр применяется ДО захвата снимка — тогда snapshot_.measured_pos = filtered_pos_
  // будет содержать то, что реально отправлено роботу в этом цикле (не прошлом).
  // Это соответствует lbr_fri_ros2_stack: снимок захватывается post-EMA.
  const double dt = robotState().getSampleTime();
  const double alpha = (joint_position_tau_ > 0.0) ? dt / (joint_position_tau_ + dt) : 1.0;
  for (size_t i = 0; i < N_JOINTS; ++i) {
    filtered_pos_[i] = alpha * target_pos_[i] + (1.0 - alpha) * filtered_pos_[i];
  }

  robotCommand().setJointPosition(filtered_pos_.data());

  // Захватываем снимок ПОСЛЕ EMA: measured_pos = filtered_pos_ = что робот только что получил.
  captureCommandingData();
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
