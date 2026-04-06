// ============================================================
// IIWAHardwareInterface.hpp
// ROS2 hardware_interface::SystemInterface для KUKA iiwa 7.
//
//   on_init() — читаем параметры из URDF/XACRO
//   on_configure() — (опционально)
//   on_activate() — устанавливаем FRI соединение
//   on_deactivate() — разрываем FRI соединение
//   read() — копируем данные FRI → интерфейсы состояния
//   write() — копируем команды интерфейсов → FRI
// ============================================================
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

// ROS2 hardware_interface
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Наш FRI клиент
#include "iiwa_controller/FRIClient.h"

namespace iiwa_controller
{

class IIWAHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // Макрос ROS2 для shared_ptr / weak_ptr
  RCLCPP_SHARED_PTR_DEFINITIONS(IIWAHardwareInterface)

  // Lifecycle callbacks (порядок вызова гарантирован ROS2)
  /// Инициализация: читаем параметры из <hardware><param> в URDF
  CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;

  /// Экспорт интерфейсов состояния: position, velocity, effort
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  /// Экспорт командных интерфейсов: position (и/или effort)
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /// Активация: открываем UDP соединение с роботом
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  /// Деактивация: закрываем соединение, сбрасываем команды
  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  /// Чтение данных с робота (вызывается перед каждым шагом контроллера)
  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  /// Запись команд на робот (вызывается после каждого шага контроллера)
  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  // Параметры из URDF <hardware><param>
  std::string robot_ip_;  //IP адрес контроллера KUKA
  int fri_port_{30200}; // UDP порт FRI (по умолчанию 30200)
  bool simulate_{false}; // Режим симуляции (без реального робота)
  std::string cmd_mode_str_{"position"}; // "position" или "torque"

  // FRI объекты
  std::unique_ptr<FRIClient> fri_client_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;

  // FRI выполняется в отдельном фоновом потоке,
  // чтобы не блокировать ros2_control loop.
  std::thread fri_thread_;
  std::atomic<bool> fri_running_{false};

  /// Функция фонового потока: крутит app_->step() в цикле
  void friThreadFunc();

  // Данные интерфейсов ros2_control
  // (ros2_control обращается к ним через указатели из export_*)
  static constexpr size_t N_JOINTS = FRIClient::N_JOINTS;

  std::vector<double> hw_pos_;   // Измеренные позиции [рад]
  std::vector<double> hw_vel_;   // Расчётные скорости [рад/с]
  std::vector<double> hw_eff_;   // Измеренные моменты [Нм]

  std::vector<double> cmd_pos_;  // Команда позиции [рад]
  std::vector<double> cmd_eff_;  // Команда момента [Нм]

  std::vector<double> prev_pos_; // Предыдущая позиция для расчёта velocity

  // Вспомогательный метод
  /// Создаёт объект CommandMode из строки параметра
  CommandMode parseCommandMode(const std::string& mode_str) const;
};

}  // namespace iiwa_controller