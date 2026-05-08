#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ros2_control Jazzy 4.44.0+ API
// ВАЖНО: НЕ переопределять export_state_interfaces() / export_command_interfaces() —
// устаревшие конструкторы не регистрируют introspection-callback pal_statistics → segfault.
// Базовый класс создаёт интерфейсы из URDF через on_export_state_interfaces().
// Доступ к данным через handle-API: set_state() / get_command().
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "iiwa_controller/FRIClient.h"

namespace iiwa_controller
{

class IIWAHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IIWAHardwareInterface)

  // Lifecycle

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  // Экспортируем external_torque как "unlisted" интерфейс (не нужно объявлять в URDF).
  // Стандартные интерфейсы (position, velocity, effort) базовый класс создаёт из URDF.
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t N_JOINTS = FRIClient::N_JOINTS;

  // Параметры из <hardware><param> в URDF
  std::string robot_ip_;
  int fri_port_{30200};
  bool simulate_{false};
  std::string cmd_mode_str_{"position"};

  // FRI объекты
  std::unique_ptr<FRIClient> fri_client_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;

  // FRI работает в фоновом потоке: step() блокируется до UDP-пакета,
  // поэтому не нагружает CPU. Синхронизация — через мьютекс FRIClient.
  std::thread fri_thread_;
  std::atomic<bool> fri_running_{false};
  void friThreadFunc();

  // Кэшированные хэндлы интерфейсов состояния (заполняются в on_activate)
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_pos_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_vel_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_eff_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_ext_;

  // Кэшированные хэндлы командных интерфейсов
  std::array<hardware_interface::CommandInterface::SharedPtr, N_JOINTS> h_cmd_pos_;
  std::array<hardware_interface::CommandInterface::SharedPtr, N_JOINTS> h_cmd_eff_;

  // Предыдущие позиции и отфильтрованные скорости (EMA, alpha=0.2)
  std::array<double, N_JOINTS> prev_pos_{};
  std::array<double, N_JOINTS> vel_filtered_{};

  CommandMode parseCommandMode(const std::string & mode_str) const;
};

}  // namespace iiwa_controller
