#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// В Jazzy 4.44.0+ нельзя переопределять export_state_interfaces() и export_command_interfaces().
// Устаревший конструктор не регистрирует introspection-callback pal_statistics,
// из-за чего падает с segfault. Базовый класс сам создаёт интерфейсы из URDF.
// Данные читаем и пишем через handle-API: set_state() и get_command().
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "iiwa_controller/FRIClient.h"

namespace iiwa_controller
{

class IIWAHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IIWAHardwareInterface)

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  // external_torque не объявлен в URDF — регистрируем вручную как unlisted.
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;

  // Полный lifecycle: configure открывает сокет, activate запускает поток,
  // deactivate останавливает поток, cleanup освобождает FRI-объекты.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t N_JOINTS = FRIClient::N_JOINTS;

  // Параметры из секции <hardware><param> в URDF
  std::string robot_ip_;
  int fri_port_{30200};
  bool simulate_{false};
  std::string cmd_mode_str_{"position"};
  double joint_position_tau_{0.04};

  // Объекты FRI SDK
  std::unique_ptr<FRIClient> fri_client_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;

  // FRI работает в отдельном потоке: step() блокируется в recvfrom().
  // read() лишь читает готовый снимок — без блокировки RT-потока.
  std::thread fri_thread_;
  std::atomic<bool> fri_running_{false};
  void friThreadFunc();

  // Хэндлы интерфейсов состояния, заполняются в on_activate
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_pos_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_vel_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_eff_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_ext_;

  // Хэндлы командных интерфейсов
  std::array<hardware_interface::CommandInterface::SharedPtr, N_JOINTS> h_cmd_pos_;
  std::array<hardware_interface::CommandInterface::SharedPtr, N_JOINTS> h_cmd_eff_;

  // Вычисление скорости конечными разностями
  std::array<double, N_JOINTS> prev_pos_{};
  std::array<double, N_JOINTS> velocity_{};
  unsigned int last_ts_sec_{0};
  unsigned int last_ts_nsec_{0};
  bool velocity_initialized_{false};
  void compute_velocity_(const IIWAStateSnapshot & snap);

  // Отслеживание сессии FRI для обнаружения потери управления
  KUKA::FRI::ESessionState previous_session_state_{KUKA::FRI::IDLE};

  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  CommandMode parseCommandMode(const std::string & mode_str) const;
};

}  // namespace iiwa_controller
