#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// Jazzy 4.48: framework-managed interfaces and handle API.
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
  ~IIWAHardwareInterface() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  // external_torque не объявлен в URDF — регистрируем вручную как unlisted.
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;

  // Полный lifecycle: configure создаёт SDK, activate открывает сокет и запускает поток,
  // deactivate останавливает поток, cleanup освобождает FRI-объекты.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

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
  // Объекты FRI SDK
  std::unique_ptr<FRIClient> fri_client_;
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;

  // FRI работает в отдельном потоке: step() блокируется в recvfrom().
  // read() лишь читает готовый снимок — без блокировки RT-потока.
  std::thread fri_thread_;
  std::atomic<bool> fri_running_{false};
  void friThreadFunc();
  void stopFRI();
  void releaseFRI();
  std::atomic<bool> fri_fault_{false};
  bool active_{false};
  IIWAStateSnapshot last_snapshot_{};
  std::array<double, N_JOINTS> lower_, upper_, max_velocity_;

  // Хэндлы интерфейсов состояния, заполняются в on_activate
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_pos_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_vel_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_eff_;
  std::array<hardware_interface::StateInterface::SharedPtr, N_JOINTS> h_ext_;

  std::array<hardware_interface::CommandInterface::SharedPtr, N_JOINTS> h_cmd_pos_;

  // Вычисление скорости по измеренной позиции и меткам времени FRI.
  std::array<double, N_JOINTS> prev_pos_{};
  std::array<double, N_JOINTS> velocity_{};        // измеренная скорость [рад/с]
  unsigned int last_ts_sec_{0};
  unsigned int last_ts_nsec_{0};
  bool velocity_initialized_{false};
  void compute_velocity_(const IIWAStateSnapshot & snap);



};

}  // namespace iiwa_controller
