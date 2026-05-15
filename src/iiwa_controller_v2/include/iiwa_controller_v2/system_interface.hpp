#pragma once

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "iiwa_controller_v2/command_guard.hpp"
#include "iiwa_controller_v2/fri_client.hpp"
#include "iiwa_controller_v2/system_interface_type_values.hpp"

namespace iiwa_controller_v2
{

class SystemInterface : public hardware_interface::SystemInterface
{
protected:
  // ── Parameters ─────────────────────────────────────────────────────────────
  struct Parameters
  {
    std::string robot_ip{"192.170.10.2"};
    int         fri_port{30200};
    bool        simulate{false};
    std::string command_mode{"position"};
    double      joint_position_tau{0.04};
    bool        open_loop{true};   // JTC sees filtered_pos, not measured_pos
    int         rt_prio{80};
  };

  // ── Command interface handles ───────────────────────────────────────────────
  struct CommandInterfaceHandles
  {
    std::array<hardware_interface::CommandInterface::SharedPtr, FRIClient::N_JOINTS>
      joint_position, torque;

    void populate(const hardware_interface::SystemInterface & si,
                  const hardware_interface::HardwareInfo & info)
    {
      for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
        const auto & jn = info.joints[i].name;
        joint_position[i] = si.get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
        torque[i]         = si.get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);
      }
    }

    void nan_interfaces() const
    {
      for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
        std::ignore = joint_position[i]->set_value(std::numeric_limits<double>::quiet_NaN());
        std::ignore = torque[i]->set_value(std::numeric_limits<double>::quiet_NaN());
      }
    }

    void pull(std::array<double, FRIClient::N_JOINTS> & pos_cmd,
              std::array<double, FRIClient::N_JOINTS> & tau_cmd) const
    {
      for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
        pos_cmd[i] = joint_position[i]->get_optional().value_or(
                       std::numeric_limits<double>::quiet_NaN());
        tau_cmd[i] = torque[i]->get_optional().value_or(
                       std::numeric_limits<double>::quiet_NaN());
      }
    }
  };

  // ── State interface handles ─────────────────────────────────────────────────
  struct StateInterfaceHandles
  {
    // Standard per-joint
    std::array<hardware_interface::StateInterface::SharedPtr, FRIClient::N_JOINTS>
      position, velocity, effort;
    // Extended per-joint (registered as unlisted)
    std::array<hardware_interface::StateInterface::SharedPtr, FRIClient::N_JOINTS>
      external_torque, commanded_torque, ipo_joint_position;
    // Auxiliary (robot-level, registered as unlisted)
    hardware_interface::StateInterface::SharedPtr
      sample_time, session_state, connection_quality,
      time_stamp_sec, time_stamp_nano_sec;

    void populate(const hardware_interface::SystemInterface & si,
                  const hardware_interface::HardwareInfo & info)
    {
      for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
        const auto & jn = info.joints[i].name;
        position[i]          = si.get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
        velocity[i]          = si.get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_VELOCITY);
        effort[i]            = si.get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);
        external_torque[i]   = si.get_state_interface_handle(jn + "/" + HW_IF_EXTERNAL_TORQUE);
        commanded_torque[i]  = si.get_state_interface_handle(jn + "/" + HW_IF_COMMANDED_TORQUE);
        ipo_joint_position[i]= si.get_state_interface_handle(jn + "/" + HW_IF_IPO_JOINT_POSITION);
      }
      const std::string aux = std::string(HW_IF_AUXILIARY_PREFIX) + "/";
      sample_time         = si.get_state_interface_handle(aux + HW_IF_SAMPLE_TIME);
      session_state       = si.get_state_interface_handle(aux + HW_IF_SESSION_STATE);
      connection_quality  = si.get_state_interface_handle(aux + HW_IF_CONNECTION_QUALITY);
      time_stamp_sec      = si.get_state_interface_handle(aux + HW_IF_TIME_STAMP_SEC);
      time_stamp_nano_sec = si.get_state_interface_handle(aux + HW_IF_TIME_STAMP_NANO_SEC);
    }

    void nan_interfaces() const
    {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
        std::ignore = position[i]->set_value(nan);
        std::ignore = velocity[i]->set_value(nan);
        std::ignore = effort[i]->set_value(nan);
        std::ignore = external_torque[i]->set_value(nan);
        std::ignore = commanded_torque[i]->set_value(nan);
        std::ignore = ipo_joint_position[i]->set_value(nan);
      }
      std::ignore = sample_time->set_value(nan);
      std::ignore = session_state->set_value(nan);
      std::ignore = connection_quality->set_value(nan);
      std::ignore = time_stamp_sec->set_value(nan);
      std::ignore = time_stamp_nano_sec->set_value(nan);
    }

    void push(const IIWAStateSnapshot & snap,
              const std::array<double, FRIClient::N_JOINTS> & vel) const
    {
      for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
        std::ignore = position[i]->set_value(snap.measured_pos[i]);
        std::ignore = velocity[i]->set_value(vel[i]);
        std::ignore = effort[i]->set_value(snap.measured_tau[i]);
        std::ignore = external_torque[i]->set_value(snap.external_tau[i]);
        std::ignore = commanded_torque[i]->set_value(snap.commanded_tau[i]);
        std::ignore = ipo_joint_position[i]->set_value(snap.ipo_pos[i]);
      }
      std::ignore = sample_time->set_value(snap.sample_time);
      std::ignore = session_state->set_value(static_cast<double>(snap.session_state));
      std::ignore = connection_quality->set_value(static_cast<double>(snap.connection_quality));
      std::ignore = time_stamp_sec->set_value(static_cast<double>(snap.time_stamp_sec));
      std::ignore = time_stamp_nano_sec->set_value(static_cast<double>(snap.time_stamp_nano_sec));
    }
  };

public:
  SystemInterface() = default;

  // ── Lifecycle ───────────────────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  bool parse_parameters_();

  // Returns true when the robot leaves COMMANDING_ACTIVE unexpectedly.
  bool exit_commanding_active_(KUKA::FRI::ESessionState previous,
                               KUKA::FRI::ESessionState current);

  void friThreadFunc();

  // Compute finite-difference velocity from FRI timestamps.
  void compute_velocity_(const IIWAStateSnapshot & snap);

  // ── Members ────────────────────────────────────────────────────────────────
  Parameters parameters_;

  std::unique_ptr<FRIClient>                     fri_client_;
  std::unique_ptr<KUKA::FRI::UdpConnection>      connection_;
  std::unique_ptr<KUKA::FRI::ClientApplication>  app_;

  std::thread          fri_thread_;
  std::atomic<bool>    fri_running_{false};
  rclcpp::Clock        throttle_clock_{RCL_STEADY_TIME};

  KUKA::FRI::ESessionState previous_session_state_{KUKA::FRI::IDLE};

  std::array<double, FRIClient::N_JOINTS> velocity_{};
  std::array<double, FRIClient::N_JOINTS> last_pos_{};
  double last_ts_sec_{0.0};
  double last_ts_nsec_{0.0};
  bool   velocity_initialized_{false};

  CommandGuard            command_guard_;
  CommandInterfaceHandles command_if_handles_;
  StateInterfaceHandles   state_if_handles_;
};

}  // namespace iiwa_controller_v2
