#include "iiwa_controller/IIWAHardwareInterface.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <limits>
#include <stdexcept>
#include "friException.h"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

PLUGINLIB_EXPORT_CLASS(
  iiwa_controller::IIWAHardwareInterface,
  hardware_interface::SystemInterface)

namespace iiwa_controller
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// The SDK keeps a 100 ms receive bound after the first packet. During startup
// only, allow 15 s for the operator to start Sunrise without retrying failed SDK steps.
class StartupConnection : public KUKA::FRI::UdpConnection
{
public:
  explicit StartupConnection(const std::atomic<bool> & running)
  : KUKA::FRI::UdpConnection(100), running_(running) {}

  int receive(char * buffer, int size) override
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    do {
      const int received = KUKA::FRI::UdpConnection::receive(buffer, size);
      if (received >= 0) {
        started_ = true;
        return received;
      }
      if (started_) {return received;}
    } while (running_.load() && std::chrono::steady_clock::now() < deadline);
    return -1;
  }
private:
  const std::atomic<bool> & running_;
  bool started_{false};
};

static const char * LOG = "IIWAHardwareInterface";

static std::string getParam(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_val = "")
{
  auto it = info.hardware_parameters.find(name);
  return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

// ── on_init ────────────────────────────────────────────────────────────────────

CallbackReturn IIWAHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;

  try {
    robot_ip_ = getParam(info, "robot_ip", "192.170.10.2");
    size_t used = 0;
    const auto port = getParam(info, "fri_port", "30200");
    fri_port_ = std::stoi(port, &used);
    if (used != port.size() || fri_port_ < 1 || fri_port_ > 65535) {
      throw std::invalid_argument("Invalid fri_port");
    }
    const auto cycle = getParam(info, "fri_cycle_ms", "5");
    fri_cycle_ms_ = std::stoi(cycle, &used);
    if (used != cycle.size() || fri_cycle_ms_ < 1 || fri_cycle_ms_ > 100) {
      throw std::invalid_argument("Invalid fri_cycle_ms (expected 1..100)");
    }
    const auto position_tau = getParam(info, "joint_position_tau", "0.04");
    joint_position_tau_ = std::stod(position_tau, &used);
    if (used != position_tau.size() || !std::isfinite(joint_position_tau_) ||
      joint_position_tau_ < 0.0)
    {
      throw std::invalid_argument("Invalid joint_position_tau (expected >= 0)");
    }
    const auto rt_prio = getParam(info, "rt_prio", "80");
    rt_prio_ = std::stoi(rt_prio, &used);
    if (used != rt_prio.size() || rt_prio_ < 1 || rt_prio_ > 99) {
      throw std::invalid_argument("Invalid rt_prio (expected 1..99)");
    }
    const auto simulation = getParam(info, "simulate", "false");
    if (simulation != "true" && simulation != "false") {
      throw std::invalid_argument("simulate must be true or false");
    }
    simulate_ = simulation == "true";
    in_addr address{};
    if (!simulate_ && inet_pton(AF_INET, robot_ip_.c_str(), &address) != 1) {
      throw std::invalid_argument("robot_ip must be an IPv4 address");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "Invalid hardware parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (info.joints.size() != N_JOINTS) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG),
      "URDF содержит %zu суставов, ожидается %zu", info.joints.size(), N_JOINTS);
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < N_JOINTS; ++i) {
    const auto & joint = info.joints[i];
    // SDK arrays are ordered A1..A7. Never silently permute axes.
    if (joint.name != "joint" + std::to_string(i + 1) ||
      joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "position")
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG), "Expected joint1..joint7 with position commands");
      return CallbackReturn::ERROR;
    }
    for (const auto * name : {"position", "velocity", "effort"}) {
      if (std::none_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [&](const auto & interface) {return interface.name == name;}))
      {
        return CallbackReturn::ERROR;
      }
    }
    const auto limits = info.limits.find(joint.name);
    if (limits == info.limits.end() || !limits->second.has_position_limits ||
      !limits->second.has_velocity_limits)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOG), "Missing position/velocity limits for %s", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    lower_[i] = limits->second.min_position;
    upper_[i] = limits->second.max_position;
    max_velocity_[i] = limits->second.max_velocity;
    if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i] ||
      !std::isfinite(max_velocity_[i]) || max_velocity_[i] <= 0.0)
    {
      return CallbackReturn::ERROR;
    }
  }

  prev_pos_.fill(0.0);
  velocity_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

// ── export_unlisted_state_interface_descriptions ────────────────────────────────

std::vector<hardware_interface::InterfaceDescription>
IIWAHardwareInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descs;
  descs.reserve(3 * N_JOINTS);

  for (size_t i = 0; i < N_JOINTS; ++i) {
    for (const auto * name : {"external_torque", "ipo_position", "fri_command_position"}) {
      hardware_interface::InterfaceInfo if_info;
      if_info.name = name;
      if_info.data_type = "double";
      if_info.initial_value = "0.0";
      descs.emplace_back(info_.joints[i].name, if_info);
    }
  }

  return descs;
}

// ── on_configure ───────────────────────────────────────────────────────────────
// Создаёт FRI-объекты. Сокет и поток запускаются при активации.

CallbackReturn IIWAHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (simulate_) {
    RCLCPP_WARN(rclcpp::get_logger(LOG), "РЕЖИМ СИМУЛЯЦИИ: FRI не используется");
    return CallbackReturn::SUCCESS;
  }

  releaseFRI();
  fri_client_ = std::make_unique<FRIClient>(joint_position_tau_);
  fri_client_->setLimits(lower_, upper_, max_velocity_);
  fri_client_->setExpectedSampleTime(fri_cycle_ms_ * 0.001);
  connection_ = std::make_unique<StartupConnection>(fri_running_);
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *fri_client_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn IIWAHardwareInterface::on_activate(const rclcpp_lifecycle::State & state)
{
  stopFRI();
  velocity_initialized_ = false;
  last_snapshot_ = {};
  try {
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & name = info_.joints[i].name;
      h_pos_[i] = get_state_interface_handle(name + "/position");
      h_vel_[i] = get_state_interface_handle(name + "/velocity");
      h_eff_[i] = get_state_interface_handle(name + "/effort");
      h_ext_[i] = get_state_interface_handle(name + "/external_torque");
      h_ipo_[i] = get_state_interface_handle(name + "/ipo_position");
      h_fri_cmd_[i] = get_state_interface_handle(name + "/fri_command_position");
      h_cmd_pos_[i] = get_command_interface_handle(name + "/position");
    }
    if (!simulate_) {
      // Recreate SDK sequence counters and buffers for every new session.
      if (on_configure(state) != CallbackReturn::SUCCESS ||
        !app_->connect(fri_port_, robot_ip_.c_str()))
      {
        releaseFRI();
        return CallbackReturn::ERROR;
      }
      fri_fault_.store(false);
      fri_running_.store(true);
      cycle_gate_.reset();
      fri_thread_ = std::thread(&IIWAHardwareInterface::friThreadFunc, this);
      const bool ready = cycle_gate_.waitReady(std::chrono::seconds(15));
      last_snapshot_ = fri_client_->getStateSnapshot();
      if (!ready || fri_fault_.load() || !last_snapshot_.valid ||
        last_snapshot_.quality < KUKA::FRI::GOOD ||
        last_snapshot_.session < KUKA::FRI::MONITORING_READY ||
        std::chrono::steady_clock::now() - last_snapshot_.received_at > std::chrono::milliseconds(100))
      {
        stopFRI();
        RCLCPP_ERROR(rclcpp::get_logger(LOG), "FRI activation failed: no fresh ready session");
        return CallbackReturn::ERROR;
      }
    }
    for (size_t i = 0; i < N_JOINTS; ++i) {
      double position = 0.0;
      if (simulate_) {
        get_state(h_pos_[i], position, true);
        if (!std::isfinite(position)) {position = 0.0;}
      } else {
        position = last_snapshot_.measured_pos[i];
      }
      set_state(h_pos_[i], position, true);
      set_state(h_vel_[i], 0.0, true);
      set_state(h_eff_[i], simulate_ ? 0.0 : last_snapshot_.measured_tau[i], true);
      set_state(h_ext_[i], simulate_ ? 0.0 : last_snapshot_.external_tau[i], true);
      set_state(h_ipo_[i], position, true);
      set_state(h_fri_cmd_[i], position, true);
      set_command(h_cmd_pos_[i], position, true);
    }
  } catch (const std::exception & e) {
    stopFRI();
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "Activation failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  active_ = true;
  return CallbackReturn::SUCCESS;
}

void IIWAHardwareInterface::stopFRI()
{
  active_ = false;
  fri_running_.store(false);
  cycle_gate_.stop();
  // Receive has a finite timeout. Never close a socket while step() uses it.
  if (fri_thread_.joinable()) {fri_thread_.join();}
  if (app_) {app_->disconnect();}
}

void IIWAHardwareInterface::releaseFRI()
{
  stopFRI();
  // ClientApplication::~ClientApplication calls the connection by reference.
  app_.reset();
  connection_.reset();
  fri_client_.reset();
}

IIWAHardwareInterface::~IIWAHardwareInterface() {releaseFRI();}

CallbackReturn IIWAHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  stopFRI();
  velocity_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn IIWAHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  releaseFRI();
  return CallbackReturn::SUCCESS;
}

CallbackReturn IIWAHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

CallbackReturn IIWAHardwareInterface::on_error(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void IIWAHardwareInterface::friThreadFunc()
{
  try {
    if (!realtime_tools::configure_sched_fifo(rt_prio_)) {
      RCLCPP_WARN(rclcpp::get_logger(LOG),
        "FRI worker could not enable FIFO priority %d; configure realtime permissions for this user",
        rt_prio_);
    } else {
      RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI worker uses FIFO priority %d", rt_prio_);
    }
    bool synchronized = false;
    const auto startup_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (fri_running_.load()) {
      if (!app_->step()) {
        throw std::runtime_error("FRI step failed");
      }
      if (fri_client_->getSessionState() == KUKA::FRI::IDLE) {
        throw std::runtime_error("FRI session ended");
      }
      const auto snapshot = fri_client_->getStateSnapshot();
      synchronized = synchronized || (snapshot.quality >= KUKA::FRI::GOOD &&
        snapshot.session >= KUKA::FRI::MONITORING_READY);
      if (synchronized) {
        if (!cycle_gate_.publishAndWait()) {
          if (!fri_running_.load()) {break;}
          throw std::runtime_error("FRI cycle was not completed by ros2_control within 100 ms");
        }
      } else if (std::chrono::steady_clock::now() >= startup_deadline) {
        throw std::runtime_error("FRI did not reach MONITORING_READY within 15 s");
      }
    }
  } catch (const KUKA::FRI::FRIException & e) {
    fri_fault_.store(true);
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "FRI SDK: %s", e.getErrorMessage());
  } catch (const std::exception & e) {
    fri_fault_.store(true);
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "FRI: %s", e.what());
  } catch (...) {
    fri_fault_.store(true);
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "Unknown FRI failure");
  }
  fri_running_.store(false);
  cycle_gate_.stop();
}

// ── compute_velocity_ ──────────────────────────────────────────────────────────
// Velocity is the finite difference of measured positions at robot timestamps.

void IIWAHardwareInterface::compute_velocity_(const IIWAStateSnapshot & snap)
{
  if (!velocity_initialized_) {
    prev_pos_     = snap.measured_pos;
    last_ts_sec_  = snap.time_stamp_sec;
    last_ts_nsec_ = snap.time_stamp_nano_sec;
    velocity_.fill(0.0);
    velocity_initialized_ = true;
    return;
  }

  if (snap.time_stamp_sec == last_ts_sec_ && snap.time_stamp_nano_sec == last_ts_nsec_) {
    return;  // нового FRI-пакета ещё нет
  }

  const double dt =
    static_cast<double>(
      static_cast<int64_t>(snap.time_stamp_sec) -
      static_cast<int64_t>(last_ts_sec_)) +
    (static_cast<double>(snap.time_stamp_nano_sec) -
     static_cast<double>(last_ts_nsec_)) * 1e-9;


  if (dt > 0.0) {
    for (size_t i = 0; i < N_JOINTS; ++i) {
      velocity_[i] = (snap.measured_pos[i] - prev_pos_[i]) / dt;
    }
  }

  prev_pos_     = snap.measured_pos;
  last_ts_sec_  = snap.time_stamp_sec;
  last_ts_nsec_ = snap.time_stamp_nano_sec;
}

// ── read ───────────────────────────────────────────────────────────────────────

hardware_interface::return_type IIWAHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  if (simulate_) {
    for (size_t i = 0; i < N_JOINTS; ++i) {
      double pos = 0.0;
      if (!get_command(h_cmd_pos_[i], pos, false) || !std::isfinite(pos)) {
        return hardware_interface::return_type::ERROR;
      }
      set_state(h_pos_[i], pos, false);
      set_state(h_vel_[i], 0.0, false);
      set_state(h_eff_[i], 0.0, false);
      set_state(h_ext_[i], 0.0, false);
      set_state(h_ipo_[i], pos, false);
      set_state(h_fri_cmd_[i], pos, false);
    }
    return hardware_interface::return_type::OK;
  }

  if (!fri_client_ || fri_fault_.load() || !fri_running_.load()) {
    return hardware_interface::return_type::ERROR;
  }
  if (!cycle_gate_.acquire()) {
    fri_fault_.store(true);
    fri_running_.store(false);
    cycle_gate_.stop();
    return hardware_interface::return_type::ERROR;
  }
  last_snapshot_ = fri_client_->getStateSnapshot();
  const auto & snap = last_snapshot_;
  if (!snap.valid || std::chrono::steady_clock::now() - snap.received_at >
    std::chrono::milliseconds(100))
  {
    fri_running_.store(false);
    return hardware_interface::return_type::ERROR;
  }
  compute_velocity_(snap);

  for (size_t i = 0; i < N_JOINTS; ++i) {
    set_state(h_pos_[i], snap.measured_pos[i], false);
    set_state(h_vel_[i], velocity_[i],         false);
    set_state(h_eff_[i], snap.measured_tau[i], false);
    set_state(h_ext_[i], snap.external_tau[i], false);
    set_state(h_ipo_[i], snap.ipo_valid ? snap.ipo_pos[i] : snap.measured_pos[i], false);
    set_state(h_fri_cmd_[i], snap.command_pos[i], false);
  }

  return hardware_interface::return_type::OK;
}

// ── write ──────────────────────────────────────────────────────────────────────

hardware_interface::return_type IIWAHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || simulate_) {return hardware_interface::return_type::OK;}
  if (!fri_client_ || fri_fault_.load() || !fri_running_.load()) {
    return hardware_interface::return_type::ERROR;
  }
  if (!fri_client_->isCommandingActive()) {
    return cycle_gate_.complete() ? hardware_interface::return_type::OK :
           hardware_interface::return_type::ERROR;
  }
  std::array<double, N_JOINTS> pos_cmd{};
  for (size_t i = 0; i < N_JOINTS; ++i) {
    if (!get_command(h_cmd_pos_[i], pos_cmd[i], false)) {
      cycle_gate_.stop();
      return hardware_interface::return_type::ERROR;
    }
    if (!std::isfinite(pos_cmd[i]) || pos_cmd[i] < lower_[i] || pos_cmd[i] > upper_[i]) {
      fri_fault_.store(true);
      fri_running_.store(false);
      return hardware_interface::return_type::ERROR;
    }
  }
  try {
    fri_client_->setTargetJointPositions(pos_cmd);
  } catch (const std::exception &) {
    cycle_gate_.stop();
    return hardware_interface::return_type::ERROR;
  }
  return cycle_gate_.complete() ? hardware_interface::return_type::OK :
         hardware_interface::return_type::ERROR;
}

}  // namespace iiwa_controller
