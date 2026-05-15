#include "iiwa_controller_v2/system_interface.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

PLUGINLIB_EXPORT_CLASS(
  iiwa_controller_v2::SystemInterface,
  hardware_interface::SystemInterface)

namespace iiwa_controller_v2
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static const char * LOG = "iiwa_controller_v2";

// ── Helpers ────────────────────────────────────────────────────────────────────

static std::string getParam(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_val = "")
{
  auto it = info.hardware_parameters.find(name);
  return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

// ── on_init ────────────────────────────────────────────────────────────────────

CallbackReturn SystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (!parse_parameters_()) {
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != FRIClient::N_JOINTS) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG),
      "Expected %zu joints, got %zu", FRIClient::N_JOINTS, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "on_init: ip=%s port=%d simulate=%s mode=%s tau=%.3f open_loop=%s",
    parameters_.robot_ip.c_str(), parameters_.fri_port,
    parameters_.simulate ? "true" : "false",
    parameters_.command_mode.c_str(),
    parameters_.joint_position_tau,
    parameters_.open_loop ? "true" : "false");

  command_guard_.configure(info_);

  velocity_.fill(0.0);
  last_pos_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

// ── export_unlisted_state_interface_descriptions ────────────────────────────────

std::vector<hardware_interface::InterfaceDescription>
SystemInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descs;

  // Per-joint extended interfaces
  const std::array<const char *, 3> per_joint_names = {
    HW_IF_EXTERNAL_TORQUE,
    HW_IF_COMMANDED_TORQUE,
    HW_IF_IPO_JOINT_POSITION,
  };
  descs.reserve(FRIClient::N_JOINTS * per_joint_names.size() + 5);

  for (const auto * if_name : per_joint_names) {
    for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
      hardware_interface::InterfaceInfo ifi;
      ifi.name          = if_name;
      ifi.data_type     = "double";
      ifi.initial_value = "0.0";
      descs.emplace_back(info_.joints[i].name, ifi);
    }
  }

  // Auxiliary robot-level interfaces (use a virtual sensor component name)
  const std::array<const char *, 5> aux_names = {
    HW_IF_SAMPLE_TIME,
    HW_IF_SESSION_STATE,
    HW_IF_CONNECTION_QUALITY,
    HW_IF_TIME_STAMP_SEC,
    HW_IF_TIME_STAMP_NANO_SEC,
  };
  for (const auto * if_name : aux_names) {
    hardware_interface::InterfaceInfo ifi;
    ifi.name          = if_name;
    ifi.data_type     = "double";
    ifi.initial_value = "0.0";
    descs.emplace_back(HW_IF_AUXILIARY_PREFIX, ifi);
  }

  return descs;
}

// ── prepare_command_mode_switch ─────────────────────────────────────────────────

hardware_interface::return_type SystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & /*start*/, const std::vector<std::string> & /*stop*/)
{
  // FRI does not support online command-mode switching.
  return hardware_interface::return_type::OK;
}

// ── on_configure ───────────────────────────────────────────────────────────────

CallbackReturn SystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (parameters_.simulate) {
    RCLCPP_WARN(rclcpp::get_logger(LOG), "SIMULATION MODE — FRI disabled");
    return CallbackReturn::SUCCESS;
  }

  const CommandMode cmd_mode =
    (parameters_.command_mode == "torque") ? CommandMode::TORQUE : CommandMode::POSITION;

  fri_client_ = std::make_unique<FRIClient>(cmd_mode, parameters_.joint_position_tau);
  // 100 ms receive timeout: ensures friThreadFunc can exit cleanly after disconnect().
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>(100);
  app_        = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *fri_client_);

  if (!app_->connect(parameters_.fri_port, parameters_.robot_ip.c_str())) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG),
      "Failed to open UDP socket on port %d (robot: %s)",
      parameters_.fri_port, parameters_.robot_ip.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "UDP socket open on port %d. Start ServerFriRos2 on robot (%s)...",
    parameters_.fri_port, parameters_.robot_ip.c_str());

  return CallbackReturn::SUCCESS;
}

// ── on_activate ────────────────────────────────────────────────────────────────

CallbackReturn SystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Activating...");

  // Populate interface handles
  command_if_handles_.populate(*this, info_);
  state_if_handles_.populate(*this, info_);
  command_if_handles_.nan_interfaces();
  state_if_handles_.nan_interfaces();

  if (parameters_.simulate) {
    return CallbackReturn::SUCCESS;
  }

  // Start FRI thread
  fri_running_.store(true, std::memory_order_relaxed);
  fri_thread_ = std::thread(&SystemInterface::friThreadFunc, this);

  // Wait up to 15 s for a FRI session to be established.
  constexpr int kTimeoutMs = 15000;
  constexpr int kPollMs    = 200;
  for (int elapsed = 0;
       fri_client_->getSessionState() == KUKA::FRI::IDLE && elapsed < kTimeoutMs;
       elapsed += kPollMs)
  {
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger(LOG), throttle_clock_, 2000,
      "Waiting for FRI session... (%d ms elapsed)", elapsed);
    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
  }

  if (fri_client_->getSessionState() == KUKA::FRI::IDLE) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG),
      "FRI session not established after %d s. Check ServerFriRos2 on %s",
      kTimeoutMs / 1000, parameters_.robot_ip.c_str());
    // Don't fail — user may still start the robot-side app.
  } else {
    RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI session established!");
    const auto snap = fri_client_->getStateSnapshot();
    last_pos_    = snap.measured_pos;
    last_ts_sec_ = static_cast<double>(snap.time_stamp_sec);
    last_ts_nsec_= static_cast<double>(snap.time_stamp_nano_sec);
    velocity_.fill(0.0);
    velocity_initialized_ = true;
  }

  previous_session_state_ = fri_client_->getSessionState();
  return CallbackReturn::SUCCESS;
}

// ── on_deactivate ──────────────────────────────────────────────────────────────

CallbackReturn SystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Deactivating...");

  if (!parameters_.simulate && fri_running_.load()) {
    fri_running_.store(false, std::memory_order_relaxed);
    // Disconnect FIRST so recvfrom() unblocks, then join the thread.
    if (app_) { app_->disconnect(); }
    if (fri_thread_.joinable()) { fri_thread_.join(); }
    RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI thread stopped");
  }

  // Release interface handles
  for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
    command_if_handles_.joint_position[i] = nullptr;
    command_if_handles_.torque[i]         = nullptr;
    state_if_handles_.position[i]         = nullptr;
    state_if_handles_.velocity[i]         = nullptr;
    state_if_handles_.effort[i]           = nullptr;
    state_if_handles_.external_torque[i]  = nullptr;
    state_if_handles_.commanded_torque[i] = nullptr;
    state_if_handles_.ipo_joint_position[i]= nullptr;
  }
  state_if_handles_.sample_time         = nullptr;
  state_if_handles_.session_state       = nullptr;
  state_if_handles_.connection_quality  = nullptr;
  state_if_handles_.time_stamp_sec      = nullptr;
  state_if_handles_.time_stamp_nano_sec = nullptr;

  velocity_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

// ── on_cleanup ─────────────────────────────────────────────────────────────────

CallbackReturn SystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  fri_client_.reset();
  connection_.reset();
  app_.reset();
  return CallbackReturn::SUCCESS;
}

// ── friThreadFunc ──────────────────────────────────────────────────────────────

void SystemInterface::friThreadFunc()
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI thread started");
  while (fri_running_.load(std::memory_order_relaxed)) {
    if (!app_->step()) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOG), throttle_clock_, 2000,
        "FRI step() returned false — connection may be lost");
    }
  }
  RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI thread stopped");
}

// ── read ───────────────────────────────────────────────────────────────────────

hardware_interface::return_type SystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (parameters_.simulate) {
    for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
      double pos = 0.0;
      get_command(command_if_handles_.joint_position[i], pos, false);
      std::ignore = state_if_handles_.position[i]->set_value(pos);
      std::ignore = state_if_handles_.velocity[i]->set_value(0.0);
      std::ignore = state_if_handles_.effort[i]->set_value(0.0);
      std::ignore = state_if_handles_.external_torque[i]->set_value(0.0);
      std::ignore = state_if_handles_.commanded_torque[i]->set_value(0.0);
      std::ignore = state_if_handles_.ipo_joint_position[i]->set_value(pos);
    }
    return hardware_interface::return_type::OK;
  }

  const auto snap = fri_client_->getStateSnapshot();

  // Detect unexpected exit from COMMANDING_ACTIVE
  const auto current_state = static_cast<KUKA::FRI::ESessionState>(snap.session_state);
  if (exit_commanding_active_(previous_session_state_, current_state)) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG),
      "Robot left COMMANDING_ACTIVE unexpectedly! Deactivate and re-activate the controller.");
    return hardware_interface::return_type::ERROR;
  }
  previous_session_state_ = current_state;

  // External torque safety check
  if (parameters_.external_torque_safety_check && !external_torque_safe_(snap)) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG),
      "External torque exceeded safety limit (%.1f Nm). Stopping.", parameters_.external_torque_limit);
    return hardware_interface::return_type::ERROR;
  }

  compute_velocity_(snap);
  state_if_handles_.push(snap, velocity_);

  return hardware_interface::return_type::OK;
}

// ── write ──────────────────────────────────────────────────────────────────────

hardware_interface::return_type SystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (parameters_.simulate) {
    return hardware_interface::return_type::OK;
  }

  if (fri_client_->getSessionState() != KUKA::FRI::COMMANDING_ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  std::array<double, FRIClient::N_JOINTS> pos_cmd{}, tau_cmd{};
  command_if_handles_.pull(pos_cmd, tau_cmd);

  if (!command_guard_.check_position(pos_cmd, LOG)) {
    return hardware_interface::return_type::ERROR;
  }
  if (!command_guard_.check_torque(tau_cmd, LOG)) {
    return hardware_interface::return_type::ERROR;
  }

  fri_client_->setTargetJointPositions(pos_cmd);
  fri_client_->setTargetJointTorques(tau_cmd);

  return hardware_interface::return_type::OK;
}

// ── Protected helpers ──────────────────────────────────────────────────────────

bool SystemInterface::parse_parameters_()
{
  const auto & info = info_;
  try {
    parameters_.robot_ip              = getParam(info, "robot_ip", "192.170.10.2");
    parameters_.fri_port              = std::stoi(getParam(info, "fri_port", "30200"));
    parameters_.simulate              = (getParam(info, "simulate", "false") == "true");
    parameters_.command_mode          = getParam(info, "command_mode", "position");
    parameters_.joint_position_tau    = std::stod(getParam(info, "joint_position_tau", "0.04"));
    parameters_.open_loop             = (getParam(info, "open_loop", "true") == "true");
    parameters_.rt_prio               = std::stoi(getParam(info, "rt_prio", "80"));
    parameters_.external_torque_safety_check =
      (getParam(info, "external_torque_safety_check", "true") == "true");
    parameters_.external_torque_limit =
      std::stod(getParam(info, "external_torque_limit", "2.0"));

    if (parameters_.fri_port < 30200 || parameters_.fri_port > 30209) {
      RCLCPP_ERROR(rclcpp::get_logger(LOG),
        "fri_port must be in [30200, 30209], got %d", parameters_.fri_port);
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "Failed to parse hardware parameters: %s", e.what());
    return false;
  }
  return true;
}

bool SystemInterface::exit_commanding_active_(
  KUKA::FRI::ESessionState previous, KUKA::FRI::ESessionState current)
{
  return previous == KUKA::FRI::COMMANDING_ACTIVE && current != KUKA::FRI::COMMANDING_ACTIVE;
}

bool SystemInterface::external_torque_safe_(const IIWAStateSnapshot & snap) const
{
  for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
    if (std::abs(snap.external_tau[i]) > parameters_.external_torque_limit) {
      return false;
    }
  }
  return true;
}

void SystemInterface::compute_velocity_(const IIWAStateSnapshot & snap)
{
  const double ts_sec  = static_cast<double>(snap.time_stamp_sec);
  const double ts_nsec = static_cast<double>(snap.time_stamp_nano_sec);

  if (!velocity_initialized_) {
    last_pos_    = snap.measured_pos;
    last_ts_sec_ = ts_sec;
    last_ts_nsec_= ts_nsec;
    velocity_.fill(0.0);
    velocity_initialized_ = true;
    return;
  }

  // No new FRI packet yet
  if (ts_sec == last_ts_sec_ && ts_nsec == last_ts_nsec_) {
    return;
  }

  const double dt = (ts_sec + ts_nsec * 1e-9) - (last_ts_sec_ + last_ts_nsec_ * 1e-9);
  if (dt > 0.0) {
    for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
      velocity_[i] = (snap.measured_pos[i] - last_pos_[i]) / dt;
    }
  }

  last_pos_    = snap.measured_pos;
  last_ts_sec_ = ts_sec;
  last_ts_nsec_= ts_nsec;
}

}  // namespace iiwa_controller_v2
