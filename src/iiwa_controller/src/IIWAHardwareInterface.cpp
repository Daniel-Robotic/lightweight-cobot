#include "iiwa_controller/IIWAHardwareInterface.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

PLUGINLIB_EXPORT_CLASS(
  iiwa_controller::IIWAHardwareInterface,
  hardware_interface::SystemInterface)

namespace iiwa_controller
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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

  robot_ip_            = getParam(info, "robot_ip", "192.170.10.2");
  fri_port_            = std::stoi(getParam(info, "fri_port", "30200"));
  simulate_            = (getParam(info, "simulate", "false") == "true");
  cmd_mode_str_        = getParam(info, "command_mode", "position");
  joint_position_tau_  = std::stod(getParam(info, "joint_position_tau", "0.04"));
  joint_velocity_tau_  = std::stod(getParam(info, "joint_velocity_tau", "0.01"));

  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "on_init: ip=%s port=%d simulate=%s mode=%s pos_tau=%.3f vel_tau=%.3f",
    robot_ip_.c_str(), fri_port_,
    simulate_ ? "true" : "false",
    cmd_mode_str_.c_str(),
    joint_position_tau_,
    joint_velocity_tau_);

  if (info.joints.size() != N_JOINTS) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG),
      "URDF содержит %zu суставов, ожидается %zu", info.joints.size(), N_JOINTS);
    return CallbackReturn::ERROR;
  }

  prev_pos_.fill(0.0);
  velocity_.fill(0.0);
  velocity_raw_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

// ── export_unlisted_state_interface_descriptions ────────────────────────────────

std::vector<hardware_interface::InterfaceDescription>
IIWAHardwareInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descs;
  descs.reserve(N_JOINTS);

  for (size_t i = 0; i < N_JOINTS; ++i) {
    hardware_interface::InterfaceInfo if_info;
    if_info.name          = "external_torque";
    if_info.data_type     = "double";
    if_info.initial_value = "0.0";
    descs.emplace_back(info_.joints[i].name, if_info);
  }

  return descs;
}

// ── on_configure ───────────────────────────────────────────────────────────────
// Открывает UDP-сокет и создаёт FRI-объекты. Не запускает поток.

CallbackReturn IIWAHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (simulate_) {
    RCLCPP_WARN(rclcpp::get_logger(LOG), "РЕЖИМ СИМУЛЯЦИИ: FRI не используется");
    return CallbackReturn::SUCCESS;
  }

  fri_client_ = std::make_unique<FRIClient>(parseCommandMode(cmd_mode_str_), joint_position_tau_);
  // 100 мс таймаут recvfrom — поток корректно завершится после disconnect().
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>(100);
  app_        = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *fri_client_);

  if (!app_->connect(fri_port_, robot_ip_.c_str())) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG),
      "Не удалось открыть UDP-сокет на порту %d (робот: %s)", fri_port_, robot_ip_.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "UDP-порт %d открыт. Запустите ServerFriRos2 на роботе (%s)...",
    fri_port_, robot_ip_.c_str());

  return CallbackReturn::SUCCESS;
}

// ── on_activate ────────────────────────────────────────────────────────────────
// Получает хэндлы интерфейсов, запускает FRI-поток и ждёт установки сессии.

CallbackReturn IIWAHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Активация...");

  for (size_t i = 0; i < N_JOINTS; ++i) {
    const std::string & jn = info_.joints[i].name;

    h_pos_[i]     = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
    h_vel_[i]     = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_VELOCITY);
    h_eff_[i]     = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);
    h_ext_[i]     = get_state_interface_handle(jn + "/external_torque");
    h_cmd_pos_[i] = get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
    h_cmd_eff_[i] = get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);

    if (!h_pos_[i] || !h_vel_[i] || !h_eff_[i] || !h_ext_[i] ||
        !h_cmd_pos_[i] || !h_cmd_eff_[i])
    {
      RCLCPP_FATAL(rclcpp::get_logger(LOG),
        "Не удалось получить хэндл интерфейса для сустава '%s'. "
        "Проверьте объявление <state_interface>/<command_interface> в URDF.",
        jn.c_str());
      return CallbackReturn::ERROR;
    }
  }

  if (simulate_) {
    return CallbackReturn::SUCCESS;
  }

  fri_running_.store(true, std::memory_order_relaxed);
  fri_thread_ = std::thread(&IIWAHardwareInterface::friThreadFunc, this);

  constexpr int kTimeoutMs = 15000;
  constexpr int kPollMs    = 200;
  for (int elapsed = 0;
       fri_client_->getSessionState() == KUKA::FRI::IDLE && elapsed < kTimeoutMs;
       elapsed += kPollMs)
  {
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger(LOG), throttle_clock_, 2000,
      "Ожидание FRI-сессии... (%d мс)", elapsed);
    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
  }

  if (fri_client_->getSessionState() == KUKA::FRI::IDLE) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG),
      "FRI не подключился за %d с. Проверьте ServerFriRos2 на %s",
      kTimeoutMs / 1000, robot_ip_.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI сессия установлена!");
    const auto snap = fri_client_->getStateSnapshot();
    prev_pos_     = snap.measured_pos;
    last_ts_sec_  = snap.time_stamp_sec;
    last_ts_nsec_ = snap.time_stamp_nano_sec;
    velocity_.fill(0.0);
    velocity_initialized_ = true;
  }

  previous_session_state_ = fri_client_->getSessionState();
  return CallbackReturn::SUCCESS;
}

// ── on_deactivate ──────────────────────────────────────────────────────────────
// Останавливает FRI-поток. FRI-объекты остаются — их очищает on_cleanup().

CallbackReturn IIWAHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Деактивация...");

  if (!simulate_ && fri_running_.load()) {
    fri_running_.store(false, std::memory_order_relaxed);
    // Сначала закрываем сокет — это разблокирует recvfrom() в FRI-потоке.
    // Только потом join(), иначе он зависнет навсегда.
    if (app_) { app_->disconnect(); }
    if (fri_thread_.joinable()) { fri_thread_.join(); }
    RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI поток остановлен");
  }

  for (size_t i = 0; i < N_JOINTS; ++i) {
    h_pos_[i] = h_vel_[i] = h_eff_[i] = h_ext_[i] = nullptr;
    h_cmd_pos_[i] = h_cmd_eff_[i] = nullptr;
  }

  velocity_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

// ── on_cleanup ─────────────────────────────────────────────────────────────────
// Освобождает FRI-объекты. Вызывается после on_deactivate().

CallbackReturn IIWAHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  fri_client_.reset();
  connection_.reset();
  app_.reset();
  return CallbackReturn::SUCCESS;
}

// ── friThreadFunc ──────────────────────────────────────────────────────────────

void IIWAHardwareInterface::friThreadFunc()
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI поток запущен");

  while (fri_running_.load(std::memory_order_relaxed)) {
    if (!app_->step()) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOG), throttle_clock_, 2000,
        "FRI: step() вернул false, возможно потеряли соединение");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG), "FRI поток завершён");
}

// ── compute_velocity_ ──────────────────────────────────────────────────────────
// Конечные разности + EMA-фильтр.
// int64-вычитание timestamp'ов предотвращает потерю точности при больших Unix-значениях.
// EMA-фильтр (joint_velocity_tau) убирает одиночные выбросы, которые видит JTC как
// скачки состояния и компенсирует агрессивными командами → хруст двигателей.

void IIWAHardwareInterface::compute_velocity_(const IIWAStateSnapshot & snap)
{
  if (!velocity_initialized_) {
    prev_pos_     = snap.measured_pos;
    last_ts_sec_  = snap.time_stamp_sec;
    last_ts_nsec_ = snap.time_stamp_nano_sec;
    velocity_.fill(0.0);
    velocity_raw_.fill(0.0);
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

  static constexpr std::array<double, N_JOINTS> kMaxVel =
    {1.71, 1.71, 1.75, 2.27, 2.44, 3.14, 3.14};
  static constexpr double kVelDeadband = 1e-4;

  if (dt > 0.0) {
    // EMA alpha для фильтра скорости: tau=0 → alpha=1 (без фильтра)
    const double vel_alpha = (joint_velocity_tau_ > 0.0)
      ? dt / (joint_velocity_tau_ + dt)
      : 1.0;

    for (size_t i = 0; i < N_JOINTS; ++i) {
      const double raw     = (snap.measured_pos[i] - prev_pos_[i]) / dt;
      const double clamped = std::clamp(raw, -kMaxVel[i], kMaxVel[i]);
      velocity_raw_[i] = (std::abs(clamped) < kVelDeadband) ? 0.0 : clamped;

      // EMA: velocity_[i] = alpha * raw + (1 - alpha) * prev_filtered
      velocity_[i] = vel_alpha * velocity_raw_[i] + (1.0 - vel_alpha) * velocity_[i];
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
  if (simulate_) {
    for (size_t i = 0; i < N_JOINTS; ++i) {
      double pos = 0.0;
      get_command(h_cmd_pos_[i], pos, false);
      set_state(h_pos_[i], pos, false);
      set_state(h_vel_[i], 0.0, false);
      set_state(h_eff_[i], 0.0, false);
      set_state(h_ext_[i], 0.0, false);
    }
    return hardware_interface::return_type::OK;
  }

  const auto snap = fri_client_->getStateSnapshot();

  // Обнаружение потери управления: неожиданный выход из COMMANDING_ACTIVE.
  const auto current_state = fri_client_->getSessionState();
  if (previous_session_state_ == KUKA::FRI::COMMANDING_ACTIVE &&
      current_state != KUKA::FRI::COMMANDING_ACTIVE)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOG),
      "Робот вышел из COMMANDING_ACTIVE! Деактивируйте и повторно активируйте контроллер.");
    return hardware_interface::return_type::ERROR;
  }
  previous_session_state_ = current_state;

  compute_velocity_(snap);

  for (size_t i = 0; i < N_JOINTS; ++i) {
    set_state(h_pos_[i], snap.measured_pos[i], false);
    set_state(h_vel_[i], velocity_[i],         false);
    set_state(h_eff_[i], snap.measured_tau[i], false);
    set_state(h_ext_[i], snap.external_tau[i], false);
  }

  return hardware_interface::return_type::OK;
}

// ── write ──────────────────────────────────────────────────────────────────────

hardware_interface::return_type IIWAHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (simulate_) {
    return hardware_interface::return_type::OK;
  }

  if (fri_client_->getSessionState() != KUKA::FRI::COMMANDING_ACTIVE) {
    return hardware_interface::return_type::OK;
  }

  std::array<double, N_JOINTS> pos_cmd{}, tau_cmd{};
  for (size_t i = 0; i < N_JOINTS; ++i) {
    get_command(h_cmd_pos_[i], pos_cmd[i], false);
    get_command(h_cmd_eff_[i], tau_cmd[i], false);
  }

  fri_client_->setTargetJointPositions(pos_cmd);
  fri_client_->setTargetJointTorques(tau_cmd);

  return hardware_interface::return_type::OK;
}

// ── parseCommandMode ───────────────────────────────────────────────────────────

CommandMode IIWAHardwareInterface::parseCommandMode(const std::string & mode_str) const
{
  return (mode_str == "torque") ? CommandMode::TORQUE : CommandMode::POSITION;
}

}  // namespace iiwa_controller
