#include "iiwa_controller/IIWAHardwareInterface.hpp"

#include <chrono>
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

static std::string getParam(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_val = "")
{
  auto it = info.hardware_parameters.find(name);
  return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

CallbackReturn IIWAHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;

  robot_ip_ = getParam(info, "robot_ip", "192.170.10.2");
  fri_port_ = std::stoi(getParam(info, "fri_port", "30200"));
  simulate_ = (getParam(info, "simulate", "false") == "true");
  cmd_mode_str_ = getParam(info, "command_mode", "position");

  RCLCPP_INFO(
    rclcpp::get_logger("IIWAHardwareInterface"),
    "on_init: ip=%s port=%d simulate=%s mode=%s",
    robot_ip_.c_str(), fri_port_,
    simulate_ ? "true" : "false",
    cmd_mode_str_.c_str());

  if (info.joints.size() != N_JOINTS) {
    RCLCPP_FATAL(
      rclcpp::get_logger("IIWAHardwareInterface"),
      "URDF содержит %zu суставов, ожидается %zu", info.joints.size(), N_JOINTS);
    return CallbackReturn::ERROR;
  }

  prev_pos_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

// external_torque не объявлен в URDF, поэтому добавляем его вручную как unlisted.
// Стандартные интерфейсы (position, velocity, effort) базовый класс берёт из URDF сам.
std::vector<hardware_interface::InterfaceDescription>
IIWAHardwareInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descs;
  descs.reserve(N_JOINTS);

  for (size_t i = 0; i < N_JOINTS; ++i) {
    hardware_interface::InterfaceInfo if_info;
    if_info.name = "external_torque";
    if_info.data_type = "double";
    if_info.initial_value = "0.0";
    descs.emplace_back(info_.joints[i].name, if_info);
  }

  return descs;
}

CommandMode IIWAHardwareInterface::parseCommandMode(const std::string & mode_str) const
{
  return (mode_str == "torque") ? CommandMode::TORQUE : CommandMode::POSITION;
}

CallbackReturn IIWAHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "Активация...");

  for (size_t i = 0; i < N_JOINTS; ++i) {
    const std::string & jn = info_.joints[i].name;

    h_pos_[i] = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
    h_vel_[i] = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_VELOCITY);
    h_eff_[i] = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);
    h_ext_[i] = get_state_interface_handle(jn + "/external_torque");

    h_cmd_pos_[i] = get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
    h_cmd_eff_[i] = get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);

    if (!h_pos_[i] || !h_vel_[i] || !h_eff_[i] || !h_ext_[i] || !h_cmd_pos_[i] || !h_cmd_eff_[i]) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IIWAHardwareInterface"),
        "Не удалось получить хэндл интерфейса для сустава '%s'. "
        "Проверьте объявление <state_interface>/<command_interface> в URDF.",
        jn.c_str());
      return CallbackReturn::ERROR;
    }
  }

  if (simulate_) {
    RCLCPP_WARN(rclcpp::get_logger("IIWAHardwareInterface"), "РЕЖИМ СИМУЛЯЦИИ: FRI не используется");
    return CallbackReturn::SUCCESS;
  }

  fri_client_ = std::make_unique<FRIClient>(parseCommandMode(cmd_mode_str_));
  // 100 мс таймаут: если закрытие сокета не разблокирует recvfrom() мгновенно,
  // поток всё равно выйдет через одну итерацию.
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>(100);
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *fri_client_);

  if (!app_->connect(fri_port_, robot_ip_.c_str())) {
    RCLCPP_FATAL(
      rclcpp::get_logger("IIWAHardwareInterface"),
      "Не удалось подключиться к %s:%d", robot_ip_.c_str(), fri_port_);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("IIWAHardwareInterface"),
    "UDP-порт %d открыт. Запустите ServerFriRos2 на роботе (%s)...",
    fri_port_, robot_ip_.c_str());

  fri_running_.store(true, std::memory_order_relaxed);
  fri_thread_ = std::thread(&IIWAHardwareInterface::friThreadFunc, this);

  // Ждём пока FRI-сессия установится, максимум 15 секунд.
  constexpr int kTimeoutMs = 15000;
  constexpr int kPollMs = 100;
  int elapsed = 0;
  while (fri_client_->getSessionState() == KUKA::FRI::IDLE && elapsed < kTimeoutMs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    elapsed += kPollMs;
  }

  if (fri_client_->getSessionState() == KUKA::FRI::IDLE) {
    RCLCPP_ERROR(
      rclcpp::get_logger("IIWAHardwareInterface"),
      "FRI не подключился за %d с. Проверьте ServerFriRos2 на %s",
      kTimeoutMs / 1000, robot_ip_.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI сессия установлена!");

    const auto snap = fri_client_->getStateSnapshot();
    prev_pos_ = snap.measured_pos;
    vel_filtered_.fill(0.0);
  }

  return CallbackReturn::SUCCESS;
}

// FRI-поток: крутит step() в ритме UDP-пакетов от Sunrise.
// После каждого успешного шага сигналит read() через condition_variable.
// Такая схема синхронизирует контрольный цикл с FRI-циклом:
//   read() всегда получает данные именно того пакета, что только что пришёл,
//   а не «какой-то из двух независимых потоков успел первый».
void IIWAHardwareInterface::friThreadFunc()
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI поток запущен");

  while (fri_running_.load(std::memory_order_relaxed)) {
    const bool ok = app_->step();

    if (ok) {
      {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        new_data_ = true;
      }
      sync_cv_.notify_one();
    } else {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("IIWAHardwareInterface"),
        throttle_clock_, 2000,
        "FRI: step() вернул false, возможно потеряли соединение");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI поток завершён");
}

CallbackReturn IIWAHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "Деактивация...");

  if (!simulate_) {
    fri_running_.store(false, std::memory_order_relaxed);
    // Сначала закрываем сокет — это разблокирует recvfrom() в FRI-потоке.
    // Только потом join(). Иначе join() зависнет навсегда.
    if (app_) {
      app_->disconnect();
    }
    // Разбудить read(), если он ждёт на cv — иначе RT-поток завис в wait_for()
    sync_cv_.notify_all();

    if (fri_thread_.joinable()) {
      fri_thread_.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI отключён");
  }

  for (size_t i = 0; i < N_JOINTS; ++i) {
    h_pos_[i] = h_vel_[i] = h_eff_[i] = h_ext_[i] = nullptr;
    h_cmd_pos_[i] = h_cmd_eff_[i] = nullptr;
  }

  return CallbackReturn::SUCCESS;
}

// read() ждёт сигнала от FRI-потока, а не читает «что успело» —
// это гарантирует, что каждый контрольный цикл обрабатывает ровно один FRI-пакет,
// устраняя рассинхрон двух независимых 200-Гц клоков.
hardware_interface::return_type IIWAHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
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

  // Ждём следующего FRI-пакета. Таймаут = 2× FRI-цикл на случай потери связи.
  // В норме wait_for() возвращается почти сразу — FRI-поток уже сигналил.
  {
    std::unique_lock<std::mutex> lock(sync_mutex_);
    sync_cv_.wait_for(lock, std::chrono::milliseconds(10),
      [this] { return new_data_ || !fri_running_.load(std::memory_order_relaxed); });
    new_data_ = false;
  }

  if (!fri_running_.load(std::memory_order_relaxed)) {
    return hardware_interface::return_type::OK;
  }

  const auto snap = fri_client_->getStateSnapshot();

  for (size_t i = 0; i < N_JOINTS; ++i) {
    const double pos = snap.measured_pos[i];

    constexpr double kAlpha = 0.2;
    const double dt = period.seconds();
    const double vel_raw = (dt > 1e-9) ? (pos - prev_pos_[i]) / dt : 0.0;
    vel_filtered_[i] = kAlpha * vel_raw + (1.0 - kAlpha) * vel_filtered_[i];
    prev_pos_[i] = pos;

    set_state(h_pos_[i], pos, false);
    set_state(h_vel_[i], vel_filtered_[i], false);
    set_state(h_eff_[i], snap.measured_tau[i], false);
    set_state(h_ext_[i], snap.external_tau[i], false);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IIWAHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (simulate_) {
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

}  // namespace iiwa_controller
