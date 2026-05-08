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

// on_init — читаем параметры из <hardware><param> в URDF
CallbackReturn IIWAHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;

  robot_ip_      = getParam(info, "robot_ip", "192.170.10.10");
  fri_port_      = std::stoi(getParam(info, "fri_port", "30200"));
  simulate_      = (getParam(info, "simulate", "false") == "true");
  cmd_mode_str_  = getParam(info, "command_mode", "position");

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

// Добавляем external_torque как "unlisted" интерфейс — не требует объявления в URDF.
// Стандартные интерфейсы (position/velocity/effort) базовый класс создаёт из URDF.
std::vector<hardware_interface::InterfaceDescription>
IIWAHardwareInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descs;
  descs.reserve(N_JOINTS);

  for (size_t i = 0; i < N_JOINTS; ++i) {
    hardware_interface::InterfaceInfo if_info;
    if_info.name           = "external_torque";
    if_info.data_type      = "double";
    if_info.initial_value  = "0.0";
    descs.emplace_back(info_.joints[i].name, if_info);
  }

  return descs;
}

CommandMode IIWAHardwareInterface::parseCommandMode(const std::string & mode_str) const
{
  return (mode_str == "torque") ? CommandMode::TORQUE : CommandMode::POSITION;
}

// on_activate — открываем FRI и ждём подключения
CallbackReturn IIWAHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "Активация...");

  // Кэшируем хэндлы интерфейсов (доступны после on_export_*,
  // базовый класс вызывает их до on_activate).
  for (size_t i = 0; i < N_JOINTS; ++i) {
    const std::string & jn = info_.joints[i].name;

    h_pos_[i] = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
    h_vel_[i] = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_VELOCITY);
    h_eff_[i] = get_state_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);
    h_ext_[i] = get_state_interface_handle(jn + "/external_torque");

    h_cmd_pos_[i] = get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_POSITION);
    h_cmd_eff_[i] = get_command_interface_handle(jn + "/" + hardware_interface::HW_IF_EFFORT);

    if (!h_pos_[i] || !h_vel_[i] || !h_eff_[i] || !h_cmd_pos_[i] || !h_cmd_eff_[i]) {
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

  // Создаём FRI-объекты
  fri_client_ = std::make_unique<FRIClient>(parseCommandMode(cmd_mode_str_));
  connection_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_        = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *fri_client_);

  // Открываем UDP-порт.
  // remoteHost=nullptr: принимаем пакеты от любого хоста.
  // Робот сам начинает слать пакеты после запуска ServerFriRos2 на контроллере.
  if (!app_->connect(fri_port_, nullptr)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("IIWAHardwareInterface"),
      "Не удалось открыть UDP-порт %d", fri_port_);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("IIWAHardwareInterface"),
    "UDP-порт %d открыт. Запустите ServerFriRos2 на роботе (%s)...",
    fri_port_, robot_ip_.c_str());

  // Запускаем FRI-поток
  fri_running_.store(true, std::memory_order_relaxed);
  fri_thread_ = std::thread(&IIWAHardwareInterface::friThreadFunc, this);

  // Ждём установки FRI-сессии (до 15 с)
  constexpr int kTimeoutMs = 15000;
  constexpr int kPollMs    = 100;
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
    // Не возвращаем ERROR: даём шанс дождаться в фоне
  } else {
    RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI-сессия установлена!");

    // Инициализируем prev_pos_ текущей измеренной позицией,
    // чтобы первый расчёт velocity не дал ложного скачка.
    const auto snap = fri_client_->getStateSnapshot();
    prev_pos_ = snap.measured_pos;
    vel_filtered_.fill(0.0);
  }

  return CallbackReturn::SUCCESS;
}

// Фоновый поток: step() ждёт UDP-пакет, вызывает callback, отправляет ответ.
// Блокирующий recv внутри step() — поток не ест CPU впустую.
void IIWAHardwareInterface::friThreadFunc()
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI-поток запущен");

  while (fri_running_.load(std::memory_order_relaxed)) {
    if (!app_->step()) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("IIWAHardwareInterface"),
        *rclcpp::Clock::make_shared(), 2000,
        "FRI: step() вернул false (соединение потеряно?)");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI-поток завершён");
}

// on_deactivate — останавливаем FRI-поток и закрываем UDP
CallbackReturn IIWAHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "Деактивация...");

  if (!simulate_) {
    fri_running_.store(false, std::memory_order_relaxed);
    if (fri_thread_.joinable()) {
      fri_thread_.join();
    }
    if (app_) {
      app_->disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"), "FRI отключён");
  }

  // Обнуляем хэндлы — они невалидны вне ACTIVE-состояния
  for (size_t i = 0; i < N_JOINTS; ++i) {
    h_pos_[i] = h_vel_[i] = h_eff_[i] = h_ext_[i] = nullptr;
    h_cmd_pos_[i] = h_cmd_eff_[i] = nullptr;
  }

  return CallbackReturn::SUCCESS;
}

// read() — копируем данные FRI → интерфейсы состояния.
// Вызывается ros2_control перед каждым шагом контроллера.
// set_state(..., false) = non-blocking try_lock, RT-безопасно.
hardware_interface::return_type IIWAHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (simulate_) {
    // Эхируем команды как состояние
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

  for (size_t i = 0; i < N_JOINTS; ++i) {
    const double pos = snap.measured_pos[i];

    // Численное дифференцирование + EMA-фильтр (alpha=0.2).
    // Сглаживает шум квантования энкодера и алиасинг при update_rate > FRI-rate.
    constexpr double kAlpha = 0.2;
    const double dt      = period.seconds();
    const double vel_raw = (dt > 1e-9) ? (pos - prev_pos_[i]) / dt : 0.0;
    vel_filtered_[i]     = kAlpha * vel_raw + (1.0 - kAlpha) * vel_filtered_[i];
    prev_pos_[i]         = pos;

    set_state(h_pos_[i], pos,              false);
    set_state(h_vel_[i], vel_filtered_[i], false);
    set_state(h_eff_[i], snap.measured_tau[i], false);
    set_state(h_ext_[i], snap.external_tau[i], false);
  }

  return hardware_interface::return_type::OK;
}

// write() — копируем команды контроллера → FRI.
// Вызывается ros2_control после шага контроллера.
// get_command(..., false) = non-blocking, RT-безопасно.
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

  // Передаём в FRIClient — применятся в следующем command()-цикле.
  // FRIClient сам удерживает последнюю безопасную позицию, если сессия неактивна.
  fri_client_->setTargetJointPositions(pos_cmd);
  fri_client_->setTargetJointTorques(tau_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace iiwa_controller
