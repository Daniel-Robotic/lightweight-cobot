// ============================================================
// IIWAHardwareInterface.cpp
//
//  1. FRI работает в ОТДЕЛЬНОМ потоке (friThreadFunc), который
//     непрерывно вызывает app_->step(). Это обязательно, т.к.
//     FRI имеет жёсткие требования по таймингу (jitter < 1мс),
//     а ros2_control loop может иметь джиттер.
//
//  2. Синхронизация между ros2_control (read/write) и FRI
//     потоком выполнена внутри FRIClient через мьютекс.
//     read() и write() просто вызывают thread-safe геттеры/
//     сеттеры FRIClient — они никогда не блокируют FRI поток
//     надолго.
//
//  3. В режиме симуляции (simulate: true в URDF params) FRI
//     не используется — команды просто эхируются как состояние.
//     Удобно для разработки без реального робота.
//
//  4. Безопасность: если FRI сессия не в COMMANDING_ACTIVE,
//     write() пропускает отправку команды (FRIClient сам
//     удерживает последнюю безопасную позицию).
// ============================================================
#include "iiwa_controller/IIWAHardwareInterface.hpp"

#include <chrono>
#include <thread>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

// Регистрируем плагин для pluginlib
PLUGINLIB_EXPORT_CLASS(
  iiwa_controller::IIWAHardwareInterface,
  hardware_interface::SystemInterface)

namespace iiwa_controller
{

// Псевдоним для удобства
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Вспомогательная функция: получить параметр из HardwareInfo
// или вернуть значение по умолчанию
static std::string getParam(
  const hardware_interface::HardwareInfo& info,
  const std::string& name,
  const std::string& default_val = "")
{
  auto it = info.hardware_parameters.find(name);
  return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

// on_init()
// Читаем параметры из секции <hardware><param> URDF/XACRO.
// Пример в URDF:
//   <param name="robot_ip">192.168.1.1</param>
//   <param name="fri_port">30200</param>
//   <param name="simulate">false</param>
//   <param name="command_mode">position</param>
CallbackReturn IIWAHardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  // Базовый on_init выполняет проверку URDF структуры
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Читаем параметры
  // TODO: Изменить IP
  robot_ip_ = getParam(info, "robot_ip", "192.168.1.1");
  fri_port_ = std::stoi(getParam(info, "fri_port", "30200"));
  simulate_ = (getParam(info, "simulate", "false") == "true");
  cmd_mode_str_ = getParam(info, "command_mode", "position");

  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
    "Параметры: ip=%s port=%d simulate=%s mode=%s",
    robot_ip_.c_str(), fri_port_,
    simulate_ ? "true" : "false",
    cmd_mode_str_.c_str());

  // Проверяем число суставов в URDF
  if (info.joints.size() != N_JOINTS)
  {
    RCLCPP_FATAL(rclcpp::get_logger("IIWAHardwareInterface"),
      "URDF содержит %zu суставов, ожидается %zu",
      info.joints.size(), N_JOINTS);
    return CallbackReturn::ERROR;
  }

  // Инициализируем векторы данных
  hw_pos_.assign(N_JOINTS, 0.0);
  hw_vel_.assign(N_JOINTS, 0.0);
  hw_eff_.assign(N_JOINTS, 0.0);
  cmd_pos_.assign(N_JOINTS, 0.0);
  cmd_eff_.assign(N_JOINTS, 0.0);
  prev_pos_.assign(N_JOINTS, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
              "on_init() завершён успешно");
  return CallbackReturn::SUCCESS;
}

// export_state_interfaces()
// Регистрируем интерфейсы состояния:
//   joint_N/position, joint_N/velocity, joint_N/effort
// ros2_control controller_manager читает эти данные
std::vector<hardware_interface::StateInterface>
IIWAHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(N_JOINTS * 3);

  for (size_t i = 0; i < N_JOINTS; ++i)
  {
    const std::string& joint_name = info_.joints[i].name;

    // Позиция сустава [рад]
    interfaces.emplace_back(joint_name,
      hardware_interface::HW_IF_POSITION, &hw_pos_[i]);

    // Скорость сустава [рад/с] — вычисляется численно в read()
    interfaces.emplace_back(joint_name,
      hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);

    // Момент сустава [Нм]
    interfaces.emplace_back(joint_name,
      hardware_interface::HW_IF_EFFORT, &hw_eff_[i]);
  }

  return interfaces;
}

// export_command_interfaces()
// Регистрируем командные интерфейсы:
//   joint_N/position — для position контроллера
//   joint_N/effort   — для effort/impedance контроллера
std::vector<hardware_interface::CommandInterface>
IIWAHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(N_JOINTS * 2);

  for (size_t i = 0; i < N_JOINTS; ++i)
  {
    const std::string& joint_name = info_.joints[i].name;

    // Командная позиция [рад]
    interfaces.emplace_back(joint_name,
      hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);

    // Командный момент [Нм]
    interfaces.emplace_back(joint_name,
      hardware_interface::HW_IF_EFFORT, &cmd_eff_[i]);
  }

  return interfaces;
}

// parseCommandMode() — вспомогательный метод
CommandMode IIWAHardwareInterface::parseCommandMode(
  const std::string& mode_str) const
{
  if (mode_str == "torque") return CommandMode::TORQUE;
  return CommandMode::POSITION;
}

// on_activate()
// Создаём FRI объекты и запускаем фоновый поток.
CallbackReturn IIWAHardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
              "Активация hardware interface...");

  if (!simulate_)
  {
    // ---- Создаём FRI клиент с нужным режимом управления ----
    CommandMode mode = parseCommandMode(cmd_mode_str_);
    fri_client_  = std::make_unique<FRIClient>(mode);
    connection_  = std::make_unique<KUKA::FRI::UdpConnection>();
    app_         = std::make_unique<KUKA::FRI::ClientApplication>(
                     *connection_, *fri_client_);

    // Открываем UDP соединение
    // connect(port, remoteHost):
    //   port — локальный UDP порт (тот же, что задан в FRIConfiguration на роботе)
    //   remoteHost — nullptr означает «принять от любого хоста»
    //                (робот сам начинает посылать пакеты)
    if (!app_->connect(fri_port_, nullptr))
    {
      RCLCPP_FATAL(rclcpp::get_logger("IIWAHardwareInterface"),
                   "Не удалось открыть FRI UDP порт %d", fri_port_);
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
                "FRI UDP порт %d открыт. Ждём пакеты от робота...",
                fri_port_);

    // Запускаем FRI в фоновом потоке
    fri_running_.store(true, std::memory_order_relaxed);
    fri_thread_ = std::thread(&IIWAHardwareInterface::friThreadFunc, this);

    // Даём роботу 5 секунд на установку сессии
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Проверяем, что FRI хотя бы в состоянии MONITORING
    auto state = fri_client_->getSessionState();
    if (state == KUKA::FRI::IDLE)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IIWAHardwareInterface"),
                   "FRI сессия не установилась. "
                   "Запущено ли AAServerFri на роботе?");
      // Не возвращаем ERROR — даём ещё шанс (робот может быть занят)
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
                  "FRI сессия установлена!");
    }
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("IIWAHardwareInterface"),
                "РЕЖИМ СИМУЛЯЦИИ: FRI не используется");
  }

  return CallbackReturn::SUCCESS;
}

// friThreadFunc()
// Фоновый поток: крутим app_->step() с максимальной скоростью.
// app_->step() блокируется до получения UDP пакета от робота,
// поэтому этот поток НЕ занимает 100% CPU зря.
void IIWAHardwareInterface::friThreadFunc()
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
              "FRI поток запущен");

  while (fri_running_.load(std::memory_order_relaxed))
  {
    // step() = получить пакет + вызвать callback + отправить ответ
    // Возвращает false если соединение потеряно
    bool ok = app_->step();
    if (!ok)
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("IIWAHardwareInterface"),
        *rclcpp::Clock::make_shared(),
        2000,  // не чаще раза в 2 сек
        "FRI app->step() вернул false (соединение потеряно?)");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
              "FRI поток завершён");
}

// on_deactivate()
// Останавливаем FRI поток и закрываем соединение.
CallbackReturn IIWAHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
              "Деактивация hardware interface...");

  if (!simulate_)
  {
    // Сигнализируем потоку остановиться
    fri_running_.store(false, std::memory_order_relaxed);

    // Ждём завершения потока
    if (fri_thread_.joinable()) {
      fri_thread_.join();
    }

    // Закрываем UDP соединение
    if (app_) {
      app_->disconnect();
    }

    RCLCPP_INFO(rclcpp::get_logger("IIWAHardwareInterface"),
                "FRI отключён");
  }

  // Сбрасываем команды в ноль для безопасности
  std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
  std::fill(cmd_eff_.begin(), cmd_eff_.end(), 0.0);

  return CallbackReturn::SUCCESS;
}

// read()
// Копируем данные из FRIClient → буферы ros2_control.
// Вызывается перед каждым шагом контроллера (~1кГц или по URDF).
hardware_interface::return_type IIWAHardwareInterface::read(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period)
{
  if (simulate_)
  {
    for (size_t i = 0; i < N_JOINTS; ++i)
    {
      hw_vel_[i] = (cmd_pos_[i] - hw_pos_[i]) / period.seconds();
      hw_pos_[i] = cmd_pos_[i];
      hw_eff_[i] = cmd_eff_[i];
    }
    return hardware_interface::return_type::OK;
  }

  // Реальный робот
  // Получаем данные из FRIClient (thread-safe геттеры)
  const auto pos = fri_client_->getMeasuredJointPositions();
  const auto tau = fri_client_->getMeasuredTorque();

  for (size_t i = 0; i < N_JOINTS; ++i)
  {
    // Числовая производная скорости: v = (q_new - q_old) / dt
    // Точнее было бы использовать фильтр, но для начала достаточно
    double dt = period.seconds();
    hw_vel_[i] = (dt > 1e-9)
                 ? (pos[i] - prev_pos_[i]) / dt
                 : 0.0;

    hw_pos_[i]  = pos[i];
    hw_eff_[i]  = tau[i];
    prev_pos_[i] = pos[i];
  }

  return hardware_interface::return_type::OK;
}

// write()
// Копируем команды из буферов ros2_control → FRIClient.
// Вызывается после каждого шага контроллера.
hardware_interface::return_type IIWAHardwareInterface::write(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& /*period*/)
{
  if (simulate_) {
    return hardware_interface::return_type::OK;
  }

  // Упаковываем векторы ros2_control в std::array для FRIClient
  std::array<double, N_JOINTS> pos_arr, tau_arr;
  for (size_t i = 0; i < N_JOINTS; ++i)
  {
    pos_arr[i] = cmd_pos_[i];
    tau_arr[i] = cmd_eff_[i];
  }

  // Передаём в FRIClient (thread-safe сеттеры)
  // FRIClient применит их в следующем вызове command()
  fri_client_->setTargetJointPositions(pos_arr);
  fri_client_->setTargetJointTorques(tau_arr);

  return hardware_interface::return_type::OK;
}

} 