#include "iiwa_controller/IIWAHardwareInterface.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace KUKA::FRI;

namespace iiwa_controller {
    
    template<typename T>
    constexpr const T& clamp(const T& v, const T& lo, const T& hi)
    {
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }

    CallbackReturn IIWAHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {

        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        simulate_ = false;
        hw_states_position_.resize(info_.joints.size(), 0.0);
        hw_states_velocity_.resize(info_.joints.size(), 0.0);
        hw_states_effort_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);
        prev_measured_pos_.resize(info_.joints.size(), 0.0);
        internal_command_position.resize(info_.joints.size(), 0.0);

        // пробегаемся по всем интерфейсам и смотрим какой режим управления установлен
        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            
            // проверка, на то что все суставы используют один и тот же тип управления
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                  rclcpp::get_logger("IiwaFRIHardwareInterface"),
                  "Joint '%s' has %li command interfaces found. 1 expected.", joint.name.c_str(),
                  joint.command_interfaces.size());
                return CallbackReturn::ERROR;
            }

            // что у каждого сустава ровно 3 интерфейса состояния:
            if (hw_command_mode_.empty()) {
                hw_command_mode_ = joint.command_interfaces[0].name;
          
                if (hw_command_mode_ != hardware_interface::HW_IF_POSITION &&
                  hw_command_mode_ != hardware_interface::HW_IF_VELOCITY &&
                  hw_command_mode_ != hardware_interface::HW_IF_EFFORT)
                {
                  RCLCPP_FATAL(
                    rclcpp::get_logger("IiwaFRIHardwareInterface"),
                    "Joint '%s' have %s unknown command interfaces.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str());
                  return CallbackReturn::ERROR;
                }
            }
            
            // 
            if (hw_command_mode_ != joint.command_interfaces[0].name) {
                RCLCPP_FATAL(
                  rclcpp::get_logger("IiwaFRIHardwareInterface"),
                  "Joint '%s' has %s command interfaces. Expected %s.", joint.name.c_str(),
                  joint.command_interfaces[0].name.c_str(), hw_command_mode_.c_str());
                return CallbackReturn::ERROR;
            }
          
            if (joint.state_interfaces.size() != 3) {
                RCLCPP_FATAL(
                  rclcpp::get_logger("IiwaFRIHardwareInterface"),
                  "Joint '%s' has %li state interface. 3 expected.", joint.name.c_str(),
                  joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }
          
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                  rclcpp::get_logger("IiwaFRIHardwareInterface"),
                  "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                  joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }
          
            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                  rclcpp::get_logger("IiwaFRIHardwareInterface"),
                  "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                  joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }
          
            if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(
                  rclcpp::get_logger("IiwaFRIHardwareInterface"),
                  "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                  joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return CallbackReturn::ERROR;
            }
        
        }

        return CallbackReturn::SUCCESS;

    }

    std::vector<hardware_interface::StateInterface> IIWAHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
        }

        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }

        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> IIWAHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (uint i = 0; i < info_.joints.size(); i++) {
            if (hw_command_mode_ == hardware_interface::HW_IF_POSITION) {
              command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                  info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
            
            } else if (hw_command_mode_ == hardware_interface::HW_IF_VELOCITY) {
              command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                  info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
            
            } else if (hw_command_mode_ == hardware_interface::HW_IF_EFFORT) {
              command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                  info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
            }

        }
        
        return command_interfaces;

    }

    CallbackReturn IIWAHardwareInterface::on_activate(const rclcpp_lifecycle::State& ) {
        RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "Starting ...please wait...");

        auto it = info_.hardware_parameters.find("simulate");
        if (it != info_.hardware_parameters.end()) {
            std::string sim_str = it->second;
            std::transform(sim_str.begin(), sim_str.end(), sim_str.begin(), ::tolower);
            simulate_ = (sim_str == "true");
        }

        if (!simulate_) {

            std::string ip = info_.hardware_parameters.at("robot_ip");
            int port = std::stoi(info_.hardware_parameters.at("robot_port"));
            
            fri_client_ = std::make_unique<FRIClient>();
            connection_ = std::make_unique<UdpConnection>();
            app_ = std::make_unique<ClientApplication>(*connection_, *fri_client_);
            app_->connect(port, ip.c_str());

            rclcpp::Time now = rclcpp::Clock().now();
            rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
            this->read(now, period);

            safety_override_active_ = true;
            hw_commands_ = hw_states_position_;
            RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "Connecting FRI to port= %i and ip= %s", port, ip.c_str());

        }

        RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "System Successfully started!");

        return CallbackReturn::SUCCESS;

    }

    CallbackReturn IIWAHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& ) {
        
        RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "Stopping ...please wait...");
        
        if (!simulate_) {
            app_->disconnect();
        }
        
        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "hw_commands_ reset to zero.");


        RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "System successfully stopped!");
        
        return CallbackReturn::SUCCESS;
    }


    hardware_interface::return_type IIWAHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration& period) {
        if (!simulate_) {

            if (!app_ || !app_->step())   // session порвалась?
            {
                RCLCPP_ERROR(rclcpp::get_logger("IIWAHardwareInterface"),
                            "FRI session lost");
                return hardware_interface::return_type::ERROR;
            }

            /* ---------- 2. Считываем измеренные данные ---------- */
            const auto pos_meas = fri_client_->getMeasuredJointPositions();
            const auto tau_meas = fri_client_->getMeasuredTorque();

            /* ---------- 3. Копируем в ros2_control ---------- */
            for (size_t i = 0; i < hw_states_position_.size(); ++i)
            {
                hw_states_position_[i] = pos_meas[i];

                // простая численная производная = (dq) / dt
                hw_states_velocity_[i] =
                (pos_meas[i] - prev_measured_pos_[i]) / period.seconds();

                hw_states_effort_[i] = tau_meas[i];
                prev_measured_pos_[i] = pos_meas[i];
            }

            return hardware_interface::return_type::OK;
        }

        for (size_t i = 0; i < hw_states_position_.size(); ++i) {
            hw_states_position_[i] = hw_commands_[i];
            hw_states_velocity_[i] = 0.0;
            hw_states_effort_[i] = 0.0;
        }
        return hardware_interface::return_type::OK;

    }

    hardware_interface::return_type IIWAHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
    {
        if (simulate_)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("IIWAHardwareInterface"),
                "Simulated write to robot (echo commands)");
            return hardware_interface::return_type::OK;
        }

        // ---------- 1. Подготовка массивов команд ----------
        std::array<double, 7> cmd_position{};
        std::array<double, 7> cmd_torque{};

        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            if (hw_command_mode_ == hardware_interface::HW_IF_POSITION)
                cmd_position[i] = hw_commands_[i];
            else if (hw_command_mode_ == hardware_interface::HW_IF_EFFORT)
                cmd_torque[i] = hw_commands_[i];
        }

        // ---------- 2. Проверка на "нулевые" команды ----------
        double sum = std::accumulate(
            hw_commands_.begin(), hw_commands_.end(), 0.0,
            [](double a, double b) { return a + std::abs(b); });

        if (sum > 1e-3 && safety_override_active_)
        {
            RCLCPP_WARN_ONCE(
                rclcpp::get_logger("IIWAHardwareInterface"),
                "Command ignored: hw_commands_ are effectively zero (likely startup or stale)");
            return hardware_interface::return_type::OK;
        }

        safety_override_active_ = false;

        // ---------- 3. Защита по лимитам углов ----------
        const double joint_limits[7][2] = {
            {-2.95, 2.95}, {-2.03, 2.03}, {-2.95, 2.95},
            {-2.03, 2.03}, {-2.95, 2.95}, {-2.03, 2.03}, {-3.0, 3.0}};

        if (hw_command_mode_ == hardware_interface::HW_IF_POSITION)
        {
            for (size_t i = 0; i < 7; ++i)
            {
                cmd_position[i] = clamp(cmd_position[i], joint_limits[i][0], joint_limits[i][1]);
            }
        }

        // ---------- 4. Отправка команды в FRI-клиент ----------
        if (hw_command_mode_ == hardware_interface::HW_IF_POSITION)
        {
            fri_client_->setTargetJointPositions(cmd_position);
        }
        else if (hw_command_mode_ == hardware_interface::HW_IF_EFFORT)
        {
            // TODO: реализовать setTargetTorque при необходимости
        }
        else if (hw_command_mode_ == hardware_interface::HW_IF_VELOCITY)
        {
            // Velocity mode не реализован в FRI
        }

        RCLCPP_DEBUG(
            rclcpp::get_logger("IIWAHardwareInterface"),
            "Command sent to FRI");
        return hardware_interface::return_type::OK;
    }

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(iiwa_controller::IIWAHardwareInterface, hardware_interface::SystemInterface)
