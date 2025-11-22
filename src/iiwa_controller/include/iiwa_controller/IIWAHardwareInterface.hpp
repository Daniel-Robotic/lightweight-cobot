#ifndef IIWA_HARDWARE_INTERFACE_HPP
#define IIWA_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "FRIClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace KUKA::FRI;

namespace iiwa_controller
{

    class IIWAHardwareInterface : public hardware_interface::SystemInterface {
        
        public:
            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        private:
            // TODO: append robotClient FRI
            std::unique_ptr<FRIClient> fri_client_;
            std::unique_ptr<ClientApplication> app_;
            std::unique_ptr<UdpConnection> connection_;

            bool simulate_;
            std::string hw_command_mode_;
            std::vector<double> hw_commands_;
            std::vector<double> hw_states_position_;
            std::vector<double> hw_states_velocity_;
            std::vector<double> hw_states_effort_;
            std::vector<double> internal_command_position;
            std::vector<double> prev_measured_pos_;
            bool safety_override_active_ = true;
    };


}



#endif