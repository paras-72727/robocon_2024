// Paras Sakhare

#ifndef FOUR_WHEEL_OMNI_HPP
#define FOUR_WHEEL_OMNI_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arduino_comms.hpp"
#include "wheel.hpp"

namespace FourWheelOmniArduino
{
    class FourWheelOmniSystemHardware : public hardware_interface::SystemInterface
    {
        private:
            ArduinoComms comms_;
            Wheel wheel_1;
            Wheel wheel_2;
            Wheel wheel_3;
            Wheel wheel_4;

            struct Config
            {
                std::string wheel_1 = "";
                std::string wheel_2 = "";
                std::string wheel_3 = "";
                std::string wheel_4 = "";
                float loop_rate = 0.0;
                std::string device = "";
                int baud_rate = 0;
                int timeout_ms = 0;
                int enc_counts_per_rev = 0; 
                int pid_p = 0;
                int pid_d = 0;
                int pid_i = 0;
                int pid_o = 0; 
            };
            
            Config cfg_;

        public:
            //override all necessat funcitons

            // on_init() function overide
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

            //export state interfaces and export command interfaces override 
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            // on_activate() function override
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            // read() function override
            hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

            // write() function override
            hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;          
    };
}

#endif