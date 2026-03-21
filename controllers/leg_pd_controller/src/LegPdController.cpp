//
// Created by tlab-uav on 24-9-19.
//

#include "leg_pd_controller/LegPdController.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace leg_pd_controller {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::CallbackReturn LegPdController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            reference_interface_types_ =
                    auto_declare<std::vector<std::string> >("reference_interfaces", reference_interface_types_);
            state_interface_types_ =
                    auto_declare<std::vector<std::string> >("state_interfaces", state_interface_types_);
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration LegPdController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size());
        for (const auto &joint_name: joint_names_) {
            conf.names.push_back(joint_name + "/effort");
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration LegPdController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }
        return conf;
    }

    controller_interface::CallbackReturn LegPdController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
        reference_interface_types_ = get_node()->get_parameter("reference_interfaces").as_string_array();

        if (joint_names_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
            return CallbackReturn::ERROR;
        }

        if (state_interface_types_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'state_interfaces' is empty.");
            return CallbackReturn::ERROR;
        }

        if (reference_interface_types_.empty()) {
            reference_interface_types_ = {"position", "velocity", "effort", "kp", "kd"};
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Parameter 'reference_interfaces' is empty. Falling back to position/velocity/effort/kp/kd.");
        }

        const size_t joint_num = joint_names_.size();
        joint_effort_command_.assign(joint_num, 0.0);
        joint_position_command_.assign(joint_num, 0.0);
        joint_velocities_command_.assign(joint_num, 0.0);
        joint_kp_command_.assign(joint_num, 0.0);
        joint_kd_command_.assign(joint_num, 0.0);

        reference_interface_index_map_.clear();
        for (size_t i = 0; i < reference_interface_types_.size(); ++i) {
            reference_interface_index_map_[reference_interface_types_[i]] = i;
        }

        reference_interfaces_.assign(
            joint_num * reference_interface_types_.size(), std::numeric_limits<double>::quiet_NaN());
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_effort_command_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        for (auto &interface: command_interfaces_) {
            joint_effort_command_interface_.emplace_back(interface);
        }

        for (auto &interface: state_interfaces_) {
            const auto it = state_interface_map_.find(interface.get_interface_name());
            if (it != state_interface_map_.end()) {
                it->second->push_back(interface);
            }
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    bool LegPdController::on_set_chained_mode(bool /*chained_mode*/) {
        return true;
    }

    controller_interface::return_type LegPdController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        const auto ref_type_count = reference_interface_types_.size();
        if (joint_names_.size() != joint_effort_command_.size() ||
            joint_names_.size() != joint_kp_command_.size() ||
            joint_names_.size() != joint_position_command_.size() ||
            joint_names_.size() != joint_position_state_interface_.size() ||
            joint_names_.size() != joint_velocity_state_interface_.size() ||
            joint_names_.size() != joint_effort_command_interface_.size() ||
            reference_interfaces_.size() != joint_names_.size() * ref_type_count) {
            throw std::runtime_error("Mismatch in vector sizes in update_and_write_commands");
        }

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            const auto ref_base = i * ref_type_count;
            auto get_reference_value = [&](const std::string &interface_name, double default_value) {
                const auto it = reference_interface_index_map_.find(interface_name);
                if (it == reference_interface_index_map_.end()) {
                    return default_value;
                }

                const double value = reference_interfaces_[ref_base + it->second];
                return std::isnan(value) ? default_value : value;
            };

            joint_position_command_[i] = get_reference_value("position", 0.0);
            joint_velocities_command_[i] = get_reference_value("velocity", 0.0);
            joint_effort_command_[i] = get_reference_value("effort", 0.0);
            joint_kp_command_[i] = get_reference_value("kp", 0.0);
            joint_kd_command_[i] = get_reference_value("kd", 0.0);

            const double torque = joint_effort_command_[i] + joint_kp_command_[i] * (
                                      joint_position_command_[i] - joint_position_state_interface_[i].get().get_value())
                                  +
                                  joint_kd_command_[i] * (
                                      joint_velocities_command_[i] - joint_velocity_state_interface_[i].get().
                                      get_value());
            joint_effort_command_interface_[i].get().set_value(torque);
        }

        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> LegPdController::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.reserve(joint_names_.size() * reference_interface_types_.size());

        std::string controller_name = get_node()->get_name();
        size_t ind = 0;
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: reference_interface_types_) {
                reference_interfaces.emplace_back(
                    controller_name, joint_name + "/" + interface_type, &reference_interfaces_[ind++]);
            }
        }

        return reference_interfaces;
    }

#ifdef ROS2_CONTROL_VERSION_LT_3
    controller_interface::return_type LegPdController::update_reference_from_subscribers() {
        return controller_interface::return_type::OK;
    }
#else
    controller_interface::return_type LegPdController::update_reference_from_subscribers(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        return controller_interface::return_type::OK;
    }
#endif
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leg_pd_controller::LegPdController, controller_interface::ChainableControllerInterface);
