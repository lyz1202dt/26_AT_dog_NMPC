#pragma once

#include <controller_common/CtrlInterfaces.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <rclcpp/rclcpp.hpp>

namespace ocs2 {
class MPC_MRT_Interface;
}

namespace ocs2::legged_robot {

class SwitchedModelReferenceManager;

class ContactEstimater {
public:
    explicit ContactEstimater(CtrlInterfaces& ctrl_interfaces,
                              const PinocchioInterface& pinocchio_interface,
                              const PinocchioEndEffectorKinematics& ee_kinematics,
                              const CentroidalModelInfo& info,
                              const MPC_MRT_Interface* mrt_interface = nullptr);

    contact_flag_t state_update(const rclcpp::Time &time, const rclcpp::Duration &period, scalar_t current_time);

    [[nodiscard]] const contact_flag_t& getContactFlags() const { return contact_flag_; }
    [[nodiscard]] const feet_array_t<scalar_t>& getFilteredFootForces() const { return filtered_foot_forces_; }

private:
    static constexpr size_t kNumLegs = 4;
    static constexpr size_t kDofPerLeg = 3;

    scalar_t computeFootForceNorm(size_t leg_index);

    CtrlInterfaces* ctrl_interfaces_ = nullptr;   // 用于读取关节命令值和状态值
    const PinocchioInterface* pinocchio_interface_ = nullptr;  // 用于计算雅可比矩阵
    std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;  // 用于获取足端雅可比
    const CentroidalModelInfo* info_ = nullptr;  // 机器人模型信息
    const ocs2::MPC_MRT_Interface* mrt_interface_ = nullptr;  // 用于获取MPC状态和参考轨迹
    
    contact_flag_t contact_flag_{{false, false, false, false}};
    feet_array_t<scalar_t> filtered_foot_forces_{{0.0, 0.0, 0.0, 0.0}};  // 足端垂直力
    feet_array_t<size_t> contact_counter_{{0, 0, 0, 0}};
    feet_array_t<size_t> swing_counter_{{0, 0, 0, 0}};

    scalar_t contact_threshold_on_ = 50.0;   // 足端力阈值（N）
    scalar_t contact_threshold_off_ = 20.0;  // 足端力阈值（N）
    scalar_t force_filter_alpha_ = 0.2;
    size_t min_contact_samples_ = 2;
    size_t min_swing_samples_ = 2;
    scalar_t swing_phase_detection_ratio_ = 0.7;  // 在摆动相的70%后才启动检测（即后30%）
};

} // namespace ocs2::legged_robot
