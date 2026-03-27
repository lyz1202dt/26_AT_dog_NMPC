#include "ocs2_quadruped_controller/estimator/ContactEstimater.h"

#include <algorithm>
#include <cmath>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <stdexcept>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

namespace ocs2::legged_robot {

ContactEstimater::ContactEstimater(
    CtrlInterfaces& ctrl_interfaces, const PinocchioInterface& pinocchio_interface, const PinocchioEndEffectorKinematics& ee_kinematics,
    const CentroidalModelInfo& info, const ocs2::MPC_MRT_Interface* mrt_interface)
    : ctrl_interfaces_(&ctrl_interfaces)
    , pinocchio_interface_(&pinocchio_interface)
    , ee_kinematics_(ee_kinematics.clone())
    , info_(&info)
    , mrt_interface_(mrt_interface) {
    contact_flag_         = contact_flag_t{false, false, false, false};
    filtered_foot_forces_ = feet_array_t<scalar_t>{0.0, 0.0, 0.0, 0.0};
    contact_counter_      = feet_array_t<size_t>{0, 0, 0, 0};
    swing_counter_        = feet_array_t<size_t>{0, 0, 0, 0};
    ee_kinematics_->setPinocchioInterface(*pinocchio_interface_);
}

scalar_t ContactEstimater::computeFootForceNorm(const size_t leg_index) {
    if (ctrl_interfaces_ == nullptr || pinocchio_interface_ == nullptr) {
        throw std::runtime_error("ContactEstimater is not initialized.");
    }

    // 获取关节力矩残差（测量力矩 - 命令力矩）
    // 力矩残差主要来自外部接触力，这正是我们要估计的
    const size_t begin = leg_index * kDofPerLeg;
    vector_t joint_torque_residuals(kDofPerLeg);
    
    for (size_t joint_offset = 0; joint_offset < kDofPerLeg; ++joint_offset) {
        const size_t joint_index = begin + joint_offset;
        const scalar_t measured_torque = 
            ctrl_interfaces_->joint_effort_state_interface_[joint_index].get().get_value();
        const scalar_t commanded_torque = 
            ctrl_interfaces_->joint_torque_command_interface_[joint_index].get().get_value();
        joint_torque_residuals(joint_offset) = measured_torque - commanded_torque;
    }

    // 获取当前关节位置和速度以更新机器人状态
    const size_t actuatedDofNum = info_->actuatedDofNum;
    vector_t q_pinocchio(info_->generalizedCoordinatesNum);
    vector_t v_pinocchio(info_->generalizedCoordinatesNum);

    // 从关节接口读取位置和速度
    q_pinocchio.setZero();
    v_pinocchio.setZero();

    // 设置基座方向（从IMU获取，这里暂时设为零，实际应从状态估计器获取）
    // 注意：在实际应用中，应该从 state estimator 或 CtrlComponent 传递这些信息
    q_pinocchio.segment<3>(3).setZero(); // Base orientation (should be updated with actual values)

    // 设置关节位置
    for (size_t i = 0; i < actuatedDofNum; ++i) {
        q_pinocchio(6 + i) = ctrl_interfaces_->joint_position_state_interface_[i].get().get_value();
        v_pinocchio(6 + i) = ctrl_interfaces_->joint_velocity_state_interface_[i].get().get_value();
    }

    // 更新运动学
    // 注意：需要使用 const_cast 来获取非 const 的 model 和 data，因为 pinocchio 的算法需要修改 data
    auto& model = const_cast<PinocchioInterface*>(pinocchio_interface_)->getModel();
    auto& data  = const_cast<PinocchioInterface*>(pinocchio_interface_)->getData();

    forwardKinematics(model, data, q_pinocchio, v_pinocchio);
    updateFramePlacements(model, data);

    // 获取足端雅可比矩阵
    // ee_kinematics_->getPositionLinearApproximation 返回所有足端的雅可比
    const auto jacobians = ee_kinematics_->getPositionLinearApproximation(q_pinocchio);

    // 获取当前腿的雅可比矩阵
    // jacobians[leg_index].dfdx 是 3 x generalizedCoordinatesNum 的矩阵
    // 我们需要提取与该腿关节对应的列（跳过前6个自由度：基座位置和姿态）
    const size_t joint_start_idx = 6 + leg_index * kDofPerLeg;
    matrix_t leg_jacobian = jacobians[leg_index].dfdx.block(0, joint_start_idx, 3, kDofPerLeg);

    // 使用雅可比转置将关节力矩残差映射到笛卡尔空间足端力
    // 关系式: tau_residual = J^T * F_contact
    // 求解足端力: F_contact = (J^T)^{-1} * tau_residual
    
    vector3_t foot_force;
    
    // 如果雅可比可逆，直接求解
    if (std::abs(leg_jacobian.determinant()) > 1e-6) {
        // F = (J^T)^{-1} * tau_residual
        foot_force = leg_jacobian.transpose().inverse() * joint_torque_residuals;
    } else {
        // 使用伪逆处理奇异情况
        foot_force = leg_jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse() * joint_torque_residuals;
    }

    // 返回垂直方向（Z轴）的力，因为这是接触检测最重要的分量
    return foot_force(2);
}

contact_flag_t ContactEstimater::state_update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/, scalar_t current_time) {
    if (ctrl_interfaces_ == nullptr || pinocchio_interface_ == nullptr) {
        throw std::runtime_error("ContactEstimater is not initialized.");
    }

    const size_t required_joint_num = kNumLegs * kDofPerLeg;
    if (ctrl_interfaces_->joint_effort_state_interface_.size() < required_joint_num
        || ctrl_interfaces_->joint_position_state_interface_.size() < required_joint_num
        || ctrl_interfaces_->joint_velocity_state_interface_.size() < required_joint_num) {
        return contact_flag_;
    }

    // 获取期望的接触状态（来自MPC的参考轨迹）
    contact_flag_t desired_contact_flags{true, true, true, true};
    if (mrt_interface_ != nullptr) {
        const auto& modeSchedule = mrt_interface_->getReferenceManager().getModeSchedule();
        const auto currentMode = modeSchedule.modeAtTime(current_time);
        desired_contact_flags = modeNumber2StanceLeg(currentMode);
    }

    for (size_t leg = 0; leg < kNumLegs; ++leg) {
        // 判断当前腿是否应该在摆动相
        const bool is_swing_phase = !desired_contact_flags[leg];
        
        if (!is_swing_phase) {
            // 支撑相：直接使用期望的接触状态
            contact_flag_[leg] = true;
            // 重置计数器
            contact_counter_[leg] = 0;
            swing_counter_[leg] = 0;
            continue;
        }
        
        // 摆动相：需要判断是否在后30%才启动检测
        bool enable_detection = false;
        
        if (mrt_interface_ != nullptr) {
            const auto& modeSchedule = mrt_interface_->getReferenceManager().getModeSchedule();
            
            // 查找当前所在的模式段
            const auto& eventTimes = modeSchedule.eventTimes;
            size_t currentModeIndex = 0;
            for (size_t i = 0; i < eventTimes.size(); ++i) {
                if (current_time >= eventTimes[i]) {
                    currentModeIndex = i + 1;
                }
            }
            
            // 获取当前段的起止时间
            scalar_t phase_start_time = (currentModeIndex > 0) ? eventTimes[currentModeIndex - 1] : 0.0;
            scalar_t phase_end_time = (currentModeIndex < eventTimes.size()) ? eventTimes[currentModeIndex] : current_time + 1.0;
            scalar_t phase_duration = phase_end_time - phase_start_time;
            scalar_t phase_progress = (current_time - phase_start_time) / phase_duration;
            
            // 只在摆动相的后30%（即phase_progress > 0.7）才启动检测
            enable_detection = (phase_progress >= swing_phase_detection_ratio_);
        }
        
        if (!enable_detection) {
            // 摆动相前70%：返回期望状态（不触地）
            contact_flag_[leg] = false;
            // 重置计数器
            contact_counter_[leg] = 0;
            swing_counter_[leg] = 0;
            continue;
        }
        
        // 摆动相后30%：启动足端触地检测
        const scalar_t foot_force = computeFootForceNorm(leg);

        // 低通滤波
        filtered_foot_forces_[leg] = force_filter_alpha_ * foot_force + (1.0 - force_filter_alpha_) * filtered_foot_forces_[leg];

        const bool contact_candidate = filtered_foot_forces_[leg] >= contact_threshold_on_;
        const bool swing_candidate   = filtered_foot_forces_[leg] <= contact_threshold_off_;

        if (contact_candidate) {
            contact_counter_[leg] += 1;
            swing_counter_[leg] = 0;
        } else if (swing_candidate) {
            swing_counter_[leg] += 1;
            contact_counter_[leg] = 0;
        } else {
            contact_counter_[leg] = 0;
            swing_counter_[leg]   = 0;
        }

        if (contact_counter_[leg] >= min_contact_samples_) {
            contact_flag_[leg] = true;
        } else if (swing_counter_[leg] >= min_swing_samples_) {
            contact_flag_[leg] = false;
        }
    }

    return contact_flag_;
}

} // namespace ocs2::legged_robot
