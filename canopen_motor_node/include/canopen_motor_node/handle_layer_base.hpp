
#ifndef CANOPEN_MOTOR_NODE_HANDLE_LAYER_BASE_H_
#define CANOPEN_MOTOR_NODE_HANDLE_LAYER_BASE_H_

#include <memory>
#include <string>
#include <canopen_master/layer.hpp>
#include <hardware_interface/robot_hardware.hpp>

namespace canopen {

class HandleLayerBase: public canopen::Layer{
public:
    HandleLayerBase(const std::string &name) : Layer(name) {}

    canopen::MotorBaseSharedPtr motor_;

    enum CanSwitchResult{
        NotSupported,
        NotReadyToSwitch,
        ReadyToSwitch,
        NoNeedToSwitch
    };

    virtual CanSwitchResult canSwitch(const canopen::MotorBase::OperationMode &m) = 0;
    virtual bool switchMode(const canopen::MotorBase::OperationMode &m) = 0;

    virtual bool forwardForMode(const canopen::MotorBase::OperationMode &m) = 0;

    virtual hardware_interface::JointStateHandle *getJointStateHandle() = 0;
    virtual hardware_interface::JointCommandHandle *getJointCommandHandle() = 0;
    // virtual void registerHandle(hardware_interface::JointStateInterface &iface) = 0;
    // virtual hardware_interface::JointHandle* registerHandle(hardware_interface::PositionJointInterface &iface,
    //                                                         const joint_limits_interface::JointLimits &limits,
    //                                                         const joint_limits_interface::SoftJointLimits *soft_limits = 0) = 0;
    // virtual hardware_interface::JointHandle* registerHandle(hardware_interface::VelocityJointInterface &iface,
    //                                                         const joint_limits_interface::JointLimits &limits,
    //                                                         const joint_limits_interface::SoftJointLimits *soft_limits = 0) = 0;
    // virtual hardware_interface::JointHandle* registerHandle(hardware_interface::EffortJointInterface &iface,
    //                                                         const joint_limits_interface::JointLimits &limits,
    //                                                         const joint_limits_interface::SoftJointLimits *soft_limits = 0) = 0;

    // virtual void enforceLimits(const ros::Duration &period, bool reset) = 0;
    virtual void enableLimits(bool enable) = 0;

    virtual void setEnableRosControlCommand(bool value) = 0;
};

typedef std::shared_ptr<HandleLayerBase> HandleLayerBaseSharedPtr;
typedef std::unordered_map<std::string, HandleLayerBaseSharedPtr> HandleMap;

}  // namespace canopen

#endif /* CANOPEN_MOTOR_NODE_HANDLE_LAYER_BASE_H_ */
