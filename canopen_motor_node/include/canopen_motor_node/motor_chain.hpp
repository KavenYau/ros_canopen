
#ifndef CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_
#define CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_

#include <memory>

// #include <ros/node_handle.h>
#include <canopen_chain_node/ros_chain.hpp>
#include <canopen_402/base.hpp>
// #include <canopen_402/motor.hpp>

#include <canopen_motor_node/robot_layer.hpp>
// #include <canopen_motor_node/controller_manager_layer.h>


namespace canopen {

class MotorChain : public RosChain {
    ClassAllocator<MotorBase> motor_allocator_;
    std::shared_ptr<LayerGroupNoDiag<MotorBase> > motors_;
    RobotLayerSharedPtr robot_layer_;

    // std::shared_ptr<ControllerManagerLayer> cm_;

    virtual bool nodeAdded(const NodeSharedPtr & node, const LoggerSharedPtr & logger);

public:
    // MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
    MotorChain(std::string node_name = "motor_chain_node");

    virtual bool setup_chain();

    // NOTE(sam): optional?
    bool setup_debug_interface();
};

}  // namespace canopen

#endif /* INCLUDE_CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_ */
