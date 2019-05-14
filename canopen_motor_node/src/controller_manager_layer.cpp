
#include <canopen_motor_node/controller_manager_layer.hpp>
// #include <controller_manager/controller_manager.hpp>

using namespace canopen;

void ControllerManagerLayer::handleRead(canopen::LayerStatus &status,
                                        const LayerState &current_state) {
  if(current_state > Shutdown){
      if(!cm_) status.error("controller_manager is not intialized");
  }
}

void ControllerManagerLayer::handleWrite(canopen::LayerStatus &status,
                                         const LayerState &current_state) {
  if(current_state > Shutdown){
      if(!cm_){
          status.error("controller_manager is not intialized");
      }else{
          time_point abs_now = canopen::get_abs_time();

          cm_->update();

          // ros::Time now = ros::Time::now();
          //
          // ros::Duration period = fixed_period_;
          //
          // if(period.isZero()) {
          //     period.fromSec(boost::chrono::duration<double>(abs_now
          //     -last_time_).count());
          // }
          //
          // last_time_ = abs_now;
          //
          // bool recover = recover_.exchange(false);
          // cm_->update(now, period, recover);
          // robot_->enforce(period, recover);
      }
  }
}

void ControllerManagerLayer::handleInit(canopen::LayerStatus &status) {
  RCUTILS_LOG_INFO("init controller");
  if (cm_) {
    status.warn("controller_manager is already intialized");
  } else {
    recover_ = true;
    // last_time_ = canopen::get_abs_time();
    cm_.reset(new controller_manager::ControllerManager(robot_, executor_));

    cm_->load_controller(
    "ros_controllers",
    "ros_controllers::JointStateController",
    "joint_state_controller");

    cm_->load_controller(
    "ros_controllers",
    "ros_controllers::JointPositionController",
    "joint_position_controller");

    // cm_->load_controller(
    //     "ros_controllers",
    //     "ros_controllers::JointTrajectoryController",
    //     "my_robot_joint_trajectory_controller");

    // or we use the controller manager to configure every loaded controller
    if (cm_->configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCUTILS_LOG_ERROR("at least one controller failed to configure");
    }
    // and activate all controller
    if (cm_->activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      RCUTILS_LOG_ERROR("at least one controller failed to activate");
    }
    // TODO(sam): move executor and future handle to motor_chain?
    future_handle_ = std::async(std::launch::async, spin, executor_);

    robot_->doSwitch();
  }
}

void ControllerManagerLayer::handleRecover(canopen::LayerStatus &status) {
  if(!cm_) status.error("controller_manager is not intialized");
  else recover_ = true;
}

void ControllerManagerLayer::handleShutdown(canopen::LayerStatus &status) {
  cm_.reset();
}
