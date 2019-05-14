#include <canopen_motor_node/handle_layer.hpp>
#include <canopen_motor_node/robot_layer.hpp>
#include <canopen_motor_node/motor_chain.hpp>

using namespace canopen;

// TODO(sam): implement this
class RosSettings : public Settings {
public:
  RosSettings() {}
  // XmlRpcSettings(const XmlRpc::XmlRpcValue &v) : value_(v) {}
  // XmlRpcSettings& operator=(const XmlRpc::XmlRpcValue &v) { value_ = v;
  // return *this; }
private:
  virtual bool getRepr(const std::string &n, std::string &repr) const {
    // if(value_.hasMember(n)){
    //     std::stringstream sstr;
    //     sstr << const_cast< XmlRpc::XmlRpcValue &>(value_)[n]; // does not
    //     write since already existing
    //     repr = sstr.str();
    //     return true;
    // }
    return false;
  }
  // XmlRpc::XmlRpcValue value_;
};

// MotorChain::MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle
// &nh_priv) :
//         RosChain(nh, nh_priv), motor_allocator_("canopen_402",
//         "canopen::MotorBase::Allocator") {}

MotorChain::MotorChain(std::string node_name)
    : RosChain(node_name),
      motor_allocator_("canopen_402", "canopen::MotorBase::Allocator") {}

bool MotorChain::nodeAdded(const canopen::NodeSharedPtr &node,
                           const LoggerSharedPtr &logger) {
  RCLCPP_INFO(this->get_logger(), "adding node with name %s", node->node_name_.c_str());


  std::string name;
  if (!get_parameter_or(node->node_name_ + ".joint_name", name, std::string("default_joint"))) {
    RCLCPP_ERROR(this->get_logger(), "joint_name parameter not specified for %s, aborting!", node->node_name_.c_str());
    return false;
  }
  // std::string name = params["name"];
  std::string &joint = name;
  // if(params.hasMember("joint")) joint.assign(params["joint"]);
  //
  // if(!robot_layer_->getJoint(joint)){
  //     ROS_ERROR_STREAM("joint " + joint + " was not found in URDF");
  //     return false;
  // }
  //

  std::string alloc_name = "canopen_402/Motor402Allocator";
  // if(params.hasMember("motor_allocator"))
  // alloc_name.assign(params["motor_allocator"]);

  RosSettings settings;
  // if(params.hasMember("motor_layer")) settings = params["motor_layer"];

  MotorBaseSharedPtr motor;

  try {
    motor = motor_allocator_.allocateInstance(alloc_name, name + "_motor",
                                              node->getStorage(), settings);
  } catch (const std::exception &e) {
    std::string info = boost::diagnostic_information(e);
    RCLCPP_ERROR(this->get_logger(), info);
    return false;
  }

  if (!motor) {
    RCLCPP_ERROR(this->get_logger(), "could not allocate motor");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "adding motor");

  motor->registerDefaultModes(node->getStorage());
  motors_->add(motor);
  logger->add(motor);

  // HandleLayerSharedPtr handle = std::make_shared<HandleLayer>(joint, motor,
  // node->getStorage(), params);
  HandleLayerSharedPtr handle =
      std::make_shared<HandleLayer>(joint, node->node_name_, motor, node->getStorage());

  // canopen::LayerStatus s;
  // if(!handle->prepareFilters(s)){
  //     ROS_ERROR_STREAM(s.reason());
  //     return false;
  // }

  auto request = std::make_shared<canopen_msgs::srv::GetJointState::Request>();
  request->joint_name = name;

  if (get_joint_state_client_->wait_for_service(1s))
  {
    using ServiceResponseFuture =
        rclcpp::Client<canopen_msgs::srv::GetJointState>::SharedFuture;

    auto response_received_callback = [this, handle](ServiceResponseFuture future) {
        RCLCPP_INFO(this->get_logger(), "%s, setting starting position: [%f]", handle->name.c_str(), future.get()->position);
        handle->position_offset_ = future.get()->position;
      };

    auto future_result = get_joint_state_client_->async_send_request(
      request, response_received_callback);
  }

  auto handle_set_joint_to_zero =
    [this, handle](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
    {
      (void)request_header;
      RCLCPP_INFO(this->get_logger(), "%s: setting to zero", handle->name.c_str());
      handle->setJointToZero();
      response->success = true;
    };

  auto set_joint_to_zero_srv_ = create_service<std_srvs::srv::Trigger>(
    node->node_name_ + "/set_joint_to_zero", handle_set_joint_to_zero);

  set_joint_to_zero_srvs_.push_back(set_joint_to_zero_srv_);

  robot_layer_->add(joint, handle);
  logger->add(handle);

  setup_debug_interface(node, motor);

  return true;
}

bool MotorChain::setup_chain() {
  RCLCPP_INFO(this->get_logger(), "setting up motor chain");

  get_joint_state_client_ = create_client<canopen_msgs::srv::GetJointState>("/get_joint_state");

  motors_.reset(new LayerGroupNoDiag<MotorBase>("402 Layer"));
  robot_layer_.reset(new RobotLayer(this->get_logger()));

  if (RosChain::setup_chain()) {
    add(motors_);
    add(robot_layer_);
    RCLCPP_INFO(this->get_logger(), "chain setup successful");

    // if(!nh_.param("use_realtime_period", false)){
    //     dur.fromSec(boost::chrono::duration<double>(update_duration_).count());
    //     ROS_INFO_STREAM("Using fixed control period: " << dur);
    // }else{
    //     ROS_INFO("Using real-time control period");
    // }
    cm_.reset(new ControllerManagerLayer(robot_layer_));
    RCLCPP_INFO(this->get_logger(), "robot layer");
    add(cm_);


    return true;
  }

  RCLCPP_INFO(this->get_logger(), "chain setup failed");

  return false;
}

void MotorChain::handleWrite(LayerStatus &status,
                             const LayerState &current_state) {
  RosChain::handleWrite(status, current_state);
  for (const auto &handle : robot_layer_->handles_) {
    publish_all_debug(handle.second);
  }
}

bool MotorChain::setup_debug_interface(const canopen::NodeSharedPtr &node,
                                       MotorBaseSharedPtr motor) {

  std::string node_name = node->node_name_;

  // TODO(sam): replace with custom services?

  auto set_object_publishers_callback = [this, node_name, node](
      const canopen_msgs::msg::DebugPublishers::SharedPtr msg) -> void {

    RCLCPP_INFO(this->get_logger(), "clearing object publishers");
    publishers_.clear();
    bool force = true;

    for (const std::string &object_name : msg->data) {
      RCLCPP_INFO(this->get_logger(), "setting debug publishers: [%s]",
                  object_name.c_str());

      PublishFuncType pub = createPublishFunc(node_name + "/obj" + object_name,
                                              node, object_name, force);

      if (!pub) {
        RCLCPP_ERROR(this->get_logger(),
                     "%s could not create publisher for object: '%s'",
                     node_name.c_str(), object_name.c_str());
      }
      this->publishers_.push_back(pub);
    }
  };

  set_debug_publishers_sub_ =
      create_subscription<canopen_msgs::msg::DebugPublishers>(
          node_name + "/set_debug_publishers", set_object_publishers_callback);

  RCLCPP_INFO(this->get_logger(), "setting up debug interface for node: %s",
              node_name.c_str());

  state_publishers_[node_name] = create_publisher<std_msgs::msg::Int32>(node_name + "/state");

  operation_mode_publishers_[node_name] =
      create_publisher<std_msgs::msg::Int32>(node_name + "/operation_mode");

  auto switch_state_callback =
      [this, motor](const std_msgs::msg::Int32::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "switching to state: [%d]", msg->data);
    if (!motor->switchState((State402::InternalState)msg->data)) {
      RCLCPP_WARN(this->get_logger(), "switching state failed");
    }

    // publish_all_debug(motor);
  };

  auto switch_state_sub = create_subscription<std_msgs::msg::Int32>(
      node_name + "/switch_state", switch_state_callback);
  switch_state_subs_.push_back(switch_state_sub);

  auto switch_operation_mode_callback =
      [this, motor](const std_msgs::msg::Int32::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "switching to operation mode: [%d]",
                msg->data);
    MotorBase::OperationMode operation_mode =
        (MotorBase::OperationMode)msg->data;

    if (!motor->enterModeAndWait(operation_mode)) {
      RCLCPP_ERROR(this->get_logger(), "could not enter mode: %d",
                   (int)operation_mode);
    }

    // publish_all_debug(motor);
  };

  auto switch_operation_mode_sub = create_subscription<std_msgs::msg::Int32>(
      node_name + "/switch_operation_mode", switch_operation_mode_callback);
  switch_operation_mode_subs_.push_back(switch_operation_mode_sub);

  auto set_target_callback =
      [this, motor](const std_msgs::msg::Float32::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "setting target for [%s]: [%f]", motor->name.c_str(), msg->data);
    motor->setTarget(msg->data);
  };

  auto set_target_sub = create_subscription<std_msgs::msg::Float32>(
      node_name + "/set_target", set_target_callback);
  set_target_subs_.push_back(set_target_sub);

  auto enable_ros_control_command_callback =
      [this, motor](const std_msgs::msg::Bool::SharedPtr msg) -> void {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "enabling ros_control command");
    } else {
      RCLCPP_INFO(this->get_logger(), "disabling ros_control command");
    }
    for (RobotLayer::HandleMap::iterator it = robot_layer_->handles_.begin();
         it != robot_layer_->handles_.end(); ++it)
     {
        it->second->setEnableRosControlCommand(msg->data);
     }
  };

  enable_ros_control_command_sub_ = create_subscription<std_msgs::msg::Bool>(
      node_name + "/enable_ros_control_command", enable_ros_control_command_callback);
}

void MotorChain::publish_all_debug(HandleLayerBaseSharedPtr handle) {
  auto motor = handle->motor_;
  std::string node_name = handle->node_name_;

  std::shared_ptr<std_msgs::msg::Int32> msg;
  msg = std::make_shared<std_msgs::msg::Int32>();
  msg->data = (int)motor->state_handler_.getState();
  state_publishers_[node_name]->publish(msg);

  msg = std::make_shared<std_msgs::msg::Int32>();
  msg->data = motor->op_mode_display_atomic_;
  operation_mode_publishers_[node_name]->publish(msg);
}
