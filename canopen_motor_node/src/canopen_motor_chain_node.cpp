#include <canopen_motor_node/motor_chain.hpp>

using namespace canopen;


int main(int argc, char** argv){
  rclcpp::init(argc,  argv);

  auto node = std::make_shared<MotorChain>("motor_chain_node");

  if(!node->setup()){
      return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
