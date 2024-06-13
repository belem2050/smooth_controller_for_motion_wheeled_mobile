#include "smooth_ctrl.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor; 
  auto rhc = std::make_shared<SmoothCtrl>("smooth_ctrl");
  executor.add_node(rhc->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  
  return 0;
}