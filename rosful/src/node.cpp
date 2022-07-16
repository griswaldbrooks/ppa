#include <say-hello/speaker.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto const node = std::make_shared<rclcpp::Node>("incrementer");
  auto const say = sh::speaker{"tacos"};
	RCLCPP_INFO(node->get_logger(), say().c_str());
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
