#include "hcoil_pkg/psu_node.hpp"

PSU_Node::PSU_Node(const std::string &nodeName, const bool &debugMode)
    : Node(nodeName) {
    nodeName_ = nodeName;
    
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Conversion values for Voltage and Current.";
    param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    this->declare_parameter("vConv", 0.01f, param_desc);
    this->declare_parameter("iConv", 0.01f, param_desc);

    vConv_ = this->get_parameter("vConv").as_double();
    iConv_ = this->get_parameter("iConv").as_double();

    RCLCPP_INFO(this->get_logger(), "vConv %d", vConv_);

    vi_sub_ = this->create_subscription<hcoil_interfaces::msg::VoltAmp>(
        nodeName_ + "/VI", 10, std::bind(&PSU_Node::callbackVIWrite, this, _1));

    if (debugMode) {
        RCLCPP_INFO(this->get_logger(), "We are in debug mode. test");
    } else {
        PSU = std::make_unique<DXKDP_PSU>(COM_PORT_, vConv_, iConv_);
    }
}

void PSU_Node::callbackVIWrite(const hcoil_interfaces::msg::VoltAmp& msg) {
    RCLCPP_INFO(this->get_logger(), "I received a message V: %f, I: %f",
                msg.voltage, msg.current);
    return;
}

// #ifndef TESTING_EXCLUDE_MAIN
// int main(int argc, char* argv[]){
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PSU_Node>("samefile_test", true));
//     rclcpp::shutdown();
//     return 0;
// }
// #endif

