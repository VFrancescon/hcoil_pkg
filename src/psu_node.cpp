#include "hcoil_pkg/psu_node.hpp"

PSU_Node::PSU_Node(): Node("psu_sub") {

    // setup parameter for root address. Think memory addreses,
    // this parameter gives you the entry point.
    auto addr_desc = rcl_interfaces::msg::ParameterDescriptor{};
    addr_desc.description = "Root Address for PSU.";
    addr_desc.type = 4; // String. see rcl_interfaces/msg/ParameterType for reference.
    this->declare_parameter("PSU_COM", "/dev/ttyUSB0", addr_desc);

    auto debug_desc = rcl_interfaces::msg::ParameterDescriptor{};
    debug_desc.description = "Toggle debug flag. If true, PSUs are never instantiated";
    debug_desc.type = 1; // Bool. see rcl_interfaces/msg/ParameterType for reference.
    this->declare_parameter("debugMode", false, debug_desc);



    vi_sub_ = this->create_subscription<hcoil_interfaces::msg::VoltAmp>(
        "mag_topic", 10, std::bind(&PSU_Node::callbackVIWrite, this, _1)
    );

    if(debugMode){
        RCLCPP_INFO(this->get_logger(), "We are in debug mode");   
    } else {
        PSU = std::make_unique<DXKDP_PSU>(COM_PORT, vConv, iConv);

    }

}

void PSU_Node::callbackVIWrite(const hcoil_interfaces::msg::VoltAmp &msg){
    RCLCPP_INFO(this->get_logger(), "I received a message V: %f, I: %f", msg.voltage, msg.current);
    return;
}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PSU_Node>());
    rclcpp::shutdown();
    return 0;
}


