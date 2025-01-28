#include "hcoil_pkg/psu_node.hpp"

PSU_Node::PSU_Node(): Node("psu_sub") {
    //initialise stuff here
    psu_addr_ = "string";

    vi_sub_ = this->create_subscription<hcoil_interfaces::msg::MagField>(
        "mag_topic", 10, std::bind(&PSU_Node::callbackVIWrite, this, _1)
    );
}

void PSU_Node::callbackVIWrite(const hcoil_interfaces::msg::MagField &msg){
    RCLCPP_INFO(this->get_logger(), "I received a message bx: %f, by: %f, bz: %f", msg.bx, msg.by, msg.bz);
    return;
}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PSU_Node>());
    rclcpp::shutdown();
    return 0;
}


