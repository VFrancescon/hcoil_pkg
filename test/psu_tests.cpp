#include <gtest/gtest.h>

#include "hcoil_interfaces/msg/volt_amp.hpp"
#include "hcoil_pkg/psu_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class TestPsuNode : public ::testing::Test {
   public:
    std::string node_name = "PSU3";

   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        // std::string node_name = "PSU3";
        node = std::make_shared<PSU_Node>(node_name, true);
    }

    void TearDown() override {
        node.reset();
        rclcpp::shutdown();
    }
    std::shared_ptr<PSU_Node> node;
};

TEST_F(TestPsuNode, TestCreation) {
    EXPECT_EQ(node->debugMode_, true);
    EXPECT_EQ(std::string(node->get_name()), node_name);
}

TEST_F(TestPsuNode, TestGen2Call) {
    auto vi_msg = hcoil_interfaces::msg::VoltAmp();
    vi_msg.voltage = 12.5;
    vi_msg.current = 1.5;

    auto vi_pub = node->create_publisher<hcoil_interfaces::msg::VoltAmp>(
        node_name + "/VI", 10);
    vi_pub->publish(vi_msg);

    rclcpp::spin_some(node);

    EXPECT_EQ(vi_msg.voltage, node->currentV_);
    EXPECT_EQ(vi_msg.current, node->currentI_);
}

TEST_F(TestPsuNode, PSU_ON) {
    auto turn_on_srv =
        node->create_client<std_srvs::srv::Trigger>(node_name + "/PowerOn");
    auto future = turn_on_srv->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    rclcpp::spin_some(node);

    EXPECT_EQ(node->POstate_, true);
}

TEST_F(TestPsuNode, PSU_OFF) {
    auto turn_off_srv =
        node->create_client<std_srvs::srv::Trigger>(node_name + "/PowerOff");
    auto future = turn_off_srv->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    rclcpp::spin_some(node);

    EXPECT_EQ(node->POstate_, false);
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}