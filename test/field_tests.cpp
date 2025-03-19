#include <gtest/gtest.h>

#include "hcoil_interfaces/msg/volt_amp.hpp"
#include "hcoil_pkg/field_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class TestFieldNode : public ::testing::Test {
    public:
     std::string node_name = "field";
 
    protected:
     void SetUp() override {
         rclcpp::init(0, nullptr);
         node = std::make_shared<FieldNode>(node_name);
     }
 
     void TearDown() override {
         node.reset();
         rclcpp::shutdown();
     }
     std::shared_ptr<FieldNode> node;
};


TEST_F(TestFieldNode, TestInit){
    ASSERT_EQ(node->get_name(), node_name);
}

TEST_F(TestFieldNode, paramAssign){
    ASSERT_EQ(node->yRoot_, "PSU4");
}

TEST_F(TestFieldNode, BuildAddresses){
    auto it = std::find(node->allAddress_.begin(), node->allAddress_.end(), "VI/PSU4");
    ASSERT_NE(it, node->allAddress_.end());
}

TEST_F(TestFieldNode, SubscriberExists){
    auto msg = hcoil_interfaces::msg::MagField();
    msg.bx = 10;
    msg.by = 0;
    msg.bz = -1;
    // auto vi_pub = node->create_publisher<hcoil_interfaces::msg::VoltAmp>(
    //     node_name + "/VI", 10);
    // vi_pub->publish(vi_msg);
    auto bpub = node->create_publisher<hcoil_interfaces::msg::MagField>("magfield", 10);   
    bpub->publish(msg);

    bool check_bx = msg.bx == node->bx_;
    bool check_bz = msg.bz == node->bz_;
    ASSERT_EQ(check_bx && check_bz, true); 
}

// TEST_F(TestFieldNode, PublishersExist){
//     auto vi_sub = node->create_subscription<hcoil_interfaces::msg::VoltAmp>(
//         "VI/PSU0", 10, [](const hcoil_interfaces::msg::VoltAmp::SharedPtr msg) {
//             ASSERT_EQ(msg->voltage, 5.0);
//             ASSERT_EQ(msg->current, 2.0);
//         });

//     auto field_pub = node->create_publisher<hcoil_interfaces::msg::MagField>(
//         "magfield", 10);

//     auto field_msg = hcoil_interfaces::msg::MagField();
//     field_msg.bx = 10.0;
//     field_msg.by = 0.0;
//     field_msg.bz = -1.0;

//     rclcpp::WallRate loop_rate(10);
//     for (int i = 0; i < 10; ++i) {
//         field_pub->publish(field_msg);
//         rclcpp::spin_some(node);
//         loop_rate.sleep();
//     }
// }

// TEST_F(TestFieldNode, CalculateVI){}


int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}