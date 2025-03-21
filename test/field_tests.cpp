#include <gtest/gtest.h>
#include "hcoil_pkg/field_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class TestFieldNode : public ::testing::Test {
   public:
    std::string node_name = "field";

   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        // node = std::make_shared<FieldNode>(node_name);
        rclcpp::NodeOptions options;
        options.parameter_overrides().emplace_back("yNum", 1);
        node = std::make_shared<FieldNode>(node_name, options);
    }

    void TearDown() override {
        node.reset();
        rclcpp::shutdown();
    }
    std::shared_ptr<FieldNode> node;
};

TEST_F(TestFieldNode, TestInit) { ASSERT_EQ(node->get_name(), node_name); }

TEST_F(TestFieldNode, paramAssign) {
    ASSERT_EQ(node->yRoot_, "PSU4");
    ASSERT_EQ(node->yNum_, 1);
}

TEST_F(TestFieldNode, BuildAddresses) {
    auto it = std::find(node->allAddress_.begin(), node->allAddress_.end(),
                        "VI/PSU4");
    ASSERT_NE(it, node->allAddress_.end());
}

TEST_F(TestFieldNode, SubscriberExists) {
    auto msg = magnetic_tentacle_interfaces::msg::MagneticField();
    msg.vector.x = 10;
    msg.vector.y = 0;
    msg.vector.z = -1;

    auto bpub =
        node->create_publisher<magnetic_tentacle_interfaces::msg::MagneticField>("magfield", 10);
    bpub->publish(msg);
    rclcpp::spin_some(node);

    bool check_bx = msg.vector.x == node->bx_;
    bool check_bz = msg.vector.z == node->bz_;
    ASSERT_EQ(check_bx && check_bz, true);
}

TEST_F(TestFieldNode, PublishersExist) {
    auto vi_sub = node->create_subscription<hcoil_interfaces::msg::VoltAmp>(
        "VI/PSU0", 10, [](const hcoil_interfaces::msg::VoltAmp::SharedPtr msg) {
            ASSERT_GT(msg->voltage, 22);
            ASSERT_GT(msg->current, 18);
        });

    auto field_pub =
        node->create_publisher<magnetic_tentacle_interfaces::msg::MagneticField>("magfield", 10);

    auto field_msg = magnetic_tentacle_interfaces::msg::MagneticField();
    field_msg.vector.x = 10.0;
    field_msg.vector.y = 0.0;
    field_msg.vector.z = -1.0;

    field_pub->publish(field_msg);
    rclcpp::spin_some(node);
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}