#include "rclcpp/rclcpp.hpp"
#include "hcoil_pkg/psu_node.hpp"
#include <gtest/gtest.h>
#define TESTING_EXCLUDE_MAIN true

class TestPsuNode : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        std::string node_name = "test_psu";
        node = std::make_shared<PSU_Node>(node_name, true);
    }

    void TearDown() override
    {
        node.reset();
        rclcpp::shutdown();
    }
    std::shared_ptr<PSU_Node> node;
};

TEST_F(TestPsuNode, TestCreation){
    EXPECT_EQ(node->debugMode_, true);
    EXPECT_EQ(std::string(node->get_name()), "test_psu");
}

int main(int argc, char* argv[]){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}