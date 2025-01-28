#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class PSU_Node : public rclcpp::Node
{
    public:
        PSU_Node();
    
    private:
        std::string psu_addr;
        void callbackVIWrite();
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vi_sub_;
};