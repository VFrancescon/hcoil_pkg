#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hcoil_interfaces/msg/mag_field.hpp"
#include "hcoil_interfaces/msg/volt_amp.hpp"
using std::placeholders::_1;

class PSU_Node : public rclcpp::Node
{
    public:
        PSU_Node();
    
    private:
        std::string psu_addr_;
        void callbackVIWrite(const hcoil_interfaces::msg::VoltAmp &msg);
        rclcpp::Subscription<hcoil_interfaces::msg::VoltAmp>::SharedPtr vi_sub_;
};


int main(int argc, char* argv[]);