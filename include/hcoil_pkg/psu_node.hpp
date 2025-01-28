#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hcoil_interfaces/msg/mag_field.hpp"
#include "hcoil_interfaces/msg/volt_amp.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

using std::placeholders::_1;

class PSU_Node : public rclcpp::Node
{
    public:
        PSU_Node();
    
    private:
        std::string psu_addr_;
        void callbackVIWrite(const hcoil_interfaces::msg::VoltAmp &msg);
        rclcpp::Subscription<hcoil_interfaces::msg::VoltAmp>::SharedPtr vi_sub_;


        double currentI = 0.f, currentV = 0.f;   // internal values for cached I and V.
        double vConv = 0.01f, iConv = 0.01f;       // minimum psu increment for voltage and current.
        uint8_t currentPolarity;  // currently stored polarity
        std::string nodeName;     // node name. Equal to PSU name.
        int RatedV = 50;          // PSU rated voltage
        int RatedI = 30;          // PSU rated current

        double vLimit;
        double iLimit;
        bool debugMode = true;  // debug flag. If true, PSU is never instantiated
                                // and function calls are empty
};  


int main(int argc, char* argv[]);