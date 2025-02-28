#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hcoil_interfaces/msg/mag_field.hpp"
#include "hcoil_interfaces/msg/volt_amp.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "DxkdpLib/DxkdpLib.hpp"

using std::placeholders::_1;

class PSU_Node : public rclcpp::Node
{

    private:
        rclcpp::Subscription<hcoil_interfaces::msg::VoltAmp>::SharedPtr vi_sub_;
        std::unique_ptr<DXKDP_PSU> PSU;
        void callbackVIWrite(const hcoil_interfaces::msg::VoltAmp &msg);

    public:
        PSU_Node(const std::string &nodeName, const bool &debugMode);   
        std::string psu_addr_;
        double currentI_ = 0.f, currentV_ = 0.f;   // internal values for cached I and V.
        double vConv_ = 0.01f, iConv_ = 0.01f;       // minimum psu increment for voltage and current.
        uint8_t currentPolarity_;  // currently stored polarity
        std::string nodeName_;     // node name. Equal to PSU name.
        std::string COM_PORT_;     // node name. Equal to PSU name.
        int RatedV_ = 50;          // PSU rated voltage
        int RatedI_ = 30;          // PSU rated current

        double vLimit;
        double iLimit;
        bool debugMode_ = true;  // debug flag. If true, PSU is never instantiated
                                // and function calls are empty
};  

// #ifndef TESTING_EXCLUDE_MAIN
// int main(int argc, char* argv[]);
// #endif