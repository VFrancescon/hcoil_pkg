#include <map>
#include <memory>
#include <rclcpp/exceptions.hpp>

#include "DxkdpLib/DxkdpLib.hpp"
#include "hcoil_interfaces/msg/mag_field.hpp"
#include "hcoil_interfaces/msg/volt_amp.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;

class PSU_Node : public rclcpp::Node {
   private:
    // Subscribers
    rclcpp::Subscription<hcoil_interfaces::msg::VoltAmp>::SharedPtr vi_sub_;
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pOn_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pOff_service_;
    // Objects
    std::unique_ptr<DXKDP_PSU> PSU;

    // Subscriber callbacks
    void callbackVIWrite(const hcoil_interfaces::msg::VoltAmp &msg);

    // Service callbacks
    void p_on_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void p_off_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);

   public:
    PSU_Node(const std::string &nodeName, const bool &debugMode);
    std::string psu_addr_;
    double currentI_ = 0.f,
           currentV_ = 0.f;  // internal values for cached I and V.
    double vConv_ = 0.01f,
           iConv_ = 0.01f;     // minimum psu increment for voltage and current.
    uint8_t currentPolarity_;  // currently stored polarity
    std::string nodeName_;     // node name. Equal to PSU name.
    std::string COM_PORT_;     // node name. Equal to PSU name.
    int RatedV_ = 50;          // PSU rated voltage
    int RatedI_ = 30;          // PSU rated current

    double vLimit;
    double iLimit;
    bool debugMode_ = true;  // debug flag. If true, PSU is never instantiated
                             // and function calls are empty
    bool POstate_ = false;   // Power Output status for the PSU
};

namespace psuExceptions {
class OverCurrent : public std::runtime_error {
   public:
    explicit OverCurrent(const std::string &message)
        : std::runtime_error(message) {}
};

class OverVoltage : public std::runtime_error {
   public:
    OverVoltage(const std::string &message) : std::runtime_error(message) {}
};
}  // namespace psuExceptions

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char *argv[]);
#endif