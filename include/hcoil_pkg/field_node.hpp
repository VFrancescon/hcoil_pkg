#include <map>
#include <memory>
#include <rclcpp/exceptions.hpp>

#include "hcoil_interfaces/msg/mag_field.hpp"
#include "hcoil_interfaces/msg/volt_amp.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;

class FieldNode : public rclcpp::Node {
   private:
    // subscribers
    rclcpp::Subscription<hcoil_interfaces::msg::MagField>::SharedPtr field_sub_;
    // publishers
    std::vector<rclcpp::Publisher<hcoil_interfaces::msg::VoltAmp>::SharedPtr>
        vi_pubs_;

   public:
    FieldNode(const std::string &nodeName);

    void callbackField(const hcoil_interfaces::msg::MagField &msg);

    std::string xRoot_, yRoot_, zRoot_;
    int xNum_ = 2, yNum_ = 2, zNum_ = 2;
    std::vector<std::string> xAddress_, yAddress_, zAddress_;
    std::vector<std::string> allAddress_;

    float cal_x_ = 0.542;  //!< Bx calibration factor. Units are mT/A
    float cal_y_ = 1.07;   //!< By calibration factor. Units are mT/A
    float cal_z_ = 0.633;  //!< Bz calibration factor. Units are mT/A
    float ix_ = 0, iy_ = 0, iz_ = 0;
    float bx_ = 0, by_ = 0, bz_ = 0;
    int maxField_ = 22;
    int maxChange_ =
        15;  //!< Maximum change in current allowed per cycle. Units are A
    int adv_num_;
    std::vector<hcoil_interfaces::msg::VoltAmp> vi_msgs_;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char *argv[]);
#endif
