#include "hcoil_pkg/field_node.hpp"

FieldNode::FieldNode(const std::string& nodeName, rclcpp::NodeOptions& options)
    : rclcpp::Node(nodeName, options) {
    // PSU number parameter declaration
    auto num_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    num_param_desc.description = "Number of supplies in given axis";
    num_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("xNum", 2, num_param_desc);
    this->declare_parameter("yNum", 2, num_param_desc);
    this->declare_parameter("zNum", 2, num_param_desc);
    xNum_ = this->get_parameter("xNum").as_int();
    yNum_ = this->get_parameter("yNum").as_int();
    zNum_ = this->get_parameter("zNum").as_int();

    // PSU Root Address declaration
    auto root_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    root_param_desc.description =
        "Root address for the first supply in the axis. Should be 1 or 2";
    root_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("xRoot", "PSU0", root_param_desc);
    this->declare_parameter("yRoot", "PSU4", root_param_desc);
    this->declare_parameter("zRoot", "PSU2", root_param_desc);
    xRoot_ = this->get_parameter("xRoot").as_string();
    yRoot_ = this->get_parameter("yRoot").as_string();
    zRoot_ = this->get_parameter("zRoot").as_string();

    for (int i = 0; i < xNum_; i++) {
        std::string modified_string = xRoot_;
        modified_string.back() +=
            i;  // Replace the last character with the value of i
        xAddress_.push_back("VI/" + modified_string);
    }
    for (int i = 0; i < yNum_; i++) {
        std::string modified_string = yRoot_;
        modified_string.back() +=
            i;  // Add the value of i to the last character
        yAddress_.push_back("VI/" + modified_string);
    }
    for (int i = 0; i < zNum_; i++) {
        std::string modified_string = zRoot_;
        modified_string.back() +=
            i;  // Add the value of i to the last character
        zAddress_.push_back("VI/" + modified_string);
    }
    allAddress_.insert(allAddress_.end(), xAddress_.begin(), xAddress_.end());
    allAddress_.insert(allAddress_.end(), yAddress_.begin(), yAddress_.end());
    allAddress_.insert(allAddress_.end(), zAddress_.begin(), zAddress_.end());

    field_sub_ = this->create_subscription<hcoil_interfaces::msg::MagField>(
        "magfield", 10, std::bind(&FieldNode::callbackField, this, _1));
    adv_num_ = allAddress_.size();
    vi_pubs_.resize(adv_num_);
    vi_msgs_.resize(adv_num_);  // Ensure vi_msgs_ is properly initialized
    for (int i = 0; i < adv_num_; i++) {
        vi_pubs_[i] = this->create_publisher<hcoil_interfaces::msg::VoltAmp>(
            allAddress_[i], 10);
    }
}

void FieldNode::callbackField(const hcoil_interfaces::msg::MagField& msg) {
    bx_ = msg.bx;
    by_ = msg.by;
    bz_ = msg.bz;
    ix_ = bx_ / cal_x_;
    iy_ = by_ / cal_y_;
    iz_ = bz_ / cal_z_;

    for (int i = 0; i < xNum_; i++) {
        vi_msgs_[i].current = ix_;
        vi_msgs_[i].voltage =
            std::min((abs(ix_) < 3 ? abs(ix_) * 1.8 : abs(ix_) * 1.2), 50.0);
        // vi_msgs_[i].voltage = abs(ix_);
        if (xNum_ != 2) {
            vi_msgs_[i].voltage *= 1.2;
        }
    }
    for (int i = 0; i < yNum_; i++) {
        vi_msgs_[i + xNum_].current = iy_;
        vi_msgs_[i + xNum_].voltage =
            std::min((abs(iy_) < 3 ? abs(iy_) * 1.8 : abs(iy_) * 1.2), 50.0);
        // vi_msgs_[i + xNum_].voltage = abs(iy_);
        if (yNum_ != 2) {
            vi_msgs_[i + xNum_].voltage *= 1.2;
        }
        // if(i == 0) vi_msgs_[i + xNum_].current *= -1;
    }
    for (int i = 0; i < zNum_; i++) {
        vi_msgs_[i + xNum_ + yNum_].current = iz_;
        vi_msgs_[i + xNum_ + yNum_].voltage =
            std::min((abs(iz_) < 3 ? abs(iz_) * 1.8 : abs(iz_) * 1.2), 50.0);
        // vi_msgs_[i + xNum_ + yNum].voltage = abs(iz_);
        if (zNum_ != 2) {
            vi_msgs_[i + xNum_ + yNum_].voltage *= 1.2;
        }
    }
    for (int i = 0; i < adv_num_; i++) {
        vi_pubs_[i]->publish(vi_msgs_[i]);
    }
}

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opt;
    rclcpp::spin(std::make_shared<FieldNode>("field_test", opt));
    rclcpp::shutdown();
    return 0;
}
#endif
