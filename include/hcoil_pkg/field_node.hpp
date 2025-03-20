#include <map>
#include <memory>
#include <exception>

#include "hcoil_interfaces/msg/mag_field.hpp"
#include "hcoil_interfaces/msg/volt_amp.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;

/**
 * @class FieldNode
 * @brief A ROS2 node for managing magnetic field control and communication.
 * 
 * The FieldNode class is responsible for subscribing to magnetic field messages,
 * publishing voltage and current messages, and managing calibration and control
 * parameters for a magnetic field system.
 */
class FieldNode : public rclcpp::Node {
    private:
     /**
      * @brief Subscriber for magnetic field messages.
      */
     rclcpp::Subscription<hcoil_interfaces::msg::MagField>::SharedPtr field_sub_;

     /**
      * @brief Publishers for voltage and current messages.
      */
     std::vector<rclcpp::Publisher<hcoil_interfaces::msg::VoltAmp>::SharedPtr> vi_pubs_;

    public:
     /**
      * @brief Constructor for the FieldNode class.
      * 
      * @param nodeName The name of the node.
      * @param options Node options for configuring the ROS2 node.
      */
     FieldNode(const std::string &nodeName, rclcpp::NodeOptions &options);

     /**
      * @brief Callback function for handling incoming magnetic field messages.
      * 
      * @param msg The received magnetic field message.
      */
     void callbackField(const hcoil_interfaces::msg::MagField &msg);

     /**
      * @brief Root addresses for x, y, and z axes.
      */
     std::string xRoot_, yRoot_, zRoot_;

     /**
      * @brief Number of devices along each axis.
      */
     int xNum_ = 2, yNum_ = 2, zNum_ = 2;

     /**
      * @brief Addresses for devices along each axis.
      */
     std::vector<std::string> xAddress_, yAddress_, zAddress_;

     /**
      * @brief Combined list of all device addresses.
      */
     std::vector<std::string> allAddress_;

     /**
      * @brief Calibration factors for magnetic field in mT/A.
      */
     float cal_x_ = 0.542; //!< Bx calibration factor.
     float cal_y_ = 1.07;  //!< By calibration factor.
     float cal_z_ = 0.633; //!< Bz calibration factor.

     /**
      * @brief Current values for x, y, and z axes in A.
      */
     float ix_ = 0, iy_ = 0, iz_ = 0;

     /**
      * @brief Magnetic field values for x, y, and z axes in mT.
      */
     float bx_ = 0, by_ = 0, bz_ = 0;

     /**
      * @brief Maximum allowed magnetic field in mT.
      */
     int maxField_ = 22;

     /**
      * @brief Maximum allowed change in current per cycle in A.
      */
     int maxChange_ = 15;

     /**
      * @brief Number of advanced settings or parameters.
      */
     int adv_num_;

     /**
      * @brief Messages containing voltage and current data.
      */
     std::vector<hcoil_interfaces::msg::VoltAmp> vi_msgs_;
//    private:
//     // subscribers
//     rclcpp::Subscription<hcoil_interfaces::msg::MagField>::SharedPtr field_sub_;
//     // publishers
//     std::vector<rclcpp::Publisher<hcoil_interfaces::msg::VoltAmp>::SharedPtr>
//         vi_pubs_;

//    public:
//     FieldNode(const std::string &nodeName, rclcpp::NodeOptions &options);
//     // FieldNode(const std::string &nodeName, rclcpp::NodeOptions &options);

//     void callbackField(const hcoil_interfaces::msg::MagField &msg);

//     std::string xRoot_, yRoot_, zRoot_;
//     int xNum_ = 2, yNum_ = 2, zNum_ = 2;
//     std::vector<std::string> xAddress_, yAddress_, zAddress_;
//     std::vector<std::string> allAddress_;

//     float cal_x_ = 0.542;  //!< Bx calibration factor. Units are mT/A
//     float cal_y_ = 1.07;   //!< By calibration factor. Units are mT/A
//     float cal_z_ = 0.633;  //!< Bz calibration factor. Units are mT/A
//     float ix_ = 0, iy_ = 0, iz_ = 0;
//     float bx_ = 0, by_ = 0, bz_ = 0;
//     int maxField_ = 22;
//     int maxChange_ =
//         15;  //!< Maximum change in current allowed per cycle. Units are A
//     int adv_num_;
//     std::vector<hcoil_interfaces::msg::VoltAmp> vi_msgs_;
};

namespace field_exceptions {
    class ChangeException : public std::runtime_error {
       public:
        ChangeException(const std::string &message) : std::runtime_error(message) {}
    };
    class maxFieldException : public std::runtime_error {
       public:
        maxFieldException(const std::string &message)
            : std::runtime_error(message) {}
    };
}

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char *argv[]);
#endif
