#include "hcoil_pkg/psu_node.hpp"

PSU_Node::PSU_Node(const std::string& nodeName, const bool& debugMode)
    : Node(nodeName) {
    nodeName_ = nodeName;

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Conversion values for Voltage and Current.";
    param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    this->declare_parameter("vConv", 0.01f, param_desc);
    this->declare_parameter("iConv", 0.01f, param_desc);

    vConv_ = this->get_parameter("vConv").as_double();
    iConv_ = this->get_parameter("iConv").as_double();

    RCLCPP_INFO(this->get_logger(), "vConv %f", vConv_);

    vi_sub_ = this->create_subscription<hcoil_interfaces::msg::VoltAmp>(
        nodeName_ + "/VI", 10, std::bind(&PSU_Node::callbackVIWrite, this, _1));

    pOn_service_ = this->create_service<std_srvs::srv::Trigger>(
        nodeName_ + "/PowerOn",
        std::bind(&PSU_Node::p_on_callback, this, _1, _2));
    pOff_service_ = this->create_service<std_srvs::srv::Trigger>(
        nodeName_ + "/PowerOff",
        std::bind(&PSU_Node::p_off_callback, this, _1, _2));

    if (debugMode) {
        RCLCPP_INFO(this->get_logger(), "We are in debug mode. test");
    } else {
        PSU = std::make_unique<DXKDP_PSU>(COM_PORT_, vConv_, iConv_);
    }
}

void PSU_Node::callbackVIWrite(const hcoil_interfaces::msg::VoltAmp& msg) {
    RCLCPP_INFO(this->get_logger(), "I received a message V: %f, I: %f",
                msg.voltage, msg.current);

    try {
        if (msg.voltage > vLimit)
            throw psuExceptions::OverVoltage("Over voltage limit");
        if (abs(msg.current) > iLimit)
            throw psuExceptions::OverCurrent("Over current limit");
    } catch (psuExceptions::OverVoltage) {
        std::string outputBuff =
            nodeName_ + ": requested V: " + std::to_string(msg.voltage) +
            "V exceeds limit of " + std::to_string(vLimit) +
            " Over voltage limit";
        throw(std::runtime_error(outputBuff));
    } catch (psuExceptions::OverCurrent) {
        std::string outputBuff =
            nodeName_ + ": requested I: " + std::to_string(msg.current) +
            "A exceeds limit of " + std::to_string(iLimit) +
            " Over current limit";
        throw(std::runtime_error(outputBuff));
    }

    if (msg.voltage != currentV_ || msg.current != currentI_) {
        currentV_ = msg.voltage;
        currentI_ = msg.current;
        switch (debugMode_) {
            case false:  // we are not in debug mode
                /**
                 * @brief General structure of the VI interface
                 * All supplies (except PSU3) have a separate Polarity and VI
                 * interface. The VI values given from the message get converted
                 * to unsigned int values. The polarity command is complete
                 * nonsense, it works differently on different PSUs. Technically
                 * the polarity command has 4 valid arguments: 0x00, 0x01, 0x02,
                 * 0x03.
                 *
                 * On the original power supplies (now PSU2, PSU4), 0x00 ->
                 * Positive output, 0x01 -> Negative output. And arguments 0x02
                 * and 0x03 actuall trigger an error.
                 *
                 * On PSU3, there is no polarity interface altogether, the sign
                 * of the output current is determined by the sign of the
                 * current value in the VI interface.
                 *
                 * On PSU0, PSU1, the behaviour is more complicated. In short,
                 * not only you need to find the correct state for the desired
                 * output, but you also need to ensure that transitioning to it
                 * is allowed. I could not find a logic to which state does what
                 * and what transition works, so I simply tested them all. See
                 * file ~ros_ws/src/ros_coils/transition_data.txt From there, I
                 * used the states that allow free transition between them.
                 */

                if (nodeName_ == "PSU3")
                    PSU->WriteVIGen2(
                        msg.voltage,
                        -msg.current);  // hilarious gen2 call with inverted
                                        // sign. Ask @VFrancescon if unsure
                                        // long story short, the PSUs are real
                                        // inconsistent
                else {                  // gen1 call with explicit polarity call
                    // ROS_INFO("Gen1 call");
                    PSU->WriteVI(msg.voltage, abs(msg.current));
                    if (nodeName_ == "PSU5") {
                        PSU->setPolarity(msg.current > 0 ? 0x00 : 0x01);
                        currentPolarity_ = msg.current > 0 ? 0x00 : 0x01;
                    } else if (nodeName_ == "PSU0") {
                        PSU->setPolarity(msg.current > 0 ? 0x01 : 0x02);
                        currentPolarity_ = msg.current > 0 ? 0x01 : 0x02;
                    } else if (nodeName_ == "PSU1") {
                        PSU->setPolarity(msg.current > 0 ? 0x00 : 0x01);
                        currentPolarity_ = msg.current > 0 ? 0x00 : 0x01;
                    } else {
                        PSU->setPolarity(msg.current > 0 ? 0x01 : 0x00);
                        currentPolarity_ = msg.current > 0 ? 0x01 : 0x00;
                    }
                }
                break;

            default:  // we are in debug mode
                RCLCPP_INFO(this->get_logger(),
                            "Debug mode, not doing a thing");
                if (nodeName_ == "PSU3")
                    RCLCPP_INFO(this->get_logger(), "Gen2 VI call");
                else
                    RCLCPP_INFO(this->get_logger(), "Gen1 VI call");
                break;
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "No need to act");
    }

    currentV_ = msg.voltage;
    currentI_ = msg.current;
    return;
}

void PSU_Node::p_on_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (POstate_ != true) {
        POstate_ = true;
        if (!debugMode_) {
            PSU->PoCtrl(0x01);
        } else {
            RCLCPP_INFO(this->get_logger(), "Turned supply on");
        }
    }
    res->success = true;
}

void PSU_Node::p_off_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (POstate_ != false) {
        POstate_ = false;
        if (!debugMode_) {
            PSU->PoCtrl(0x00);
        } else {
            RCLCPP_INFO(this->get_logger(), "Turned supply off");
        }
    }
    res->success = true;
}

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PSU_Node>("samefile_test", true));
    rclcpp::shutdown();
    return 0;
}
#endif
