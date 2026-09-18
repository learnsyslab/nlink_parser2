#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include "rclcpp/rclcpp.hpp"
#include <unordered_map>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "nlink_utils.h"
#include "nprotocol_extracter.h"
#include "nlink_utils/nlink_protocol.h"

class SerialPort;
class NProtocolExtracter;

namespace linktrack
{
extern std::map<NLinkProtocol *, std::shared_ptr<rclcpp::PublisherBase>> publishers_;

class Init
{
public:
    explicit Init(
        rclcpp::Node::SharedPtr node,
        NProtocolExtracter *protocol_extraction,
        SerialPort *serial);

private:
    void initDataTransmission();
    void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
    void initTagFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame1(NProtocolExtracter *protocol_extraction);
    void initNodeFrame2(NProtocolExtracter *protocol_extraction);
    void initNodeFrame3(NProtocolExtracter *protocol_extraction);
    void initNodeFrame4(NProtocolExtracter *protocol_extraction);
    void initNodeFrame5(NProtocolExtracter *protocol_extraction);
    void initNodeFrame6(NProtocolExtracter *protocol_extraction);
    void initNodeFrame7(NProtocolExtracter *protocol_extraction);

    std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr dt_bin_sub_;
};

}  /* namespace linktrack */

#endif  /* LINKTRACKINIT_H */