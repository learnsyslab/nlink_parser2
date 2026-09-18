#include "linktrack_node/linktrackinit.h"

#include "nlink_parser2/msg/linktrack_anchorframe0.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe0.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe1.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe2.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe3.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe4.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe5.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe6.hpp"
#include "nlink_parser2/msg/linktrack_nodeframe7.hpp"
#include "nlink_parser2/msg/linktrack_tagframe0.hpp"
#include "serial/serial_port.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "nlink_utils/nutils.h"
#include "nlink_utils/linktrack_protocols.h"

#define ARRAY_ASSIGN(DEST, SRC)                                        \
    for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) \
    {                                                                  \
        DEST[_CNT] = SRC[_CNT];                                        \
    }

namespace linktrack
{
std::map<NLinkProtocol*, std::shared_ptr<rclcpp::PublisherBase>> publishers_;

nlink_parser2::msg::LinktrackAnchorframe0 g_msg_anchorframe0;
nlink_parser2::msg::LinktrackTagframe0 g_msg_tagframe0;
nlink_parser2::msg::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser2::msg::LinktrackNodeframe1 g_msg_nodeframe1;
nlink_parser2::msg::LinktrackNodeframe2 g_msg_nodeframe2;
nlink_parser2::msg::LinktrackNodeframe3 g_msg_nodeframe3;
nlink_parser2::msg::LinktrackNodeframe4 g_msg_nodeframe4;
nlink_parser2::msg::LinktrackNodeframe5 g_msg_nodeframe5;
nlink_parser2::msg::LinktrackNodeframe6 g_msg_nodeframe6;
nlink_parser2::msg::LinktrackNodeframe7 g_msg_nodeframe7;

static SerialPort* serial_ = nullptr;

Init::Init(
    rclcpp::Node::SharedPtr node,
    NProtocolExtracter* protocol_extraction,
    SerialPort* serial) : node_(node)
{
    serial_ = serial;
    initDataTransmission();
    initAnchorFrame0(protocol_extraction);
    initTagFrame0(protocol_extraction);
    initNodeFrame0(protocol_extraction);
    initNodeFrame1(protocol_extraction);
    initNodeFrame2(protocol_extraction);
    initNodeFrame3(protocol_extraction);
    initNodeFrame4(protocol_extraction);
    initNodeFrame5(protocol_extraction);
    initNodeFrame6(protocol_extraction);
    initNodeFrame7(protocol_extraction);
}

static void DTCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (serial_)
    {
        serial_->write(msg->data);
    }
}

/* Binary-safe variant of DTCallback.
 *
 * std_msgs::msg::String is a UTF-8 string, so rclpy publishers mangle or drop
 * payloads containing bytes above 0x7F; only ASCII survives that path. The
 * radio itself is fully binary transparent, so binary senders should publish
 * UInt8MultiArray here instead. See nlink_parser2/uwb_dt_codec.hpp. */
static void DTBinCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    if (serial_ && !msg->data.empty())
    {
        serial_->write(std::string(
            reinterpret_cast<const char *>(msg->data.data()), msg->data.size()));
    }
}

void Init::initDataTransmission()
{
    dt_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "nlink_linktrack_data_transmission", 1000, DTCallback);

    dt_bin_sub_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "nlink_linktrack_data_transmission_bin", 1000, DTBinCallback);
}

void Init::initAnchorFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolAnchorFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_anchorframe0";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackAnchorframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }
        auto data = nlt_anchorframe0_.result;
        linktrack::g_msg_anchorframe0.role = data.role;
        linktrack::g_msg_anchorframe0.id = data.id;
        linktrack::g_msg_anchorframe0.voltage = data.voltage;
        linktrack::g_msg_anchorframe0.local_time = data.local_time;
        linktrack::g_msg_anchorframe0.system_time = data.system_time;
        auto& msg_nodes = linktrack::g_msg_anchorframe0.nodes;
        msg_nodes.clear();
        decltype(linktrack::g_msg_anchorframe0.nodes)::value_type msg_node;
        for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i)
        {
            auto node = data.nodes[i];
            msg_node.role = node->role;
            msg_node.id = node->id;
            ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d);
            ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr);
            msg_nodes.push_back(msg_node);
        }
        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackAnchorframe0>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_anchorframe0);
        }
    });
}

void Init::initTagFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolTagFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_tagframe0";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackTagframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_tagframe0.result;
        auto& msg_data = g_msg_tagframe0;

        linktrack::g_msg_tagframe0.role = data.role;
        linktrack::g_msg_tagframe0.id = data.id;
        linktrack::g_msg_tagframe0.local_time = data.local_time;
        linktrack::g_msg_tagframe0.system_time = data.system_time;
        linktrack::g_msg_tagframe0.voltage = data.voltage;

        ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d);
        ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d);
        ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d);
        ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr);
        ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d);
        ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d);
        ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d);
        ARRAY_ASSIGN(msg_data.quaternion, data.quaternion);

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackTagframe0>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_tagframe0);
        }
    });
}

void Init::initNodeFrame0(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe0";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe0>(topic, 200);
            TopicAdvertisedTip(topic);
        }
        const auto& data = g_nlt_nodeframe0.result;
        auto& msg_data = linktrack::g_msg_nodeframe0;
        auto& msg_nodes = msg_data.nodes;

        msg_data.role = data.role;
        msg_data.id = data.id;
        msg_nodes.resize(data.valid_node_count);

        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.data.resize(node->data_length);
            memcpy(msg_node.data.data(), node->data, node->data_length);
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe0>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe0);
        }
    });
}

void Init::initNodeFrame1(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame1;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe1";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe1>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe1.result;
        auto& msg_data = linktrack::g_msg_nodeframe1;
        auto& msg_nodes = msg_data.nodes;

        msg_data.role = data.role;
        msg_data.id = data.id;
        msg_data.local_time = data.local_time;
        msg_data.system_time = data.system_time;
        msg_data.voltage = data.voltage;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d);
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe1>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe1);
        }
    });
}

void Init::initNodeFrame2(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame2;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe2";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe2>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe2.result;
        auto& msg_data = linktrack::g_msg_nodeframe2;
        auto& msg_nodes = msg_data.nodes;

        linktrack::g_msg_nodeframe2.role = data.role;
        linktrack::g_msg_nodeframe2.id = data.id;
        linktrack::g_msg_nodeframe2.local_time = data.local_time;
        linktrack::g_msg_nodeframe2.system_time = data.system_time;
        linktrack::g_msg_nodeframe2.voltage = data.voltage;
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.pos_3d, data.pos_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.eop_3d, data.eop_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.vel_3d, data.vel_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.imu_gyro_3d, data.imu_gyro_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.imu_acc_3d, data.imu_acc_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.angle_3d, data.angle_3d);
        ARRAY_ASSIGN(linktrack::g_msg_nodeframe2.quaternion, data.quaternion);

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.dis = node->dis;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe2>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe2);
        }
    });
}

void Init::initNodeFrame3(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame3;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe3";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe3>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe3.result;
        auto& msg_data = linktrack::g_msg_nodeframe3;
        auto& msg_nodes = msg_data.nodes;

        msg_data.role = data.role;
        msg_data.id = data.id;
        msg_data.local_time = data.local_time;
        msg_data.system_time = data.system_time;
        msg_data.voltage = data.voltage;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.dis = node->dis;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe3>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe3);
        }
    });
}

void Init::initNodeFrame4(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame4;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe4";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe4>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe4.result;

        linktrack::g_msg_nodeframe4.role = data.role;
        linktrack::g_msg_nodeframe4.id = data.id;
        linktrack::g_msg_nodeframe4.local_time = data.local_time;
        linktrack::g_msg_nodeframe4.system_time = data.system_time;
        linktrack::g_msg_nodeframe4.voltage = data.voltage;

        linktrack::g_msg_nodeframe4.tags.resize(data.tag_count);
        for (int i = 0; i < data.tag_count; ++i)
        {
            auto& msg_tag = linktrack::g_msg_nodeframe4.tags[i];
            auto tag = data.tags[i];
            msg_tag.id = tag->id;
            msg_tag.voltage = tag->voltage;
            msg_tag.anchors.resize(tag->anchor_count);
            for (int j = 0; j < tag->anchor_count; ++j)
            {
                auto& msg_anchor = msg_tag.anchors[j];
                auto anchor = tag->anchors[j];
                msg_anchor.id = anchor->id;
                msg_anchor.dis = anchor->dis;
            }
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe4>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe4);
        }
    });
}

void Init::initNodeFrame5(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame5;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe5";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe5>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe5.result;
        auto& msg_data = linktrack::g_msg_nodeframe5;
        auto& msg_nodes = msg_data.nodes;

        linktrack::g_msg_nodeframe5.role = data.role;
        linktrack::g_msg_nodeframe5.id = data.id;
        linktrack::g_msg_nodeframe5.local_time = data.local_time;
        linktrack::g_msg_nodeframe5.system_time = data.system_time;
        linktrack::g_msg_nodeframe5.voltage = data.voltage;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.dis = node->dis;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe5>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe5);
        }
    });
}

void Init::initNodeFrame6(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame6;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe6";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe6>(topic, 200);
            TopicAdvertisedTip(topic);
        }

        const auto& data = g_nlt_nodeframe6.result;
        auto& msg_data = linktrack::g_msg_nodeframe6;
        auto& msg_nodes = msg_data.nodes;

        linktrack::g_msg_nodeframe6.role = data.role;
        linktrack::g_msg_nodeframe6.id = data.id;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.data.resize(node->data_length);
            memcpy(msg_node.data.data(), node->data, node->data_length);
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe6>>(linktrack::publishers_[protocol]);
        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe6);
        }
    });
}

void Init::initNodeFrame7(NProtocolExtracter* protocol_extraction)
{
    auto protocol = new NLT_ProtocolNodeFrame7;
    protocol_extraction->AddProtocol(protocol);

    protocol->SetHandleDataCallback([=]
    {
        if (!linktrack::publishers_[protocol])
        {
            auto topic = "nlink_linktrack_nodeframe7";
            linktrack::publishers_[protocol] =
                node_->create_publisher<nlink_parser2::msg::LinktrackNodeframe7>(topic, 200);
            TopicAdvertisedTip(topic);
        }
        const auto& data = g_nlt_nodeframe7.result;
        auto& msg_data = linktrack::g_msg_nodeframe7;
        auto& msg_nodes = msg_data.nodes;

        msg_data.role = data.role;
        msg_data.id = data.id;
        msg_data.local_time = data.local_time;
        msg_data.system_time = data.system_time;
        msg_data.voltage = data.voltage;

        msg_nodes.resize(data.valid_node_count);
        for (size_t i = 0; i < data.valid_node_count; ++i)
        {
            auto& msg_node = msg_nodes[i];
            auto node = data.nodes[i];
            msg_node.id = node->id;
            msg_node.role = node->role;
            msg_node.dis = node->dis;
            msg_node.angle0 = node->angle0;
            msg_node.angle1 = node->angle1;
            msg_node.fp_rssi = node->fp_rssi;
            msg_node.rx_rssi = node->rx_rssi;
        }

        auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<nlink_parser2::msg::LinktrackNodeframe7>>(linktrack::publishers_[protocol]);

        if (publisher)
        {
            publisher->publish(linktrack::g_msg_nodeframe7);
        }
    });
}

}  /* namespace linktrack */