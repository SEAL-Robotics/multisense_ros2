#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "multisense_ros/h264_packet.h"

using multisense_ros::h264_demand;
using multisense_ros::make_h264_packet;

TEST(H264Demand, NoSubscribersDoesNothing)
{
    const auto demand = h264_demand(0, 0);
    EXPECT_FALSE(demand.decode);
    EXPECT_FALSE(demand.packet);
}

TEST(H264Demand, PacketOnlySubscriberSkipsDecoder)
{
    const auto demand = h264_demand(0, 1);
    EXPECT_FALSE(demand.decode);
    EXPECT_TRUE(demand.packet);
}

TEST(H264Demand, DecodedOnlySubscriberPublishesNoPacket)
{
    const auto demand = h264_demand(2, 0);
    EXPECT_TRUE(demand.decode);
    EXPECT_FALSE(demand.packet);
}

TEST(H264Demand, BothSubscribedDoesBoth)
{
    const auto demand = h264_demand(1, 3);
    EXPECT_TRUE(demand.decode);
    EXPECT_TRUE(demand.packet);
}

TEST(H264Packet, CarriesAccessUnitBytesAfterTheOffset)
{
    const std::vector<uint8_t> raw{0xAA, 0xBB, 0x00, 0x00, 0x00, 0x01, 0x67, 0x42};
    builtin_interfaces::msg::Time stamp;
    stamp.sec = 12;
    stamp.nanosec = 345;

    const auto packet = make_h264_packet(&raw, 2, "left_camera_optical_frame", stamp);

    ASSERT_NE(packet, nullptr);
    EXPECT_EQ(packet->format, "h264");
    EXPECT_EQ(packet->header.frame_id, "left_camera_optical_frame");
    EXPECT_EQ(packet->header.stamp.sec, 12);
    EXPECT_EQ(packet->header.stamp.nanosec, 345u);
    EXPECT_EQ(packet->data, (std::vector<uint8_t>{0x00, 0x00, 0x00, 0x01, 0x67, 0x42}));
}

TEST(H264Packet, ZeroOffsetKeepsWholeBuffer)
{
    const std::vector<uint8_t> raw{1, 2, 3};
    const auto packet = make_h264_packet(&raw, 0, "f", builtin_interfaces::msg::Time{});

    ASSERT_NE(packet, nullptr);
    EXPECT_EQ(packet->data, raw);
}

TEST(H264Packet, EmptyOrMissingAccessUnitIsRejected)
{
    const std::vector<uint8_t> raw{1, 2, 3};
    const builtin_interfaces::msg::Time stamp;

    EXPECT_EQ(make_h264_packet(nullptr, 0, "f", stamp), nullptr);
    EXPECT_EQ(make_h264_packet(&raw, 3, "f", stamp), nullptr);
    EXPECT_EQ(make_h264_packet(&raw, 9, "f", stamp), nullptr);
}
