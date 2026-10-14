#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace multisense_ros {

constexpr char H264_PACKET_FORMAT[] = "h264";

// A recorder that stalls briefly must not silently lose frames, so the packet topics are
// reliable with about a second of history at the S27's 10 Hz, unlike the best-effort images.
constexpr size_t H264_PACKET_QOS_DEPTH = 10;

inline rclcpp::QoS h264_packet_qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(H264_PACKET_QOS_DEPTH)).reliable();
}

///
/// @brief What an H.264 image source has to do this frame. A packet-only subscriber must not
///        wake the decoder, so decoding follows the decoded topic alone.
///
struct H264Demand
{
    bool decode = false;
    bool packet = false;
};

inline H264Demand h264_demand(size_t decoded_subscribers, size_t packet_subscribers)
{
    return {decoded_subscribers > 0, packet_subscribers > 0};
}

///
/// @brief Wrap the encoded access unit at raw[offset..] as published by the camera. Returns
///        nullptr when the buffer holds no access unit. The stamp and frame id are those of
///        the decoded image for the same frame.
///
inline std::unique_ptr<sensor_msgs::msg::CompressedImage> make_h264_packet(
    const std::vector<uint8_t> *raw,
    size_t offset,
    const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp)
{
    if (!raw || raw->size() <= offset)
    {
        return nullptr;
    }

    auto packet = std::make_unique<sensor_msgs::msg::CompressedImage>();
    packet->header.stamp = stamp;
    packet->header.frame_id = frame_id;
    packet->format = H264_PACKET_FORMAT;
    packet->data.assign(raw->begin() + static_cast<std::ptrdiff_t>(offset), raw->end());
    return packet;
}

}
