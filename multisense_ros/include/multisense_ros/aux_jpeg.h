/**
 * @file aux_jpeg.h
 *
 * JPEG of the aux colour image, encoded from the camera's own YCbCr 4:2:0
 * planes: the S27 sends no compressed colour, and a JPEG is 4:2:0 YCbCr
 * already, so no BGR image is built for it.
 **/

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace multisense_ros {

// compressed_image_transport's format string, so image_transport tools decode it to BGR.
constexpr char AUX_JPEG_FORMAT[] = "bgr8; jpeg compressed bgr8";

// A recorder that stalls briefly must not lose frames, and any subscriber QoS matches it.
inline rclcpp::QoS aux_jpeg_qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
}

///
/// @brief What the aux colour thread has to build this frame. Each topic pays only for
///        itself: a JPEG-only subscriber never builds the BGR image.
///
struct AuxColourDemand
{
    bool bgr = false;
    bool jpeg = false;
};

inline AuxColourDemand aux_colour_demand(size_t bgr_subscribers, size_t jpeg_subscribers)
{
    return {bgr_subscribers > 0, jpeg_subscribers > 0};
}

///
/// @brief The camera's chroma is scaled for B = Y + 2.03211 (U - 128) and
///        R = Y + 1.13983 (V - 128); a JPEG decoder applies JFIF's 1.772 and 1.402.
///        These map one onto the other; the green terms then agree as well.
///
struct JfifChromaTables
{
    std::array<uint8_t, 256> cb;
    std::array<uint8_t, 256> cr;
};

const JfifChromaTables &jfif_chroma_tables();

///
/// @brief Write the frame as planar I420 (Y, then Cb, then Cr) with JFIF chroma into
///        dst, which holds width * height * 3 / 2 bytes.
///
void fill_i420(const uint8_t *luma, const uint8_t *chroma, int width, int height, uint8_t *dst);

std::unique_ptr<sensor_msgs::msg::CompressedImage> make_aux_jpeg_message(
    std::vector<uint8_t> jpeg,
    const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp);

class JpegEncoder
{
public:
    ///
    /// @brief True when this build can encode JPEG (it was compiled against GStreamer)
    ///
    static bool available();

    ///
    /// @brief Uses the Jetson's NVJPG engine (nvjpegenc) when present, otherwise libjpeg on
    ///        the CPU (jpegenc). quality is 1-100. Throws if neither encoder exists.
    ///
    JpegEncoder(const std::string &name, int quality);

    ///
    /// @brief The same, trying only the named GStreamer encoders, in order
    ///
    JpegEncoder(const std::string &name, int quality, const std::vector<std::string> &elements);

    ///
    /// @brief True when the GStreamer element exists in this installation
    ///
    static bool has_encoder(const std::string &element);
    ~JpegEncoder();

    JpegEncoder(const JpegEncoder&) = delete;
    JpegEncoder& operator=(const JpegEncoder&) = delete;

    ///
    /// @brief Encode one frame from the camera's planes. Returns nullptr when no JPEG of
    ///        this frame came out in time.
    ///
    std::shared_ptr<std::vector<uint8_t>> encode(const uint8_t *luma,
                                                 const uint8_t *chroma,
                                                 int width,
                                                 int height);

    ///
    /// @brief The GStreamer element doing the encode, for logging
    ///
    const std::string &encoder_element() const { return encoder_element_; }

private:
    struct Pipeline;
    bool start(const std::string &element, int width, int height);

    std::string name_;
    int quality_;
    std::vector<std::string> candidates_;
    std::unique_ptr<Pipeline> pipeline_;
    std::string encoder_element_;
    int width_ = 0;
    int height_ = 0;
};

}
