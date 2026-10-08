/**
 * @file h264_decoder.h
 *
 * Decodes the MultiSense's H.264 image sources back to MONO8 so they publish on
 * the same topics, in the same encoding, as the raw sources they replace.
 **/

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace multisense_ros {

class H264Decoder
{
public:
    ///
    /// @brief True when this build can decode H.264 (it was compiled against GStreamer)
    ///
    static bool available();

    ///
    /// @brief One GStreamer pipeline per image source. Uses the Jetson's hardware decoder
    ///        (nvv4l2decoder) when present, otherwise libav on the CPU. Throws if neither exists.
    ///
    explicit H264Decoder(const std::string &name);
    ~H264Decoder();

    H264Decoder(const H264Decoder&) = delete;
    H264Decoder& operator=(const H264Decoder&) = delete;

    ///
    /// @brief Decode one access unit. Returns the tightly packed MONO8 image, or nullptr
    ///        when the decoder produced no frame of the expected size in time.
    ///
    std::shared_ptr<std::vector<uint8_t>> decode(const uint8_t *data,
                                                 size_t size,
                                                 int width,
                                                 int height);

    ///
    /// @brief The GStreamer element doing the decode, for logging
    ///
    const std::string &decoder_element() const { return decoder_element_; }

private:
    struct Pipeline;
    std::unique_ptr<Pipeline> pipeline_;
    std::string decoder_element_;
};

}
