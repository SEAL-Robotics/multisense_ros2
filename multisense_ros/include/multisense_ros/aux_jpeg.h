/**
 * @file aux_jpeg.h
 *
 * JPEG of the aux colour image, encoded from the camera's own YCbCr 4:2:0
 * planes: the S27 sends no compressed colour, and a JPEG is 4:2:0 YCbCr
 * already, so no BGR image is built for it.
 **/

#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
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

///
/// @brief The same into planes with their own row strides, as GStreamer lays I420 out
///
void fill_i420(const uint8_t *luma, const uint8_t *chroma, int width, int height,
               uint8_t *y, size_t y_stride, uint8_t *cb, size_t cb_stride, uint8_t *cr, size_t cr_stride);

std::unique_ptr<sensor_msgs::msg::CompressedImage> make_aux_jpeg_message(
    std::vector<uint8_t> jpeg,
    const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp);

///
/// @brief Whether a sample the encoder hands back is the frame just pushed. An encoder that
///        keeps timestamps is matched on them; one that drops them (sample_pts empty) can
///        only be matched when that frame is the only one out.
///
inline bool accept_sample(std::optional<uint64_t> sample_pts, uint64_t pushed_pts, uint64_t in_flight)
{
    return sample_pts ? *sample_pts == pushed_pts : in_flight == 1;
}

///
/// @brief A GStreamer encoder to try: element names it for logs, description is the pipeline
///        fragment between appsrc (I420) and appsink (image/jpeg).
///
struct EncoderCandidate
{
    std::string element;
    std::string description;
};

struct JpegEncoderOptions
{
    // NVJPG and CUDA set up on the first frame, which can take a second or more.
    std::chrono::milliseconds first_frame_timeout{2000};
    std::chrono::milliseconds frame_timeout{200};
    // Short after a miss, so a failing encoder does not hold up the frames behind it.
    std::chrono::milliseconds timeout_after_miss{20};
    // Misses before a first delivery that give up a candidate for the next one.
    int misses_before_fallback = 3;
    // Frames waiting for the encoder at most; a full queue drops its oldest frame.
    size_t max_queued_frames = 2;
    // Frames in a row with no JPEG, after a first one, that tear the pipeline down and start it again.
    int failures_before_restart = 10;
    // When no candidate will start, how long before trying them all again.
    std::chrono::milliseconds start_retry{5000};
};

using WarningSink = std::function<void(const std::string &)>;

class JpegEncoder
{
public:
    ///
    /// @brief True when this build can encode JPEG (it was compiled against GStreamer)
    ///
    static bool available();

    ///
    /// @brief True when the GStreamer element exists in this installation
    ///
    static bool has_encoder(const std::string &element);

    ///
    /// @brief nvjpegenc (the Jetson's NVJPG engine), then jpegenc (libjpeg on the CPU), those
    ///        of them installed here, at quality 1-100. Empty when neither is.
    ///
    static std::vector<EncoderCandidate> default_candidates(int quality);

    JpegEncoder(const std::string &name, int quality);

    ///
    /// @brief The named elements, in order, at quality 1-100
    ///
    JpegEncoder(const std::string &name, int quality, const std::vector<std::string> &elements);

    ///
    /// @brief The installed ones of candidates, in order. warn hears each fallback.
    ///        Throws when none is installed.
    ///
    JpegEncoder(const std::string &name,
                std::vector<EncoderCandidate> candidates,
                JpegEncoderOptions options,
                WarningSink warn);
    ~JpegEncoder();

    JpegEncoder(const JpegEncoder&) = delete;
    JpegEncoder& operator=(const JpegEncoder&) = delete;

    ///
    /// @brief Encode one frame from the camera's planes. Returns nullptr when no JPEG of
    ///        this frame came out in time, or the encoder is still busy with earlier ones.
    ///
    std::shared_ptr<std::vector<uint8_t>> encode(const uint8_t *luma,
                                                 const uint8_t *chroma,
                                                 int width,
                                                 int height);

    ///
    /// @brief The GStreamer element doing the encode, for logging
    ///
    const std::string &encoder_element() const;

    ///
    /// @brief Frames waiting in front of the encoder now
    ///
    size_t queued_frames() const;

private:
    struct Pipeline;
    bool start(size_t candidate, int width, int height);
    void fall_back(const std::string &why);
    void note_failure();

    std::string name_;
    JpegEncoderOptions options_;
    WarningSink warn_;
    std::vector<EncoderCandidate> candidates_;
    size_t current_ = 0;
    std::unique_ptr<Pipeline> pipeline_;
    int width_ = 0;
    int height_ = 0;
    std::chrono::steady_clock::time_point retry_after_{};
};

///
/// @brief One frame for the aux JPEG worker: the camera's planes, kept alive by their buffers
///
struct AuxJpegJob
{
    std::shared_ptr<const std::vector<uint8_t>> luma;
    size_t luma_offset = 0;
    std::shared_ptr<const std::vector<uint8_t>> chroma;
    size_t chroma_offset = 0;
    int width = 0;
    int height = 0;
    builtin_interfaces::msg::Time stamp;
};

///
/// @brief Encodes on its own thread, so a slow or stalled encoder never holds up the thread
///        that publishes aux/image_rect_color. Holds one frame waiting: a newer one replaces it.
///
class AuxJpegWorker
{
public:
    using Encode = std::function<std::shared_ptr<std::vector<uint8_t>>(const AuxJpegJob &)>;
    using Publish = std::function<void(std::vector<uint8_t>, const builtin_interfaces::msg::Time &)>;
    using Error = std::function<void(const std::string &)>;

    ///
    /// @brief error hears whatever encode or publish throws; the frame is dropped and the
    ///        worker carries on
    ///
    AuxJpegWorker(Encode encode, Publish publish, Error error = nullptr);
    ~AuxJpegWorker();

    AuxJpegWorker(const AuxJpegWorker&) = delete;
    AuxJpegWorker& operator=(const AuxJpegWorker&) = delete;

    ///
    /// @brief Hand over a frame; never blocks on the encoder
    ///
    void post(AuxJpegJob job);

    ///
    /// @brief Block until nothing is waiting or being encoded
    ///
    void wait_idle();

private:
    void run();

    Encode encode_;
    Publish publish_;
    Error error_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::condition_variable idle_;
    std::optional<AuxJpegJob> pending_;
    bool busy_ = false;
    bool stop_ = false;
    std::thread thread_;
};

}
