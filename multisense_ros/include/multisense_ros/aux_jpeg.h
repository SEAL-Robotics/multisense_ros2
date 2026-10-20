/**
 * @file aux_jpeg.h
 *
 * JPEG of the aux colour image, encoded from the camera's own YCbCr 4:2:0
 * planes: the S27 sends no compressed colour, and a JPEG is 4:2:0 YCbCr
 * already, so no BGR image is built for it.
 **/

#pragma once

#include <array>
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
/// @brief The same into planes with their own row strides
///
void fill_i420(const uint8_t *luma, const uint8_t *chroma, int width, int height,
               uint8_t *y, size_t y_stride, uint8_t *cb, size_t cb_stride, uint8_t *cr, size_t cr_stride);

std::unique_ptr<sensor_msgs::msg::CompressedImage> make_aux_jpeg_message(
    std::vector<uint8_t> jpeg,
    const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp);

///
/// @brief JPEG from the camera's planes on libjpeg-turbo, the libjpeg the driver already has
///        loaded through OpenCV, on the CPU
///
class JpegEncoder
{
public:
    ///
    /// @brief quality 1-100; throws std::invalid_argument outside it
    ///
    explicit JpegEncoder(int quality);

    ///
    /// @brief Encode one frame: full-size luma and half-size interleaved CbCr on the camera's
    ///        scale, width and height even. Throws std::invalid_argument for any other size
    ///        and std::runtime_error when libjpeg fails.
    ///
    std::shared_ptr<std::vector<uint8_t>> encode(const uint8_t *luma,
                                                 const uint8_t *chroma,
                                                 int width,
                                                 int height);

    int quality() const { return quality_; }

private:
    int quality_;
    // Y, Cb and Cr padded out to whole MCUs, which libjpeg's raw input reads.
    std::vector<uint8_t> planes_;
};

///
/// @brief JPEG on NVJPG, the Jetson's JPEG engine. Its library, libnvmm_jpeg, exports the
///        libjpeg names at libjpeg 9's ABI, so it is opened local to this class and reached
///        only through its own symbols. Linked normally, or loaded by GStreamer's nvjpegenc,
///        its callers bind to the driver's libjpeg-turbo instead ("JPEG parameter struct
///        mismatch") and the driver dies.
///
class NvJpegEncoder
{
public:
    ///
    /// @brief An encoder for frames of one size, or nullptr with error set where this build
    ///        or this host has no NVJPG
    ///
    static std::unique_ptr<NvJpegEncoder> create(int quality, int width, int height, std::string *error);

    ~NvJpegEncoder();

    NvJpegEncoder(const NvJpegEncoder&) = delete;
    NvJpegEncoder& operator=(const NvJpegEncoder&) = delete;

    ///
    /// @brief As JpegEncoder::encode, at the size it was created for. Throws
    ///        std::runtime_error when NVJPG fails.
    ///
    std::shared_ptr<std::vector<uint8_t>> encode(const uint8_t *luma,
                                                 const uint8_t *chroma,
                                                 int width,
                                                 int height);

private:
    struct Impl;
    explicit NvJpegEncoder(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

using JpegFrameEncoder = std::function<std::shared_ptr<std::vector<uint8_t>>(
    const uint8_t *luma, const uint8_t *chroma, int width, int height)>;
using WarningSink = std::function<void(const std::string &)>;

///
/// @brief The aux JPEG encoder the driver runs: a hardware encoder while it works,
///        libjpeg-turbo otherwise. A hardware encoder that does not start, throws, returns
///        something not a whole JPEG, or whose JPEG of a check frame does not decode to the
///        picture libjpeg-turbo's does is given up for good, with one warning, and that frame
///        is encoded on the CPU instead.
///
class AuxJpegEncoder
{
public:
    ///
    /// @brief Makes the hardware encoder for frames of one size, or returns empty with error set
    ///
    using HardwareFactory = std::function<JpegFrameEncoder(int width, int height, std::string *error)>;

    ///
    /// @brief NVJPG, through NvJpegEncoder
    ///
    static HardwareFactory nvjpg(int quality);

    ///
    /// @brief quality 1-100; an empty hardware encodes on libjpeg-turbo alone
    ///
    AuxJpegEncoder(int quality, HardwareFactory hardware, std::string hardware_name, WarningSink warn);

    ///
    /// @brief As JpegEncoder::encode, on whichever encoder is current
    ///
    std::shared_ptr<std::vector<uint8_t>> encode(const uint8_t *luma,
                                                 const uint8_t *chroma,
                                                 int width,
                                                 int height);

    ///
    /// @brief The encoder the last JPEG came from: the hardware's name or "libjpeg-turbo"
    ///
    const std::string &encoder_name() const { return encoder_name_; }

private:
    std::shared_ptr<std::vector<uint8_t>> encode_on_hardware(const uint8_t *luma,
                                                             const uint8_t *chroma,
                                                             int width,
                                                             int height);
    void give_up_hardware(const std::string &why);
    // Empty when the hardware encodes a check frame as libjpeg-turbo does, else why not
    std::string check_hardware(const JpegFrameEncoder &hardware, int width, int height);

    JpegEncoder cpu_;
    HardwareFactory factory_;
    std::string hardware_name_;
    WarningSink warn_;
    JpegFrameEncoder hardware_;
    int width_ = 0;
    int height_ = 0;
    std::string encoder_name_;
};

///
/// @brief PSNR in dB between two JPEGs' decoded pictures; 0 when either does not decode
///        or their sizes differ
///
double decoded_psnr(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b);

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
