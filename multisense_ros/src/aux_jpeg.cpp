/**
 * @file aux_jpeg.cpp
 **/

#include <multisense_ros/aux_jpeg.h>

#include <algorithm>
#include <cmath>
#include <csetjmp>
#include <cstdio>
#include <cstring>
#include <new>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

// jpeglib.h needs FILE and size_t declared first.
#include <jpeglib.h>
#include <jerror.h>

namespace multisense_ros {

namespace {

uint8_t rescale(int value, double factor)
{
    const double scaled = 128.0 + (value - 128) * factor;
    return static_cast<uint8_t>(std::clamp(std::lround(scaled), 0L, 255L));
}

}

const JfifChromaTables &jfif_chroma_tables()
{
    static const JfifChromaTables tables = []()
    {
        JfifChromaTables t{};
        for (int v = 0; v < 256; ++v)
        {
            t.cb[v] = rescale(v, 2.03211 / 1.772);
            t.cr[v] = rescale(v, 1.13983 / 1.402);
        }
        return t;
    }();
    return tables;
}

void fill_i420(const uint8_t *luma, const uint8_t *chroma, int width, int height, uint8_t *dst)
{
    const size_t pixels = static_cast<size_t>(width) * height;
    fill_i420(luma, chroma, width, height, dst, width, dst + pixels, width / 2,
              dst + pixels + pixels / 4, width / 2);
}

void fill_i420(const uint8_t *luma, const uint8_t *chroma, int width, int height,
               uint8_t *y, size_t y_stride, uint8_t *cb, size_t cb_stride, uint8_t *cr, size_t cr_stride)
{
    const auto &lut = jfif_chroma_tables();
    const size_t w = static_cast<size_t>(width);
    for (size_t row = 0; row < static_cast<size_t>(height); ++row)
    {
        std::copy(luma + row * w, luma + (row + 1) * w, y + row * y_stride);
    }
    for (size_t row = 0; row < static_cast<size_t>(height) / 2; ++row)
    {
        const uint8_t *c = chroma + row * w;   // w / 2 Cb, Cr pairs
        uint8_t *b = cb + row * cb_stride;
        uint8_t *r = cr + row * cr_stride;
        for (size_t col = 0; col < w / 2; ++col)
        {
            b[col] = lut.cb[c[2 * col]];
            r[col] = lut.cr[c[2 * col + 1]];
        }
    }
}

std::unique_ptr<sensor_msgs::msg::CompressedImage> make_aux_jpeg_message(
    std::vector<uint8_t> jpeg,
    const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp)
{
    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id;
    msg->format = AUX_JPEG_FORMAT;
    msg->data = std::move(jpeg);
    return msg;
}

AuxJpegWorker::AuxJpegWorker(Encode encode, Publish publish, Error error):
    encode_(std::move(encode)),
    publish_(std::move(publish)),
    error_(std::move(error)),
    thread_(&AuxJpegWorker::run, this)
{
}

AuxJpegWorker::~AuxJpegWorker()
{
    {
        std::lock_guard<std::mutex> lock{mutex_};
        stop_ = true;
        pending_.reset();
    }
    wake_.notify_all();
    thread_.join();
}

void AuxJpegWorker::post(AuxJpegJob job)
{
    {
        std::lock_guard<std::mutex> lock{mutex_};
        pending_ = std::move(job);
    }
    wake_.notify_one();
}

void AuxJpegWorker::wait_idle()
{
    std::unique_lock<std::mutex> lock{mutex_};
    idle_.wait(lock, [this] { return !pending_ && !busy_; });
}

void AuxJpegWorker::run()
{
    while (true)
    {
        AuxJpegJob job;
        {
            std::unique_lock<std::mutex> lock{mutex_};
            wake_.wait(lock, [this] { return stop_ || pending_; });
            if (stop_)
            {
                return;
            }
            job = std::move(*pending_);
            pending_.reset();
            busy_ = true;
        }
        // Escaping this thread would terminate the driver.
        try
        {
            if (auto jpeg = encode_(job))
            {
                publish_(std::move(*jpeg), job.stamp);
            }
        }
        catch (const std::exception &e)
        {
            if (error_) error_(std::string{"aux JPEG frame dropped: "} + e.what());
        }
        catch (...)
        {
            if (error_) error_("aux JPEG frame dropped: unknown exception");
        }
        {
            std::lock_guard<std::mutex> lock{mutex_};
            busy_ = false;
        }
        idle_.notify_all();
    }
}

namespace {

size_t round_up(size_t value, size_t multiple)
{
    return (value + multiple - 1) / multiple * multiple;
}

// libjpeg reports errors by calling error_exit, which must not return; C++ exceptions
// cannot cross its C frames, so it jumps back to encode_planes instead.
struct ErrorManager
{
    jpeg_error_mgr pub;
    std::jmp_buf jump;
    char message[JMSG_LENGTH_MAX];
};

void error_exit(j_common_ptr cinfo)
{
    auto *err = reinterpret_cast<ErrorManager *>(cinfo->err);
    (*cinfo->err->format_message)(cinfo, err->message);
    std::longjmp(err->jump, 1);
}

// Writes straight into the message's byte vector.
struct VectorDestination
{
    jpeg_destination_mgr pub;
    std::vector<uint8_t> *out;
};

void init_destination(j_compress_ptr cinfo)
{
    auto *dest = reinterpret_cast<VectorDestination *>(cinfo->dest);
    dest->pub.next_output_byte = dest->out->data();
    dest->pub.free_in_buffer = dest->out->size();
}

boolean empty_output_buffer(j_compress_ptr cinfo)
{
    auto *dest = reinterpret_cast<VectorDestination *>(cinfo->dest);
    const size_t used = dest->out->size();
    bool grown = false;
    try
    {
        dest->out->resize(used * 2);
        grown = true;
    }
    catch (const std::bad_alloc &)
    {
    }
    // Outside the handler: jumping out of one would leak the exception.
    if (!grown)
    {
        ERREXIT(cinfo, JERR_OUT_OF_MEMORY);
    }
    dest->pub.next_output_byte = dest->out->data() + used;
    dest->pub.free_in_buffer = dest->out->size() - used;
    return TRUE;
}

void term_destination(j_compress_ptr cinfo)
{
    auto *dest = reinterpret_cast<VectorDestination *>(cinfo->dest);
    dest->out->resize(dest->out->size() - dest->pub.free_in_buffer);
}

struct Planes
{
    uint8_t *data[3];
    size_t stride[3];
    size_t rows[3];
};

// No C++ object with a destructor may live in this frame: longjmp skips destructors.
bool encode_planes(const Planes &planes, int width, int height, int quality,
                   std::vector<uint8_t> *out, char (&message)[JMSG_LENGTH_MAX])
{
    jpeg_compress_struct cinfo;
    ErrorManager err;
    VectorDestination dest;
    cinfo.err = jpeg_std_error(&err.pub);
    err.pub.error_exit = error_exit;
    if (setjmp(err.jump))
    {
        jpeg_destroy_compress(&cinfo);
        std::memcpy(message, err.message, sizeof(message));
        return false;
    }
    jpeg_create_compress(&cinfo);
    dest.pub.init_destination = init_destination;
    dest.pub.empty_output_buffer = empty_output_buffer;
    dest.pub.term_destination = term_destination;
    dest.out = out;
    cinfo.dest = &dest.pub;

    cinfo.image_width = static_cast<JDIMENSION>(width);
    cinfo.image_height = static_cast<JDIMENSION>(height);
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;
    jpeg_set_defaults(&cinfo);
    jpeg_set_colorspace(&cinfo, JCS_YCbCr);
    jpeg_set_quality(&cinfo, quality, TRUE);
    // The planes are already 4:2:0 YCbCr, so libjpeg neither converts nor downsamples.
    cinfo.raw_data_in = TRUE;
    cinfo.comp_info[0].h_samp_factor = 2;
    cinfo.comp_info[0].v_samp_factor = 2;
    for (int c = 1; c < 3; ++c)
    {
        cinfo.comp_info[c].h_samp_factor = 1;
        cinfo.comp_info[c].v_samp_factor = 1;
    }
    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW rows[3][2 * DCTSIZE];
    JSAMPARRAY groups[3] = {rows[0], rows[1], rows[2]};
    while (cinfo.next_scanline < cinfo.image_height)
    {
        const size_t luma_row = cinfo.next_scanline;
        for (int c = 0; c < 3; ++c)
        {
            const size_t first = c == 0 ? luma_row : luma_row / 2;
            const int count = c == 0 ? 2 * DCTSIZE : DCTSIZE;
            for (int r = 0; r < count; ++r)
            {
                rows[c][r] = planes.data[c] + (first + r) * planes.stride[c];
            }
        }
        jpeg_write_raw_data(&cinfo, groups, 2 * DCTSIZE);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    return true;
}

// Repeats the last column and row into the MCU padding: a flat edge block encodes without
// ringing into the picture.
void pad(uint8_t *plane, size_t width, size_t height, size_t stride, size_t rows)
{
    for (size_t row = 0; row < height; ++row)
    {
        uint8_t *line = plane + row * stride;
        std::fill(line + width, line + stride, line[width - 1]);
    }
    for (size_t row = height; row < rows; ++row)
    {
        std::copy(plane + (height - 1) * stride, plane + height * stride, plane + row * stride);
    }
}

void check_size(int width, int height)
{
    if (width < 2 || height < 2 || width % 2 != 0 || height % 2 != 0)
    {
        throw std::invalid_argument("JPEG encoder: 4:2:0 needs an even width and height, got " +
                                    std::to_string(width) + "x" + std::to_string(height));
    }
}

}

JpegEncoder::JpegEncoder(int quality):
    quality_(quality)
{
    if (quality < 1 || quality > 100)
    {
        throw std::invalid_argument("JPEG encoder: quality must be 1-100");
    }
}

std::shared_ptr<std::vector<uint8_t>> JpegEncoder::encode(const uint8_t *luma,
                                                          const uint8_t *chroma,
                                                          int width,
                                                          int height)
{
    check_size(width, height);
    const size_t w = static_cast<size_t>(width);
    const size_t h = static_cast<size_t>(height);
    const size_t mcu = 2 * DCTSIZE;
    Planes planes{};
    planes.stride[0] = round_up(w, mcu);
    planes.rows[0] = round_up(h, mcu);
    planes.stride[1] = planes.stride[2] = planes.stride[0] / 2;
    planes.rows[1] = planes.rows[2] = planes.rows[0] / 2;
    const size_t luma_size = planes.stride[0] * planes.rows[0];
    const size_t chroma_size = planes.stride[1] * planes.rows[1];
    planes_.resize(luma_size + 2 * chroma_size);
    planes.data[0] = planes_.data();
    planes.data[1] = planes.data[0] + luma_size;
    planes.data[2] = planes.data[1] + chroma_size;

    fill_i420(luma, chroma, width, height, planes.data[0], planes.stride[0],
              planes.data[1], planes.stride[1], planes.data[2], planes.stride[2]);
    pad(planes.data[0], w, h, planes.stride[0], planes.rows[0]);
    for (int c = 1; c < 3; ++c)
    {
        pad(planes.data[c], w / 2, h / 2, planes.stride[c], planes.rows[c]);
    }

    // About a third of the frame's pixel count holds a quality-75 JPEG of a camera scene.
    auto out = std::make_shared<std::vector<uint8_t>>(w * h / 3 + 4096);
    char message[JMSG_LENGTH_MAX] = {};
    if (!encode_planes(planes, width, height, quality_, out.get(), message))
    {
        throw std::runtime_error(std::string{"JPEG encoder: libjpeg: "} + message);
    }
    return out;
}

#ifndef MULTISENSE_ROS_HAVE_NVJPG

struct NvJpegEncoder::Impl {};

std::unique_ptr<NvJpegEncoder> NvJpegEncoder::create(int, int, int, std::string *error)
{
    *error = "multisense_ros was built without the Jetson Multimedia API";
    return nullptr;
}

NvJpegEncoder::NvJpegEncoder(std::unique_ptr<Impl> impl):
    impl_(std::move(impl))
{
}

NvJpegEncoder::~NvJpegEncoder() = default;

std::shared_ptr<std::vector<uint8_t>> NvJpegEncoder::encode(const uint8_t *, const uint8_t *, int, int)
{
    throw std::logic_error("NVJPG: multisense_ros was built without the Jetson Multimedia API");
}

#endif

double decoded_psnr(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b)
{
    const cv::Mat first = cv::imdecode(a, cv::IMREAD_COLOR);
    const cv::Mat second = cv::imdecode(b, cv::IMREAD_COLOR);
    if (first.empty() || second.empty() || first.size() != second.size())
    {
        return 0.0;
    }
    return cv::PSNR(first, second);
}

namespace {

// Two encoders of one noisy, textured frame at quality 75 decode about 38 dB apart, and even
// quality 30 against 75 about 26 dB; swapped chroma planes land near 16 dB, and a wrong pitch
// or size lower still.
constexpr double MIN_HARDWARE_PSNR_DB = 25.0;

constexpr char CPU_ENCODER[] = "libjpeg-turbo";

// A frame no wrong pitch, plane order or size survives: a checkerboard over luma ramps, Cb
// ramping across and Cr down. A camera's first frames can be flat, so the hardware is checked
// on this, never on them.
void check_frame(int width, int height, std::vector<uint8_t> &luma, std::vector<uint8_t> &chroma)
{
    const size_t w = static_cast<size_t>(width);
    const size_t h = static_cast<size_t>(height);
    luma.resize(w * h);
    chroma.resize(w * h / 2);
    for (size_t y = 0; y < h; ++y)
    {
        for (size_t x = 0; x < w; ++x)
        {
            luma[y * w + x] = static_cast<uint8_t>(40 + 120 * x / w + 40 * y / h + ((x / 8 + y / 8) % 2) * 50);
        }
    }
    for (size_t y = 0; y < h / 2; ++y)
    {
        for (size_t x = 0; x < w / 2; ++x)
        {
            chroma[2 * (y * w / 2 + x)] = static_cast<uint8_t>(28 + 200 * x / (w / 2));
            chroma[2 * (y * w / 2 + x) + 1] = static_cast<uint8_t>(28 + 200 * y / (h / 2));
        }
    }
}

bool is_jpeg(const std::vector<uint8_t> &data)
{
    const size_t n = data.size();
    return n >= 4 && data[0] == 0xFF && data[1] == 0xD8 && data[n - 2] == 0xFF && data[n - 1] == 0xD9;
}

std::string decibels(double psnr)
{
    char text[32];
    std::snprintf(text, sizeof(text), "%.1f dB", psnr);
    return text;
}

}

AuxJpegEncoder::HardwareFactory AuxJpegEncoder::nvjpg(int quality)
{
    return [quality](int width, int height, std::string *error) -> JpegFrameEncoder
    {
        std::shared_ptr<NvJpegEncoder> encoder = NvJpegEncoder::create(quality, width, height, error);
        if (!encoder)
        {
            return nullptr;
        }
        return [encoder](const uint8_t *luma, const uint8_t *chroma, int w, int h)
        {
            return encoder->encode(luma, chroma, w, h);
        };
    };
}

AuxJpegEncoder::AuxJpegEncoder(int quality, HardwareFactory hardware, std::string hardware_name, WarningSink warn):
    cpu_(quality),
    factory_(std::move(hardware)),
    hardware_name_(std::move(hardware_name)),
    warn_(std::move(warn))
{
}

void AuxJpegEncoder::give_up_hardware(const std::string &why)
{
    factory_ = nullptr;
    hardware_ = nullptr;
    if (warn_)
    {
        warn_(why + "; encoding the aux JPEG with " + CPU_ENCODER + " from now on");
    }
}

std::string AuxJpegEncoder::check_hardware(const JpegFrameEncoder &hardware, int width, int height)
{
    std::vector<uint8_t> luma, chroma;
    check_frame(width, height, luma, chroma);
    const auto jpeg = hardware(luma.data(), chroma.data(), width, height);
    if (!jpeg || !is_jpeg(*jpeg))
    {
        return hardware_name_ + " returned no JPEG of its check frame";
    }
    const double psnr = decoded_psnr(*jpeg, *cpu_.encode(luma.data(), chroma.data(), width, height));
    if (psnr < MIN_HARDWARE_PSNR_DB)
    {
        return hardware_name_ + "'s JPEG of its check frame decodes " + decibels(psnr) + " from " +
               CPU_ENCODER + "'s";
    }
    return {};
}

std::shared_ptr<std::vector<uint8_t>> AuxJpegEncoder::encode_on_hardware(const uint8_t *luma,
                                                                         const uint8_t *chroma,
                                                                         int width,
                                                                         int height)
{
    try
    {
        if (!hardware_ || width != width_ || height != height_)
        {
            hardware_ = nullptr;
            std::string error;
            JpegFrameEncoder made = factory_(width, height, &error);
            if (!made)
            {
                give_up_hardware(hardware_name_ + " unavailable: " + error);
                return nullptr;
            }
            // Before any frame reaches a subscriber: an engine that runs but reads its
            // surface wrong would otherwise publish a wrong picture all session.
            const std::string fault = check_hardware(made, width, height);
            if (!fault.empty())
            {
                give_up_hardware(fault);
                return nullptr;
            }
            hardware_ = std::move(made);
            width_ = width;
            height_ = height;
        }
        auto jpeg = hardware_(luma, chroma, width, height);
        if (!jpeg || !is_jpeg(*jpeg))
        {
            give_up_hardware(hardware_name_ + " returned no JPEG");
            return nullptr;
        }
        encoder_name_ = hardware_name_;
        return jpeg;
    }
    catch (const std::exception &e)
    {
        give_up_hardware(hardware_name_ + " failed: " + e.what());
        return nullptr;
    }
}

std::shared_ptr<std::vector<uint8_t>> AuxJpegEncoder::encode(const uint8_t *luma,
                                                             const uint8_t *chroma,
                                                             int width,
                                                             int height)
{
    check_size(width, height);
    if (factory_)
    {
        if (auto jpeg = encode_on_hardware(luma, chroma, width, height))
        {
            return jpeg;
        }
    }
    auto jpeg = cpu_.encode(luma, chroma, width, height);
    encoder_name_ = CPU_ENCODER;
    return jpeg;
}

}
