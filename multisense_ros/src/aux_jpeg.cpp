/**
 * @file aux_jpeg.cpp
 **/

#include <multisense_ros/aux_jpeg.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifdef MULTISENSE_ROS_HAVE_GSTREAMER
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#endif

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
    const auto &lut = jfif_chroma_tables();
    const size_t pixels = static_cast<size_t>(width) * height;
    std::copy(luma, luma + pixels, dst);
    uint8_t *cb = dst + pixels;
    uint8_t *cr = cb + pixels / 4;
    for (size_t i = 0; i < pixels / 4; ++i)
    {
        cb[i] = lut.cb[chroma[2 * i]];
        cr[i] = lut.cr[chroma[2 * i + 1]];
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

#ifdef MULTISENSE_ROS_HAVE_GSTREAMER

namespace {

// A frame not out within this window is dropped. nvjpegenc and jpegenc take
// a few ms on a 1920x1200 frame; the window is shorter after a miss so a
// failing encoder does not hold up the colour thread.
constexpr GstClockTime PULL_TIMEOUT = 200 * GST_MSECOND;
constexpr GstClockTime PULL_TIMEOUT_AFTER_MISS = 20 * GST_MSECOND;

bool has_element(const char *name)
{
    GstElementFactory *factory = gst_element_factory_find(name);
    if (factory == nullptr)
    {
        return false;
    }
    gst_object_unref(factory);
    return true;
}

}

struct JpegEncoder::Pipeline
{
    GstElement *pipeline = nullptr;
    GstElement *src = nullptr;
    GstElement *sink = nullptr;
    guint64 frame_count = 0;
    bool last_missed = false;
    bool delivered = false;

    ~Pipeline()
    {
        if (pipeline != nullptr)
        {
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }
        if (src != nullptr) gst_object_unref(src);
        if (sink != nullptr) gst_object_unref(sink);
        if (pipeline != nullptr) gst_object_unref(pipeline);
    }
};

bool JpegEncoder::available()
{
    return true;
}

bool JpegEncoder::has_encoder(const std::string &element)
{
    gst_init(nullptr, nullptr);
    return has_element(element.c_str());
}

JpegEncoder::JpegEncoder(const std::string &name, int quality):
    JpegEncoder(name, quality, {"nvjpegenc", "jpegenc"})
{
}

JpegEncoder::JpegEncoder(const std::string &name, int quality, const std::vector<std::string> &elements):
    name_(name),
    quality_(quality)
{
    if (quality < 1 || quality > 100)
    {
        throw std::invalid_argument("JPEG encoder " + name + ": quality must be 1-100");
    }
    gst_init(nullptr, nullptr);
    for (const auto &element : elements)
    {
        if (has_element(element.c_str()))
        {
            candidates_.push_back(element);
        }
    }
    if (candidates_.empty())
    {
        throw std::runtime_error("JPEG encoder " + name +
                                 ": no JPEG encoder: install nvjpegenc (Jetson) or gstreamer1.0-plugins-good");
    }
    encoder_element_ = candidates_.front();
}

JpegEncoder::~JpegEncoder() = default;

bool JpegEncoder::start(const std::string &element, int width, int height)
{
    pipeline_.reset();
    const std::string description =
        "appsrc name=src is-live=true format=time caps=video/x-raw,format=I420,width=" +
        std::to_string(width) + ",height=" + std::to_string(height) + ",framerate=0/1 ! " +
        element + " quality=" + std::to_string(quality_) +
        " ! appsink name=sink sync=false max-buffers=2 drop=false";

    GError *error = nullptr;
    auto pipeline = std::make_unique<Pipeline>();
    pipeline->pipeline = gst_parse_launch(description.c_str(), &error);
    if (error != nullptr)
    {
        g_error_free(error);
        return false;
    }
    pipeline->src = gst_bin_get_by_name(GST_BIN(pipeline->pipeline), "src");
    pipeline->sink = gst_bin_get_by_name(GST_BIN(pipeline->pipeline), "sink");
    if (pipeline->src == nullptr || pipeline->sink == nullptr ||
        gst_element_set_state(pipeline->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        return false;
    }
    pipeline_ = std::move(pipeline);
    encoder_element_ = element;
    width_ = width;
    height_ = height;
    return true;
}

std::shared_ptr<std::vector<uint8_t>> JpegEncoder::encode(const uint8_t *luma,
                                                          const uint8_t *chroma,
                                                          int width,
                                                          int height)
{
    if (!pipeline_ || width != width_ || height != height_)
    {
        // An encoder that will not start or never delivers a frame (nvjpegenc refusing
        // system-memory input on some L4T release) gives way to the next one for good.
        while (!candidates_.empty() && !start(candidates_.front(), width, height))
        {
            candidates_.erase(candidates_.begin());
        }
        if (!pipeline_)
        {
            return nullptr;
        }
    }

    const size_t size = static_cast<size_t>(width) * height * 3 / 2;
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
    {
        gst_buffer_unref(buffer);
        return nullptr;
    }
    fill_i420(luma, chroma, width, height, map.data);
    gst_buffer_unmap(buffer, &map);

    const GstClockTime pts = pipeline_->frame_count++ * GST_MSECOND;
    GST_BUFFER_PTS(buffer) = pts;
    if (gst_app_src_push_buffer(GST_APP_SRC(pipeline_->src), buffer) != GST_FLOW_OK)
    {
        return nullptr;
    }

    // The caller stamps the result with this frame's capture time, so an older
    // frame still queued from a late encode must be dropped, never returned.
    const GstClockTime timeout = pipeline_->last_missed ? PULL_TIMEOUT_AFTER_MISS : PULL_TIMEOUT;
    GstSample *sample = nullptr;
    while ((sample = gst_app_sink_try_pull_sample(GST_APP_SINK(pipeline_->sink), timeout)) != nullptr)
    {
        if (GST_BUFFER_PTS(gst_sample_get_buffer(sample)) == pts)
        {
            break;
        }
        gst_sample_unref(sample);
    }
    pipeline_->last_missed = (sample == nullptr);
    if (sample == nullptr)
    {
        if (!pipeline_->delivered && candidates_.size() > 1)
        {
            candidates_.erase(candidates_.begin());
            pipeline_.reset();
        }
        return nullptr;
    }

    std::shared_ptr<std::vector<uint8_t>> out = nullptr;
    GstMapInfo jpeg;
    if (gst_buffer_map(gst_sample_get_buffer(sample), &jpeg, GST_MAP_READ))
    {
        if (jpeg.size > 2 && jpeg.data[0] == 0xFF && jpeg.data[1] == 0xD8)
        {
            out = std::make_shared<std::vector<uint8_t>>(jpeg.data, jpeg.data + jpeg.size);
            pipeline_->delivered = true;
        }
        gst_buffer_unmap(gst_sample_get_buffer(sample), &jpeg);
    }
    gst_sample_unref(sample);
    return out;
}

#else

struct JpegEncoder::Pipeline {};

bool JpegEncoder::available()
{
    return false;
}

bool JpegEncoder::has_encoder(const std::string &)
{
    return false;
}

JpegEncoder::JpegEncoder(const std::string &name, int quality):
    JpegEncoder(name, quality, {})
{
}

JpegEncoder::JpegEncoder(const std::string &name, int, const std::vector<std::string> &):
    quality_(0)
{
    throw std::runtime_error("JPEG encoder " + name + ": multisense_ros was built without GStreamer");
}

JpegEncoder::~JpegEncoder() = default;

bool JpegEncoder::start(const std::string &, int, int)
{
    return false;
}

std::shared_ptr<std::vector<uint8_t>> JpegEncoder::encode(const uint8_t *, const uint8_t *, int, int)
{
    return nullptr;
}

#endif

}
