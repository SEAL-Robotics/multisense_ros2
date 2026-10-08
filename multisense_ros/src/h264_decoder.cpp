/**
 * @file h264_decoder.cpp
 **/

#include <multisense_ros/h264_decoder.h>

#include <cstring>
#include <stdexcept>

#ifdef MULTISENSE_ROS_HAVE_GSTREAMER
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#endif

namespace multisense_ros {

#ifdef MULTISENSE_ROS_HAVE_GSTREAMER

namespace {

// Every S27 H.264 frame is an IDR carrying its own SPS/PPS, so no frame
// depends on another and one not out within this window is dropped. After a
// miss the short window keeps a failing stream from stalling every other
// topic the shared publisher thread serves; a decode takes ~4 ms on NVDEC.
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

struct H264Decoder::Pipeline
{
    GstElement *pipeline = nullptr;
    GstElement *src = nullptr;
    GstElement *sink = nullptr;
    guint64 frame_count = 0;
    bool last_missed = false;

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

bool H264Decoder::available()
{
    return true;
}

H264Decoder::H264Decoder(const std::string &name):
    pipeline_(std::make_unique<Pipeline>())
{
    gst_init(nullptr, nullptr);

    std::string decode;
    if (has_element("nvv4l2decoder") && has_element("nvvidconv"))
    {
        // disable-dpb: with intra-only input the reorder buffer only adds a frame of latency.
        decode = "nvv4l2decoder disable-dpb=true ! nvvidconv";
        decoder_element_ = "nvv4l2decoder";
    }
    else if (has_element("avdec_h264"))
    {
        // One thread: frame threading holds frames back to fill its pipeline.
        decode = "avdec_h264 max-threads=1 ! videoconvert";
        decoder_element_ = "avdec_h264";
    }
    else
    {
        throw std::runtime_error("no H.264 decoder: install nvv4l2decoder (Jetson) or gstreamer1.0-libav");
    }

    const std::string description =
        "appsrc name=src is-live=true format=time "
        "caps=video/x-h264,stream-format=byte-stream,alignment=au ! h264parse ! " + decode +
        " ! video/x-raw,format=GRAY8 ! appsink name=sink sync=false max-buffers=2 drop=false";

    GError *error = nullptr;
    pipeline_->pipeline = gst_parse_launch(description.c_str(), &error);
    if (error != nullptr)
    {
        const std::string message = error->message;
        g_error_free(error);
        throw std::runtime_error("H.264 decoder " + name + ": " + message);
    }

    pipeline_->src = gst_bin_get_by_name(GST_BIN(pipeline_->pipeline), "src");
    pipeline_->sink = gst_bin_get_by_name(GST_BIN(pipeline_->pipeline), "sink");
    if (pipeline_->src == nullptr || pipeline_->sink == nullptr ||
        gst_element_set_state(pipeline_->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        throw std::runtime_error("H.264 decoder " + name + ": pipeline failed to start");
    }
}

H264Decoder::~H264Decoder() = default;

std::shared_ptr<std::vector<uint8_t>> H264Decoder::decode(const uint8_t *data,
                                                          size_t size,
                                                          int width,
                                                          int height)
{
    const GstClockTime pts = pipeline_->frame_count++ * GST_MSECOND;
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    gst_buffer_fill(buffer, 0, data, size);
    GST_BUFFER_PTS(buffer) = pts;
    if (gst_app_src_push_buffer(GST_APP_SRC(pipeline_->src), buffer) != GST_FLOW_OK)
    {
        return nullptr;
    }

    // The caller stamps the result with this frame's capture time, so an older
    // frame still queued from a late decode must be dropped, never returned.
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
        return nullptr;
    }

    std::shared_ptr<std::vector<uint8_t>> out = nullptr;
    GstVideoInfo info;
    GstVideoFrame frame;
    if (gst_video_info_from_caps(&info, gst_sample_get_caps(sample)) &&
        GST_VIDEO_INFO_WIDTH(&info) == width && GST_VIDEO_INFO_HEIGHT(&info) == height &&
        gst_video_frame_map(&frame, &info, gst_sample_get_buffer(sample), GST_MAP_READ))
    {
        // The decoder pads rows to its own stride; ROS images are tightly packed.
        const auto *plane = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
        const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
        out = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(width) * height);
        for (int row = 0; row < height; ++row)
        {
            std::memcpy(out->data() + static_cast<size_t>(row) * width, plane + static_cast<size_t>(row) * stride, width);
        }
        gst_video_frame_unmap(&frame);
    }

    gst_sample_unref(sample);
    return out;
}

#else

struct H264Decoder::Pipeline {};

bool H264Decoder::available()
{
    return false;
}

H264Decoder::H264Decoder(const std::string &name)
{
    throw std::runtime_error("H.264 decoder " + name + ": multisense_ros was built without GStreamer");
}

H264Decoder::~H264Decoder() = default;

std::shared_ptr<std::vector<uint8_t>> H264Decoder::decode(const uint8_t *, size_t, int, int)
{
    return nullptr;
}

#endif

}
