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
#include <gst/video/video.h>
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

std::vector<EncoderCandidate> JpegEncoder::default_candidates(int quality)
{
    std::vector<EncoderCandidate> out;
    for (const char *element : {"nvjpegenc", "jpegenc"})
    {
        if (has_encoder(element))
        {
            out.push_back({element, std::string{element} + " quality=" + std::to_string(quality)});
        }
    }
    return out;
}

namespace {

void check_quality(const std::string &name, int quality)
{
    if (quality < 1 || quality > 100)
    {
        throw std::invalid_argument("JPEG encoder " + name + ": quality must be 1-100");
    }
}

std::vector<EncoderCandidate> named(const std::vector<std::string> &elements, int quality)
{
    std::vector<EncoderCandidate> out;
    for (const auto &element : elements)
    {
        out.push_back({element, element + " quality=" + std::to_string(quality)});
    }
    return out;
}

}

JpegEncoder::JpegEncoder(const std::string &name, int quality):
    JpegEncoder(name, quality, std::vector<std::string>{"nvjpegenc", "jpegenc"})
{
}

JpegEncoder::JpegEncoder(const std::string &name, int quality, const std::vector<std::string> &elements):
    JpegEncoder(name, (check_quality(name, quality), named(elements, quality)), JpegEncoderOptions{}, nullptr)
{
}

#ifdef MULTISENSE_ROS_HAVE_GSTREAMER

namespace {

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

// The first word of a pipeline fragment, which is the element that must exist.
std::string first_element(const std::string &description)
{
    return description.substr(0, description.find_first_of(" !"));
}

GstClockTime to_gst(std::chrono::milliseconds timeout)
{
    return static_cast<GstClockTime>(timeout.count()) * GST_MSECOND;
}

bool has_property(GstElement *element, const char *property)
{
    return g_object_class_find_property(G_OBJECT_GET_CLASS(element), property) != nullptr;
}

}

struct JpegEncoder::Pipeline
{
    GstElement *pipeline = nullptr;
    GstElement *src = nullptr;
    GstElement *sink = nullptr;
    GstBufferPool *pool = nullptr;
    GstVideoInfo info{};
    guint64 frame_count = 0;
    uint64_t in_flight = 0;
    int misses = 0;
    int failures = 0;
    bool last_missed = false;
    bool delivered = false;

    ~Pipeline()
    {
        if (pipeline != nullptr)
        {
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }
        if (pool != nullptr)
        {
            gst_buffer_pool_set_active(pool, FALSE);
            gst_object_unref(pool);
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

JpegEncoder::JpegEncoder(const std::string &name,
                         std::vector<EncoderCandidate> candidates,
                         JpegEncoderOptions options,
                         WarningSink warn):
    name_(name),
    options_(options),
    warn_(std::move(warn))
{
    gst_init(nullptr, nullptr);
    for (auto &candidate : candidates)
    {
        if (has_element(first_element(candidate.description).c_str()))
        {
            candidates_.push_back(std::move(candidate));
        }
    }
    if (candidates_.empty())
    {
        throw std::runtime_error("JPEG encoder " + name +
                                 ": no JPEG encoder: install nvjpegenc (Jetson) or gstreamer1.0-plugins-good");
    }
}

JpegEncoder::~JpegEncoder() = default;

const std::string &JpegEncoder::encoder_element() const
{
    return candidates_[current_].element;
}

size_t JpegEncoder::queued_frames() const
{
    if (!pipeline_ || !has_property(pipeline_->src, "current-level-buffers"))
    {
        return 0;
    }
    guint64 level = 0;
    g_object_get(pipeline_->src, "current-level-buffers", &level, nullptr);
    return static_cast<size_t>(level);
}

bool JpegEncoder::start(size_t candidate, int width, int height)
{
    pipeline_.reset();
    const std::string caps = "video/x-raw,format=I420,width=" + std::to_string(width) +
                             ",height=" + std::to_string(height) + ",framerate=0/1";
    const std::string description =
        "appsrc name=src is-live=true format=time caps=" + caps + " ! " +
        candidates_[candidate].description +
        " ! appsink name=sink sync=false max-buffers=2 drop=true";

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
    if (pipeline->src == nullptr || pipeline->sink == nullptr)
    {
        return false;
    }

    // Bounded twice: the queue in front of the encoder drops its oldest frame when full, and the
    // frames themselves come from a fixed pool, so a stalled encoder cannot grow memory.
    if (has_property(pipeline->src, "max-buffers") && has_property(pipeline->src, "leaky-type"))
    {
        g_object_set(pipeline->src, "max-buffers", static_cast<guint64>(options_.max_queued_frames),
                     "leaky-type", GST_APP_LEAKY_TYPE_DOWNSTREAM, nullptr);
    }
    // GStreamer pads each I420 row to 4 bytes, so a frame is laid out by its video info, not packed.
    gst_video_info_set_format(&pipeline->info, GST_VIDEO_FORMAT_I420, static_cast<guint>(width),
                              static_cast<guint>(height));
    GstCaps *pool_caps = gst_caps_from_string(caps.c_str());
    pipeline->pool = gst_buffer_pool_new();
    GstStructure *config = gst_buffer_pool_get_config(pipeline->pool);
    gst_buffer_pool_config_set_params(config, pool_caps, static_cast<guint>(GST_VIDEO_INFO_SIZE(&pipeline->info)), 0,
                                      static_cast<guint>(options_.max_queued_frames + 2));
    gst_caps_unref(pool_caps);
    if (!gst_buffer_pool_set_config(pipeline->pool, config) ||
        !gst_buffer_pool_set_active(pipeline->pool, TRUE) ||
        gst_element_set_state(pipeline->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        return false;
    }
    pipeline_ = std::move(pipeline);
    width_ = width;
    height_ = height;
    return true;
}

void JpegEncoder::fall_back(const std::string &why)
{
    const std::string from = candidates_[current_].element;
    pipeline_.reset();
    ++current_;
    if (current_ >= candidates_.size())
    {
        // None will start: wait before trying them all again rather than on every frame.
        current_ = 0;
        retry_after_ = std::chrono::steady_clock::now() + options_.start_retry;
    }
    if (warn_)
    {
        warn_("JPEG encoder " + name_ + ": " + from + " " + why +
              (current_ > 0 ? "; falling back to " + candidates_[current_].element
                            : "; no encoder left, retrying in " +
                                  std::to_string(options_.start_retry.count()) + " ms"));
    }
}

void JpegEncoder::note_failure()
{
    // An encoder that has delivered and then stops (a wedged NVJPG, a pool it never returns)
    // would otherwise drop every frame from here on.
    if (!pipeline_ || !pipeline_->delivered || ++pipeline_->failures < options_.failures_before_restart)
    {
        return;
    }
    if (warn_)
    {
        warn_("JPEG encoder " + name_ + ": " + candidates_[current_].element + " gave no JPEG in " +
              std::to_string(pipeline_->failures) + " frames; restarting it");
    }
    pipeline_.reset();
}

std::shared_ptr<std::vector<uint8_t>> JpegEncoder::encode(const uint8_t *luma,
                                                          const uint8_t *chroma,
                                                          int width,
                                                          int height)
{
    if (width != width_ || height != height_)
    {
        // A new mode may suit the preferred encoder that an earlier one did not.
        pipeline_.reset();
        current_ = 0;
        retry_after_ = {};
        width_ = width;
        height_ = height;
    }
    if (!pipeline_ && std::chrono::steady_clock::now() < retry_after_)
    {
        return nullptr;
    }
    while (!pipeline_)
    {
        const size_t trying = current_;
        if (start(trying, width, height))
        {
            break;
        }
        fall_back("did not start");
        if (current_ == 0)
        {
            return nullptr;
        }
    }
    Pipeline &p = *pipeline_;

    // Whatever came out after an earlier frame gave up waiting is for an older stamp.
    while (GstSample *stale = gst_app_sink_try_pull_sample(GST_APP_SINK(p.sink), 0))
    {
        gst_sample_unref(stale);
        p.in_flight = p.in_flight > 0 ? p.in_flight - 1 : 0;
    }

    GstBuffer *buffer = nullptr;
    GstBufferPoolAcquireParams params{};
    params.flags = GST_BUFFER_POOL_ACQUIRE_FLAG_DONTWAIT;
    if (gst_buffer_pool_acquire_buffer(p.pool, &buffer, &params) != GST_FLOW_OK)
    {
        // Every pooled frame is still with the encoder: it is behind, so this frame is dropped.
        note_failure();
        return nullptr;
    }
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
    {
        gst_buffer_unref(buffer);
        return nullptr;
    }
    const GstVideoInfo &info = p.info;
    fill_i420(luma, chroma, width, height,
              map.data + GST_VIDEO_INFO_PLANE_OFFSET(&info, 0), GST_VIDEO_INFO_PLANE_STRIDE(&info, 0),
              map.data + GST_VIDEO_INFO_PLANE_OFFSET(&info, 1), GST_VIDEO_INFO_PLANE_STRIDE(&info, 1),
              map.data + GST_VIDEO_INFO_PLANE_OFFSET(&info, 2), GST_VIDEO_INFO_PLANE_STRIDE(&info, 2));
    gst_buffer_unmap(buffer, &map);

    const GstClockTime pts = p.frame_count++ * GST_MSECOND;
    GST_BUFFER_PTS(buffer) = pts;
    if (gst_app_src_push_buffer(GST_APP_SRC(p.src), buffer) != GST_FLOW_OK)
    {
        return nullptr;
    }
    ++p.in_flight;

    const GstClockTime timeout = !p.delivered ? to_gst(options_.first_frame_timeout)
                               : p.last_missed ? to_gst(options_.timeout_after_miss)
                               : to_gst(options_.frame_timeout);
    GstSample *sample = nullptr;
    while ((sample = gst_app_sink_try_pull_sample(GST_APP_SINK(p.sink), timeout)) != nullptr)
    {
        const GstClockTime sample_pts = GST_BUFFER_PTS(gst_sample_get_buffer(sample));
        const bool ours = accept_sample(
            GST_CLOCK_TIME_IS_VALID(sample_pts) ? std::optional<uint64_t>{sample_pts} : std::nullopt,
            pts, p.in_flight);
        p.in_flight = p.in_flight > 0 ? p.in_flight - 1 : 0;
        if (ours)
        {
            break;
        }
        gst_sample_unref(sample);
    }
    p.last_missed = (sample == nullptr);
    if (sample == nullptr)
    {
        // A leaky queue drops frames without a sample, so recount what is really out.
        p.in_flight = queued_frames() + 1;
        if (!p.delivered && ++p.misses >= options_.misses_before_fallback &&
            current_ + 1 < candidates_.size())
        {
            fall_back("delivered no frame in " + std::to_string(p.misses) + " tries");
            return nullptr;
        }
        note_failure();
        return nullptr;
    }

    std::shared_ptr<std::vector<uint8_t>> out = nullptr;
    GstMapInfo jpeg;
    if (gst_buffer_map(gst_sample_get_buffer(sample), &jpeg, GST_MAP_READ))
    {
        if (jpeg.size > 2 && jpeg.data[0] == 0xFF && jpeg.data[1] == 0xD8)
        {
            out = std::make_shared<std::vector<uint8_t>>(jpeg.data, jpeg.data + jpeg.size);
            p.delivered = true;
            p.misses = 0;
            p.failures = 0;
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

JpegEncoder::JpegEncoder(const std::string &name, std::vector<EncoderCandidate>, JpegEncoderOptions, WarningSink)
{
    throw std::runtime_error("JPEG encoder " + name + ": multisense_ros was built without GStreamer");
}

JpegEncoder::~JpegEncoder() = default;

const std::string &JpegEncoder::encoder_element() const
{
    return name_;
}

size_t JpegEncoder::queued_frames() const
{
    return 0;
}

bool JpegEncoder::start(size_t, int, int)
{
    return false;
}

void JpegEncoder::fall_back(const std::string &)
{
}

void JpegEncoder::note_failure()
{
}

std::shared_ptr<std::vector<uint8_t>> JpegEncoder::encode(const uint8_t *, const uint8_t *, int, int)
{
    return nullptr;
}

#endif

}
