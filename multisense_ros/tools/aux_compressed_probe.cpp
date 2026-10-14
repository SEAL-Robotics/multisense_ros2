/**
 * @file aux_compressed_probe.cpp
 *
 * Asks one MultiSense whether it delivers compressed aux colour: requests each
 * compressed aux source and checks whether its frames carry H.264.
 *
 * Starts streams on the camera, and connecting sends stop-all-streams, so run it
 * only while no other client, such as the ROS driver, is connected to that camera.
 **/

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace lms = multisense;

namespace
{

constexpr int CONTROL_SECONDS = 3;
constexpr int DEFAULT_SECONDS = 6;
constexpr int DEFAULT_MTU = 1500;
constexpr size_t NAL_SCAN_LIMIT = 1 << 16;
constexpr size_t HEAD_BYTES = 24;
constexpr uint8_t NAL_IDR = 5;
constexpr uint8_t NAL_SPS = 7;
constexpr uint8_t NAL_PPS = 8;

struct NalSeen
{
    bool sps = false;
    bool pps = false;
    bool idr = false;

    bool all() const { return sps && pps && idr; }
};

struct Stat
{
    size_t frames = 0;
    size_t empty = 0;
    size_t min_bytes = SIZE_MAX;
    size_t max_bytes = 0;
    size_t total_bytes = 0;
    int width = 0;
    int height = 0;
    lms::Image::PixelFormat format = lms::Image::PixelFormat::UNKNOWN;
    NalSeen after_offset;
    NalSeen whole_buffer;
    bool first_captured = false;
    int64_t first_offset = 0;
    size_t first_length = 0;
    size_t first_buffer = 0;
    std::string first_head;
};

struct PhaseResult
{
    lms::DataSource source = lms::DataSource::UNKNOWN;
    lms::Status started = lms::Status::UNKNOWN;
    size_t callbacks = 0;
    Stat requested;
};

struct Options
{
    std::string ip_address;
    int mtu = DEFAULT_MTU;
    int seconds = DEFAULT_SECONDS;
};

const char *format_name(lms::Image::PixelFormat format)
{
    switch (format)
    {
        case lms::Image::PixelFormat::MONO8: return "MONO8";
        case lms::Image::PixelFormat::BGR8: return "BGR8";
        case lms::Image::PixelFormat::MONO16: return "MONO16";
        case lms::Image::PixelFormat::FLOAT32: return "FLOAT32";
        case lms::Image::PixelFormat::JPEG: return "JPEG";
        case lms::Image::PixelFormat::H264: return "H264";
        default: return "UNKNOWN";
    }
}

NalSeen scan_nals(const uint8_t *data, size_t size)
{
    NalSeen seen;
    const size_t limit = std::min(size, NAL_SCAN_LIMIT);
    for (size_t i = 0; i + 3 < limit; ++i)
    {
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1)
        {
            const uint8_t type = data[i + 3] & 0x1f;
            seen.sps |= type == NAL_SPS;
            seen.pps |= type == NAL_PPS;
            seen.idr |= type == NAL_IDR;
        }
    }
    return seen;
}

std::string hex_head(const uint8_t *data, size_t size)
{
    static const char digits[] = "0123456789abcdef";
    std::string out;
    for (size_t i = 0; i < std::min(size, HEAD_BYTES); ++i)
    {
        out += digits[data[i] >> 4];
        out += digits[data[i] & 0xf];
        out += ' ';
    }
    return out;
}

NalSeen merge(const NalSeen &a, const NalSeen &b)
{
    return {a.sps || b.sps, a.pps || b.pps, a.idr || b.idr};
}

// Frames arrive on the library's thread, so every access to the counters is locked.
class Collector
{
public:
    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.clear();
        callbacks_ = 0;
    }

    void on_frame(const lms::ImageFrame &frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++callbacks_;
        for (const auto &[source, image] : frame.images)
        {
            add_image(stats_[source], image);
        }
    }

    size_t callbacks() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return callbacks_;
    }

    std::map<lms::DataSource, Stat> stats() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

private:
    static void add_image(Stat &stat, const lms::Image &image)
    {
        if (!image.raw_data)
        {
            ++stat.empty;
            return;
        }
        const uint8_t *data = image.raw_data->data();
        const size_t buffer = image.raw_data->size();
        const auto offset = static_cast<size_t>(image.image_data_offset);
        const bool h264 = image.format == lms::Image::PixelFormat::H264;
        if (!stat.first_captured)
        {
            stat.first_captured = true;
            stat.first_offset = image.image_data_offset;
            stat.first_length = image.image_data_length;
            stat.first_buffer = buffer;
            stat.first_head = hex_head(data, buffer);
        }
        if (h264)
        {
            stat.whole_buffer = merge(stat.whole_buffer, scan_nals(data, buffer));
        }
        if (buffer <= offset)
        {
            ++stat.empty;
            return;
        }
        const size_t payload = buffer - offset;
        ++stat.frames;
        stat.min_bytes = std::min(stat.min_bytes, payload);
        stat.max_bytes = std::max(stat.max_bytes, payload);
        stat.total_bytes += payload;
        stat.width = image.width;
        stat.height = image.height;
        stat.format = image.format;
        if (h264)
        {
            stat.after_offset = merge(stat.after_offset, scan_nals(data + offset, payload));
        }
    }

    mutable std::mutex mutex_;
    std::map<lms::DataSource, Stat> stats_;
    size_t callbacks_ = 0;
};

const char *yes_no(bool value)
{
    return value ? "yes" : "no";
}

void print_source(const lms::DataSource &seen, const Stat &stat, bool requested)
{
    std::cout << "  delivered " << lms::to_string(seen) << (requested ? " [requested]" : "") << ": "
              << stat.frames << " frames (" << stat.empty << " without data)";
    if (stat.frames > 0)
    {
        std::cout << ", " << format_name(stat.format) << " " << stat.width << "x" << stat.height
                  << ", bytes/frame min " << stat.min_bytes << " max " << stat.max_bytes << " mean "
                  << stat.total_bytes / stat.frames;
    }
    if (stat.format == lms::Image::PixelFormat::H264)
    {
        std::cout << ", after offset: SPS " << yes_no(stat.after_offset.sps) << " PPS "
                  << yes_no(stat.after_offset.pps) << " IDR " << yes_no(stat.after_offset.idr)
                  << "; whole buffer: SPS " << yes_no(stat.whole_buffer.sps) << " PPS "
                  << yes_no(stat.whole_buffer.pps) << " IDR " << yes_no(stat.whole_buffer.idr);
    }
    std::cout << "\n";
    if (stat.first_captured)
    {
        std::cout << "    first frame: image_data_offset " << stat.first_offset << ", image_data_length "
                  << stat.first_length << ", buffer " << stat.first_buffer << " bytes\n"
                  << "    first " << HEAD_BYTES << " bytes of the buffer: " << stat.first_head << "\n";
    }
}

PhaseResult run_phase(lms::Channel &channel, Collector &collector, lms::DataSource source, int seconds)
{
    collector.reset();
    PhaseResult result;
    result.source = source;
    result.started = channel.start_streams({source});
    if (result.started == lms::Status::OK)
    {
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
    }
    channel.stop_streams({source});

    result.callbacks = collector.callbacks();
    std::cout << "\n" << lms::to_string(source) << "\n  start_streams: " << lms::to_string(result.started)
              << "\n  frame callbacks in " << seconds << " s: " << result.callbacks << "\n";
    const auto stats = collector.stats();
    if (stats.empty())
    {
        std::cout << "  sources delivered: none\n";
    }
    for (const auto &[seen, stat] : stats)
    {
        print_source(seen, stat, seen == source);
        if (seen == source)
        {
            result.requested = stat;
        }
    }
    return result;
}

bool carries_h264(const PhaseResult &result)
{
    return result.requested.frames > 0 && result.requested.format == lms::Image::PixelFormat::H264 &&
           result.requested.after_offset.all();
}

void usage(const char *name)
{
    std::cerr << "USAGE: " << name << " -a <ip_address> [-t <seconds>] [-m <mtu>]\n"
              << "Where <options> are:\n"
              << "\t-a <ip_address> : camera to probe (required)\n"
              << "\t-t <seconds>    : seconds per compressed source (default=" << DEFAULT_SECONDS << ")\n"
              << "\t-m <mtu>        : MTU to use to communicate with the camera (default=" << DEFAULT_MTU
              << ")\n"
              << "Starts streams on the camera: run it only while no ROS driver is connected to it.\n";
}

bool parse_options(int argc, char **argv, Options &options)
{
    int c;
    while (-1 != (c = getopt(argc, argv, "a:t:m:")))
    {
        switch (c)
        {
            case 'a': options.ip_address = optarg; break;
            case 't': options.seconds = std::atoi(optarg); break;
            case 'm': options.mtu = std::atoi(optarg); break;
            default: return false;
        }
    }
    return !options.ip_address.empty() && options.seconds >= 1 && options.seconds <= 60 && options.mtu > 0;
}

int probe(const Options &options)
{
    Collector collector;
    auto channel = lms::Channel::create(lms::Channel::Config{options.ip_address, static_cast<int16_t>(options.mtu)});
    if (!channel)
    {
        std::cerr << "could not connect to " << options.ip_address << "\n";
        return 1;
    }
    channel->add_image_frame_callback([&collector](const lms::ImageFrame &frame) { collector.on_frame(frame); });
    std::cout << "connected to " << options.ip_address << "\n";

    // A compressed stream the camera is known to serve gives the aux byte counts something to be read against.
    const auto control = run_phase(*channel, collector, lms::DataSource::LEFT_RECTIFIED_COMPRESSED, CONTROL_SECONDS);
    if (!carries_h264(control))
    {
        std::cout << "\nthe control stream carries no H.264, so the aux results would mean nothing\n";
        channel->stop_streams({});
        return 3;
    }

    std::vector<PhaseResult> results;
    for (const auto source : {lms::DataSource::AUX_RECTIFIED_COMPRESSED, lms::DataSource::AUX_COMPRESSED})
    {
        results.push_back(run_phase(*channel, collector, source, options.seconds));
    }
    channel->stop_streams({});

    std::cout << "\nVERDICT\n";
    for (const auto &result : results)
    {
        std::cout << "  " << lms::to_string(result.source) << ": "
                  << (carries_h264(result) ? "H.264 present (SPS, PPS and IDR seen)"
                                           : "no decodable H.264 (frames without SPS, PPS and IDR, or none)")
                  << " [start_streams " << lms::to_string(result.started) << ", " << result.requested.frames
                  << " frames]\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!parse_options(argc, argv, options))
    {
        usage(argv[0]);
        return 2;
    }

    std::cerr << "WARNING: this starts streams on " << options.ip_address
              << " and connecting sends stop-all-streams; stop any ROS driver using that camera first.\n";
    try
    {
        return probe(options);
    }
    catch (const std::exception &e)
    {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
