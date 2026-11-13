/**
 * @file test_h264_pull.cpp
 *
 * The S27 left rectified stream once stayed dark for up to an hour after a session start: one
 * slow first frame, then a decode latency above the short post-miss pull window made every later
 * frame miss. A simulated decoder with virtual time stands in for GStreamer.
 **/

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

#include <multisense_ros/h264_pull.h>

using multisense_ros::H264FrameMatcher;
using multisense_ros::H264_PULL_TIMEOUT_DEAD_NS;
using multisense_ros::H264_PULL_TIMEOUT_NS;

namespace {

constexpr uint64_t MS = 1'000'000;
constexpr uint64_t FRAME_PERIOD = 100 * MS;  // the S27's 10 Hz

struct Sample
{
    uint64_t pts;
};

// In-order decoder in virtual time. A frame pushed at t with latency L is out at t + L, but never
// before the frame ahead of it. A latency of nullopt is a decoder that never answers.
class SimDecoder
{
public:
    using Latency = std::function<std::optional<uint64_t>(uint64_t frame)>;

    explicit SimDecoder(Latency latency): latency_(std::move(latency)) {}

    // Like the driver's publisher thread: frames arrive every FRAME_PERIOD, and a decode that
    // overruns delays the next one rather than overlapping it.
    std::optional<Sample> decode(uint64_t frame)
    {
        now_ = std::max(now_, frame * FRAME_PERIOD);
        if (const auto latency = latency_(frame))
        {
            last_ready_ = std::max(last_ready_, now_ + *latency);
            queue_.push_back({last_ready_, frame});
        }

        return matcher_.match<Sample>(
            frame,
            [this](uint64_t timeout) { return pull(timeout); },
            [](const Sample &s) { return s.pts; });
    }

    const std::vector<uint64_t> &timeouts() const { return timeouts_; }

private:
    struct Ready
    {
        uint64_t at;
        uint64_t pts;
    };

    std::optional<Sample> pull(uint64_t timeout)
    {
        timeouts_.push_back(timeout);
        if (!queue_.empty() && queue_.front().at <= now_ + timeout)
        {
            const Ready ready = queue_.front();
            queue_.pop_front();
            now_ = std::max(now_, ready.at);
            return Sample{ready.pts};
        }
        now_ += timeout;
        return std::nullopt;
    }

    Latency latency_;
    H264FrameMatcher matcher_;
    uint64_t now_ = 0;
    uint64_t last_ready_ = 0;
    std::deque<Ready> queue_;
    std::vector<uint64_t> timeouts_;
};

// Decode `count` frames; one entry per frame, true when it came out.
std::vector<bool> run(SimDecoder &decoder, uint64_t count)
{
    std::vector<bool> decoded;
    for (uint64_t frame = 0; frame < count; ++frame)
    {
        const auto sample = decoder.decode(frame);
        if (sample)
        {
            // Never another frame's pixels under this frame's stamp.
            EXPECT_EQ(sample->pts, frame);
        }
        decoded.push_back(sample.has_value());
    }
    return decoded;
}

size_t drops(const std::vector<bool> &decoded)
{
    return static_cast<size_t>(std::count(decoded.begin(), decoded.end(), false));
}

bool decoded_from(const std::vector<bool> &decoded, size_t first)
{
    return std::all_of(decoded.begin() + static_cast<std::ptrdiff_t>(first), decoded.end(),
                       [](bool ok) { return ok; });
}

}

TEST(H264FrameMatcher, HealthyStreamDecodesEveryFrameOnTheFullWindow)
{
    SimDecoder decoder([](uint64_t) { return std::optional<uint64_t>(5 * MS); });
    const auto decoded = run(decoder, 50);

    EXPECT_EQ(drops(decoded), 0u);
    for (const auto timeout : decoder.timeouts())
    {
        EXPECT_EQ(timeout, H264_PULL_TIMEOUT_NS);
    }
}

// The failure seen on the pod: the first frame out after a start is slow, every later frame
// takes longer than the short window. The stream has to come back, not stay dark.
TEST(H264FrameMatcher, SlowFirstFrameDoesNotLockTheStreamOut)
{
    for (const uint64_t first_ms : {250u, 300u, 600u, 1500u})
    {
        for (const uint64_t steady_ms : {30u, 50u, 150u})
        {
            SCOPED_TRACE("first " + std::to_string(first_ms) + " ms, steady " + std::to_string(steady_ms) + " ms");
            SimDecoder decoder([=](uint64_t frame)
                               { return std::optional<uint64_t>((frame == 0 ? first_ms : steady_ms) * MS); });
            const auto decoded = run(decoder, 100);

            EXPECT_FALSE(decoded.front());
            // Only the frames that arrive while the slow frame is still pending are lost.
            const size_t pending = static_cast<size_t>(first_ms * MS / FRAME_PERIOD) + 1;
            EXPECT_LE(drops(decoded), pending);
            EXPECT_TRUE(decoded_from(decoded, pending));
        }
    }
}

// The same lock-in begun mid-stream by one late frame.
TEST(H264FrameMatcher, OneLateFrameMidStreamRecovers)
{
    SimDecoder decoder([](uint64_t frame) { return std::optional<uint64_t>((frame == 30 ? 400 : 50) * MS); });
    const auto decoded = run(decoder, 100);

    EXPECT_FALSE(decoded[30]);
    EXPECT_TRUE(std::all_of(decoded.begin(), decoded.begin() + 30, [](bool ok) { return ok; }));
    EXPECT_TRUE(decoded_from(decoded, 40));
}

// A decoder that never answers must still report every frame as dropped, never a made-up one, and
// must not hold the publisher thread for the full window on every frame.
TEST(H264FrameMatcher, DeadPipelineDropsEveryFrameAndFallsToTheShortWindow)
{
    SimDecoder decoder([](uint64_t) { return std::optional<uint64_t>(); });
    const auto decoded = run(decoder, 50);

    EXPECT_EQ(drops(decoded), 50u);

    const auto &timeouts = decoder.timeouts();
    ASSERT_EQ(timeouts.size(), 50u);
    EXPECT_EQ(timeouts[0], H264_PULL_TIMEOUT_NS);
    for (size_t i = 2; i < timeouts.size(); ++i)
    {
        EXPECT_EQ(timeouts[i], H264_PULL_TIMEOUT_DEAD_NS) << "frame " << i;
    }
}

TEST(H264FrameMatcher, PipelineThatDiesAfterRunningDropsAndFallsToTheShortWindow)
{
    SimDecoder decoder([](uint64_t frame) { return frame < 10 ? std::optional<uint64_t>(5 * MS) : std::nullopt; });
    const auto decoded = run(decoder, 40);

    EXPECT_EQ(drops(decoded), 30u);
    EXPECT_TRUE(std::all_of(decoded.begin(), decoded.begin() + 10, [](bool ok) { return ok; }));
    EXPECT_EQ(decoder.timeouts().back(), H264_PULL_TIMEOUT_DEAD_NS);
}

// A dead pipeline that comes back, e.g. the camera restarts its stream, decodes again.
TEST(H264FrameMatcher, DeadPipelineThatAnswersAgainDecodes)
{
    SimDecoder decoder([](uint64_t frame) { return frame < 20 || frame >= 30 ? std::optional<uint64_t>(5 * MS) : std::nullopt; });
    const auto decoded = run(decoder, 60);

    EXPECT_TRUE(decoded_from(decoded, 31));
}
