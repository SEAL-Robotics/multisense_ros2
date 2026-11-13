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
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include <multisense_ros/h264_pull.h>

using multisense_ros::describe;
using multisense_ros::H264FrameMatcher;
using multisense_ros::H264PullReport;
using multisense_ros::H264_PULL_TIMEOUT_NS;
using multisense_ros::H264_PULL_TIMEOUT_SHORT_NS;

namespace {

constexpr uint64_t MS = 1'000'000;
constexpr uint64_t FRAME_PERIOD = 100 * MS;  // the S27's 10 Hz

struct Sample
{
    uint64_t pts;
};

struct Model
{
    // Time in ns from push to output of the push-th frame; nullopt is a decoder that never
    // answers. Frames overlap in the pipeline unless `serial`, which makes it the time the
    // decoder spends on the frame after the one ahead of it is out.
    std::function<std::optional<uint64_t>(uint64_t push)> latency;
    bool serial = false;
    // Frame N comes out, labelled N, only once frame N+1 has been pushed and decoded.
    bool one_frame_behind = false;
};

// In-order decoder in virtual time, driven like the driver's publisher thread: a camera frame that
// arrives while the thread is still busy is lost (FrameNotifier keeps no backlog).
class SimDecoder
{
public:
    explicit SimDecoder(Model model): model_(std::move(model)) {}

    struct Result
    {
        std::vector<bool> decoded;           // one per pushed frame
        std::vector<uint64_t> blocked_ns;    // time the thread spent in each decode
        std::vector<H264PullReport> reports;
        size_t lost = 0;                     // camera frames that arrived while the thread was busy
    };

    Result run(uint64_t camera_frames)
    {
        Result result;
        for (uint64_t camera = 0; camera < camera_frames; ++camera)
        {
            const uint64_t arrival = camera * FRAME_PERIOD;
            if (arrival < now_)
            {
                ++result.lost;
                continue;
            }

            const uint64_t started = now_ = arrival;
            const uint64_t pts = pushed_++;
            push(pts);
            const auto sample = matcher_.match<Sample>(
                pts,
                [this](uint64_t timeout) { return pull(timeout); },
                [](const Sample &s) { return s.pts; });
            if (sample)
            {
                // Never another frame's pixels under this frame's stamp.
                EXPECT_EQ(sample->pts, pts);
            }
            result.decoded.push_back(sample.has_value());
            result.blocked_ns.push_back(now_ - started);
            result.reports.push_back(matcher_.report());
        }
        return result;
    }

private:
    struct Ready
    {
        uint64_t at;
        uint64_t pts;
    };

    void push(uint64_t pts)
    {
        const auto latency = model_.latency(pts);
        if (!latency)
        {
            return;
        }
        const uint64_t start = model_.serial ? std::max(last_ready_, now_) : now_;
        last_ready_ = std::max(last_ready_, start + *latency);
        if (model_.one_frame_behind)
        {
            // This push releases the previous frame; this frame waits for the next push.
            if (held_)
            {
                queue_.push_back({last_ready_, *held_});
            }
            held_ = pts;
            return;
        }
        queue_.push_back({last_ready_, pts});
    }

    std::optional<Sample> pull(uint64_t timeout)
    {
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

    Model model_;
    H264FrameMatcher matcher_;
    uint64_t now_ = 0;
    uint64_t pushed_ = 0;
    uint64_t last_ready_ = 0;
    std::optional<uint64_t> held_;
    std::deque<Ready> queue_;
};

Model constant(uint64_t ms)
{
    return {[=](uint64_t) { return std::optional<uint64_t>(ms * MS); }, false, false};
}

size_t drops(const std::vector<bool> &decoded)
{
    return static_cast<size_t>(std::count(decoded.begin(), decoded.end(), false));
}

bool decoded_from(const std::vector<bool> &decoded, size_t first)
{
    return first <= decoded.size() &&
           std::all_of(decoded.begin() + static_cast<std::ptrdiff_t>(first), decoded.end(),
                       [](bool ok) { return ok; });
}

uint64_t mean(const std::vector<uint64_t> &values, size_t from)
{
    return std::accumulate(values.begin() + static_cast<std::ptrdiff_t>(from), values.end(), uint64_t{0}) /
           (values.size() - from);
}

}

TEST(H264FrameMatcher, HealthyStreamDecodesEveryFrameOnTheFullWindow)
{
    SimDecoder decoder(constant(5));
    const auto result = decoder.run(50);

    EXPECT_EQ(drops(result.decoded), 0u);
    EXPECT_EQ(result.lost, 0u);
    for (const auto &report : result.reports)
    {
        EXPECT_EQ(report.timeout_ns, H264_PULL_TIMEOUT_NS);
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
            SimDecoder decoder({[=](uint64_t push)
                                { return std::optional<uint64_t>((push == 0 ? first_ms : steady_ms) * MS); },
                                false, false});
            const auto result = decoder.run(100);

            EXPECT_FALSE(result.decoded.front());
            // Only the frames pushed while the slow frame is still pending are lost.
            const size_t pending = static_cast<size_t>(first_ms * MS / FRAME_PERIOD) + 1;
            EXPECT_LE(drops(result.decoded), pending);
            EXPECT_TRUE(decoded_from(result.decoded, pending));
        }
    }
}

// The same lock-in begun mid-stream by one late frame.
TEST(H264FrameMatcher, OneLateFrameMidStreamRecovers)
{
    SimDecoder decoder({[](uint64_t push) { return std::optional<uint64_t>((push == 30 ? 400 : 50) * MS); },
                        false, false});
    const auto result = decoder.run(100);

    EXPECT_FALSE(result.decoded[30]);
    EXPECT_TRUE(std::all_of(result.decoded.begin(), result.decoded.begin() + 30, [](bool ok) { return ok; }));
    EXPECT_TRUE(decoded_from(result.decoded, 36));
}

// A decoder that never answers must still report every frame as dropped, never a made-up one, and
// must not hold the publisher thread for the full window on every frame.
TEST(H264FrameMatcher, DeadPipelineDropsEveryFrameAndFallsToTheShortWindow)
{
    SimDecoder decoder({[](uint64_t) { return std::optional<uint64_t>(); }, false, false});
    const auto result = decoder.run(50);

    EXPECT_EQ(drops(result.decoded), result.decoded.size());
    EXPECT_EQ(result.reports[0].timeout_ns, H264_PULL_TIMEOUT_NS);
    for (size_t i = 2; i < result.reports.size(); ++i)
    {
        EXPECT_EQ(result.reports[i].timeout_ns, H264_PULL_TIMEOUT_SHORT_NS) << "push " << i;
        EXPECT_EQ(result.reports[i].outcome, H264PullReport::Outcome::NO_SAMPLE);
    }
}

TEST(H264FrameMatcher, PipelineThatDiesAfterRunningDropsAndFallsToTheShortWindow)
{
    SimDecoder decoder({[](uint64_t push) { return push < 10 ? std::optional<uint64_t>(5 * MS) : std::nullopt; },
                        false, false});
    const auto result = decoder.run(40);

    EXPECT_TRUE(std::all_of(result.decoded.begin(), result.decoded.begin() + 10, [](bool ok) { return ok; }));
    EXPECT_EQ(drops(result.decoded), result.decoded.size() - 10);
    EXPECT_EQ(result.reports.back().timeout_ns, H264_PULL_TIMEOUT_SHORT_NS);
}

// A dead pipeline that comes back, e.g. the camera restarts its stream, decodes again.
TEST(H264FrameMatcher, DeadPipelineThatAnswersAgainDecodes)
{
    SimDecoder decoder({[](uint64_t push)
                        { return push < 20 || push >= 30 ? std::optional<uint64_t>(5 * MS) : std::nullopt; },
                        false, false});
    const auto result = decoder.run(60);

    EXPECT_TRUE(decoded_from(result.decoded, 31));
}

// Running but never in time: every sample is of an older frame. The stream is dark either way, so
// the shared publisher thread must not pay the full window for every frame of it.
TEST(H264FrameMatcher, PipelineAlwaysLateDropsEveryFrameWithoutBlockingTheThread)
{
    SimDecoder decoder({[](uint64_t) { return std::optional<uint64_t>(250 * MS); }, true, false});
    const auto result = decoder.run(200);

    EXPECT_EQ(drops(result.decoded), result.decoded.size());
    EXPECT_LT(mean(result.blocked_ns, 20), 60 * MS);
    EXPECT_NE(result.reports.back().outcome, H264PullReport::Outcome::MATCHED);
}

TEST(H264FrameMatcher, PipelineOneFrameBehindDropsEveryFrameWithoutBlockingTheThread)
{
    SimDecoder decoder({[](uint64_t) { return std::optional<uint64_t>(5 * MS); }, false, true});
    const auto result = decoder.run(200);

    // Frame 0 is released by frame 1, so only frame N-1 is ever out when frame N waits for it.
    EXPECT_EQ(drops(result.decoded), result.decoded.size());
    EXPECT_LT(mean(result.blocked_ns, 20), 60 * MS);
    EXPECT_EQ(result.reports.back().outcome, H264PullReport::Outcome::STALE_ONLY);
    EXPECT_EQ(result.reports.back().frames_behind, 1u);
}

// A late pipeline gets the full window now and then: once it catches up it must decode again
// rather than stay on the short window.
TEST(H264FrameMatcher, LatePipelineThatCatchesUpDecodesAgain)
{
    SimDecoder decoder({[](uint64_t push) { return std::optional<uint64_t>((push < 15 ? 250 : 50) * MS); },
                        true, false});
    const auto result = decoder.run(300);

    EXPECT_TRUE(std::any_of(result.decoded.begin() + 15, result.decoded.end(), [](bool ok) { return ok; }));
    EXPECT_TRUE(decoded_from(result.decoded, 100));
}

// The warning has to tell the pipeline producing nothing from one producing the wrong frames.
TEST(H264FrameMatcher, ReportSaysWhyTheFrameWasDropped)
{
    SimDecoder dead({[](uint64_t) { return std::optional<uint64_t>(); }, false, false});
    const auto dead_report = dead.run(1).reports.back();
    EXPECT_EQ(dead_report.outcome, H264PullReport::Outcome::NO_SAMPLE);
    EXPECT_NE(describe(dead_report).find("no sample"), std::string::npos);

    SimDecoder behind({[](uint64_t) { return std::optional<uint64_t>(5 * MS); }, false, true});
    const auto behind_report = behind.run(3).reports.back();
    EXPECT_EQ(behind_report.outcome, H264PullReport::Outcome::STALE_ONLY);
    EXPECT_EQ(behind_report.frames_behind, 1u);
    EXPECT_NE(describe(behind_report).find("1 frame(s) behind"), std::string::npos);
}
