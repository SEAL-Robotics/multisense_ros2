/**
 * @file h264_pull.h
 *
 * Which decoded sample answers which pushed H.264 frame, and how long to wait for it.
 * Kept free of GStreamer so the policy can be tested without a decoder.
 **/

#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace multisense_ros {

// Every S27 H.264 frame is an IDR carrying its own SPS/PPS, so no frame depends on another and
// one not out within this window is dropped. The decode latency from push to appsink on the pod
// has not been measured; the documented NVDEC cost is a few ms per frame.
constexpr uint64_t H264_PULL_TIMEOUT_NS = 200'000'000;

// Used while the pipeline looks dead or hopelessly late: it keeps a failing stream from stalling
// every other topic the shared publisher thread serves.
constexpr uint64_t H264_PULL_TIMEOUT_SHORT_NS = 20'000'000;

// Consecutive frames that produced no sample of any kind before the pipeline looks dead.
constexpr unsigned H264_SILENT_FRAMES_UNTIL_DEAD = 2;

// Consecutive frames whose only samples were of older frames before the pipeline looks late, and
// how often a late pipeline gets the full window anyway to notice it caught up.
constexpr unsigned H264_LATE_FRAMES_UNTIL_SHORT = 3;
constexpr unsigned H264_LATE_FRAMES_PROBE_EVERY = 10;

///
/// @brief What the last match saw, for the drop warning: a miss can mean the pipeline produced
///        nothing, or produced only samples of older frames, and the two have different causes.
///
struct H264PullReport
{
    enum class Outcome
    {
        NONE,        ///< nothing matched yet
        MATCHED,
        NO_SAMPLE,   ///< nothing came out within the window
        STALE_ONLY   ///< samples came out, all of older frames
    };

    Outcome outcome = Outcome::NONE;
    uint64_t timeout_ns = 0;        ///< window of each pull
    unsigned stale_samples = 0;     ///< older samples popped
    uint64_t frames_behind = 0;     ///< pushed frame minus the newest stale sample, in frames
    unsigned silent_frames = 0;     ///< consecutive frames with no sample at all, including this one
    unsigned late_frames = 0;       ///< consecutive frames with only stale samples, including this one
};

inline std::string describe(const H264PullReport &report)
{
    using Outcome = H264PullReport::Outcome;
    const std::string window = std::to_string(report.timeout_ns / 1'000'000) + " ms window";
    switch (report.outcome)
    {
        case Outcome::NO_SAMPLE:
            return "no sample out in the " + window + " (" + std::to_string(report.silent_frames) +
                   " frames running with none)";
        case Outcome::STALE_ONLY:
            return std::to_string(report.stale_samples) + " sample(s) of older frames out in the " + window +
                   ", newest " + std::to_string(report.frames_behind) + " frame(s) behind (" +
                   std::to_string(report.late_frames) + " frames running late)";
        case Outcome::MATCHED:
            return "matched";
        case Outcome::NONE:
            break;
    }
    return "no match attempted";
}

///
/// @brief Matches decoded samples to the frame just pushed. The caller stamps the result with
///        that frame's capture time, so a sample of an older frame, still queued from a late
///        decode, is dropped and never returned.
///
///        The short window must not follow a single miss. A frame that is slow once, such as the
///        first after a start, leaves its own sample arriving later while the next frames wait
///        only the short window: each frame then pops the previous frame's stale sample and times
///        out before its own, as long as the steady decode latency exceeds the short window, and
///        the stream stays dark until a lucky quick frame. So a pipeline is given the full window
///        until it has stayed silent, or has been late with every frame, for several frames
///        running. A late pipeline still gets the full window every few frames, because the
///        cost of the short window is a stream that stays dark after latency recovers.
///
class H264FrameMatcher
{
public:
    ///
    /// @param pts_per_frame  pts distance between consecutive frames, to report gaps in frames
    ///
    explicit H264FrameMatcher(uint64_t pts_per_frame = 1): pts_per_frame_(std::max<uint64_t>(pts_per_frame, 1)) {}

    ///
    /// @brief Return the sample whose pts is `pts`, or nullopt when none arrived in time.
    /// @param pull    pull(timeout_ns) -> std::optional<Sample>; nullopt when nothing arrived
    ///                within timeout_ns. Called once per sample plus once for the final timeout.
    /// @param pts_of  pts_of(const Sample &) -> uint64_t
    ///
    template <typename Sample, typename Pull, typename PtsOf>
    std::optional<Sample> match(uint64_t pts, Pull &&pull, PtsOf &&pts_of)
    {
        const bool dead = silent_frames_ >= H264_SILENT_FRAMES_UNTIL_DEAD;
        const bool late = late_frames_ >= H264_LATE_FRAMES_UNTIL_SHORT &&
                          late_frames_ % H264_LATE_FRAMES_PROBE_EVERY != 0;
        const uint64_t timeout = dead || late ? H264_PULL_TIMEOUT_SHORT_NS : H264_PULL_TIMEOUT_NS;

        unsigned stale_samples = 0;
        uint64_t newest_stale = 0;
        std::optional<Sample> matched;
        while (std::optional<Sample> sample = pull(timeout))
        {
            const uint64_t sample_pts = pts_of(*sample);
            if (sample_pts == pts)
            {
                matched = std::move(sample);
                break;
            }
            ++stale_samples;
            newest_stale = std::max(newest_stale, sample_pts);
        }

        const bool any_sample = matched || stale_samples > 0;
        silent_frames_ = any_sample ? 0 : silent_frames_ + 1;
        late_frames_ = matched ? 0 : stale_samples > 0 ? late_frames_ + 1 : late_frames_;

        report_.outcome = matched ? H264PullReport::Outcome::MATCHED :
                          any_sample ? H264PullReport::Outcome::STALE_ONLY : H264PullReport::Outcome::NO_SAMPLE;
        report_.timeout_ns = timeout;
        report_.stale_samples = stale_samples;
        report_.frames_behind = stale_samples > 0 && pts >= newest_stale ? (pts - newest_stale) / pts_per_frame_ : 0;
        report_.silent_frames = silent_frames_;
        report_.late_frames = late_frames_;
        return matched;
    }

    const H264PullReport &report() const { return report_; }

private:
    uint64_t pts_per_frame_;
    unsigned silent_frames_ = 0;
    unsigned late_frames_ = 0;
    H264PullReport report_;
};

}
