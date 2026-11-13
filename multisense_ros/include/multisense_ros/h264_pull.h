/**
 * @file h264_pull.h
 *
 * Which decoded sample answers which pushed H.264 frame, and how long to wait for it.
 * Kept free of GStreamer so the policy can be tested without a decoder.
 **/

#pragma once

#include <cstdint>
#include <optional>
#include <utility>

namespace multisense_ros {

// Every S27 H.264 frame is an IDR carrying its own SPS/PPS, so no frame depends on another and
// one not out within this window is dropped. A decode takes ~4 ms on NVDEC once the pipeline is
// running; the first frame after a start can take far longer.
constexpr uint64_t H264_PULL_TIMEOUT_NS = 200'000'000;

// Used while the pipeline looks dead: it keeps a failing stream from stalling every other topic
// the shared publisher thread serves.
constexpr uint64_t H264_PULL_TIMEOUT_DEAD_NS = 20'000'000;

// Consecutive frames that produced no sample of any kind before the pipeline looks dead.
constexpr unsigned H264_SILENT_FRAMES_UNTIL_DEAD = 2;

///
/// @brief Matches decoded samples to the frame just pushed. The caller stamps the result with
///        that frame's capture time, so a sample of an older frame, still queued from a late
///        decode, is dropped and never returned.
///
///        The short window must not follow a single miss. A frame that is slow once, such as the
///        first after a start, leaves its own sample arriving later while the next frames wait
///        only the short window: each frame then pops the previous frame's stale sample and times
///        out before its own, as long as the steady decode latency exceeds the short window, and
///        the stream stays dark until a lucky quick frame. Any sample at all, stale or not, shows
///        the pipeline is alive and resets the count, so the short window is left to a pipeline
///        that stayed silent for several frames running.
///
class H264FrameMatcher
{
public:
    ///
    /// @brief Return the sample whose pts is `pts`, or nullopt when none arrived in time.
    /// @param pull    pull(timeout_ns) -> std::optional<Sample>; nullopt when nothing arrived
    ///                within timeout_ns. Called once per sample plus once for the final timeout.
    /// @param pts_of  pts_of(const Sample &) -> uint64_t
    ///
    template <typename Sample, typename Pull, typename PtsOf>
    std::optional<Sample> match(uint64_t pts, Pull &&pull, PtsOf &&pts_of)
    {
        const uint64_t timeout = silent_frames_ >= H264_SILENT_FRAMES_UNTIL_DEAD ?
                                 H264_PULL_TIMEOUT_DEAD_NS : H264_PULL_TIMEOUT_NS;

        bool any_sample = false;
        std::optional<Sample> matched;
        while (std::optional<Sample> sample = pull(timeout))
        {
            any_sample = true;
            if (pts_of(*sample) == pts)
            {
                matched = std::move(sample);
                break;
            }
        }

        silent_frames_ = any_sample ? 0 : silent_frames_ + 1;
        return matched;
    }

private:
    unsigned silent_frames_ = 0;
};

}
