/**
 * @file test_aux_jpeg_worker.cpp
 *
 * The aux JPEG is encoded off the colour thread, so a slow or stalled encoder
 * never holds up aux/image_rect_color, the line detector's input.
 **/

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <multisense_ros/aux_jpeg.h>

using namespace multisense_ros;
using Clock = std::chrono::steady_clock;

namespace {

constexpr int W = 64;
constexpr int H = 32;

AuxJpegJob job(int32_t sec)
{
    AuxJpegJob j;
    j.luma = std::make_shared<std::vector<uint8_t>>(W * H, 100);
    j.chroma = std::make_shared<std::vector<uint8_t>>(W * H / 2, 128);
    j.width = W;
    j.height = H;
    j.stamp.sec = sec;
    return j;
}

// Stands in for the encoder: blocks until released, then returns a fixed JPEG.
struct Gate
{
    std::mutex mutex;
    std::condition_variable cv;
    bool open = false;
    std::atomic<int> calls{0};

    std::shared_ptr<std::vector<uint8_t>> encode()
    {
        ++calls;
        std::unique_lock<std::mutex> lock{mutex};
        cv.wait(lock, [this] { return open; });
        return std::make_shared<std::vector<uint8_t>>(std::vector<uint8_t>{0xFF, 0xD8, 0xFF, 0xD9});
    }

    void release()
    {
        {
            std::lock_guard<std::mutex> lock{mutex};
            open = true;
        }
        cv.notify_all();
    }
};

}

TEST(AuxJpegWorker, PostingNeverWaitsForTheEncoder)
{
    Gate gate;
    std::vector<int32_t> published;
    std::mutex published_mutex;
    {
        AuxJpegWorker worker{
            [&](const AuxJpegJob &) { return gate.encode(); },
            [&](std::vector<uint8_t>, const builtin_interfaces::msg::Time &stamp)
            {
                std::lock_guard<std::mutex> lock{published_mutex};
                published.push_back(stamp.sec);
            }};
        worker.post(job(1));
        while (gate.calls == 0)
        {
            std::this_thread::yield();
        }
        const auto start = Clock::now();
        for (int32_t i = 2; i <= 20; ++i)
        {
            worker.post(job(i));
        }
        EXPECT_LT(Clock::now() - start, std::chrono::milliseconds{50});
        gate.release();
        worker.wait_idle();
    }
    // The first frame was in the encoder; of the rest only the latest is kept.
    EXPECT_EQ(published, (std::vector<int32_t>{1, 20}));
}

TEST(AuxJpegWorker, AFrameTheEncoderDropsIsNotPublished)
{
    int published = 0;
    AuxJpegWorker worker{
        [](const AuxJpegJob &) { return std::shared_ptr<std::vector<uint8_t>>{}; },
        [&](std::vector<uint8_t>, const builtin_interfaces::msg::Time &) { ++published; }};
    worker.post(job(1));
    worker.wait_idle();
    EXPECT_EQ(published, 0);
}

TEST(AuxJpegWorker, StopsWithAFramePending)
{
    Gate gate;
    auto worker = std::make_unique<AuxJpegWorker>(
        [&](const AuxJpegJob &) { return gate.encode(); },
        [](std::vector<uint8_t>, const builtin_interfaces::msg::Time &) {});
    worker->post(job(1));
    worker->post(job(2));
    gate.release();
    worker.reset();   // must return, not hang on the pending frame
    SUCCEED();
}
