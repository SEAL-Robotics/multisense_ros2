/**
 * @file test_aux_jpeg.cpp
 *
 * The driver's aux JPEG takes the place of pod_hl_image_jpeg's re-encode of the
 * raw BGR image: same topic contract, a picture at least as close to the raw
 * image as that re-encode was.
 **/

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <MultiSense/MultiSenseUtilities.hh>

#include <multisense_ros/aux_jpeg.h>
#include <multisense_ros/ycbcr.h>

namespace lms = multisense;
using namespace multisense_ros;

namespace {

constexpr int W = 640;
constexpr int H = 400;

// A smooth colour scene with texture, written as the S27 sends it: full-size
// luma and half-size interleaved chroma on the camera's own YUV scale.
void camera_planes(std::vector<uint8_t> &luma, std::vector<uint8_t> &chroma)
{
    std::mt19937 rng{3};
    std::normal_distribution<double> noise{0.0, 6.0};
    cv::Mat bgr(H, W, CV_8UC3);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(
                cv::saturate_cast<uint8_t>(40 + 180.0 * x / W + noise(rng)),
                cv::saturate_cast<uint8_t>(90 + 60 * std::sin(y * 0.05) + noise(rng)),
                cv::saturate_cast<uint8_t>(200 - 150.0 * y / H + ((x / 16 + y / 16) % 2) * 30));
    luma.assign(W * H, 0);
    chroma.assign(W * H / 2, 0);
    auto Y = [&](const cv::Vec3b &p) { return 0.299 * p[2] + 0.587 * p[1] + 0.114 * p[0]; };
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            luma[y * W + x] = cv::saturate_cast<uint8_t>(Y(bgr.at<cv::Vec3b>(y, x)));
    for (int y = 0; y < H / 2; ++y)
        for (int x = 0; x < W / 2; ++x)
        {
            double u = 0, v = 0;
            for (int k = 0; k < 4; ++k)
            {
                const auto &p = bgr.at<cv::Vec3b>(2 * y + k / 2, 2 * x + k % 2);
                u += (p[0] - Y(p)) / 2.03211;
                v += (p[2] - Y(p)) / 1.13983;
            }
            chroma[2 * (y * W / 2 + x)] = cv::saturate_cast<uint8_t>(u / 4 + 128);
            chroma[2 * (y * W / 2 + x) + 1] = cv::saturate_cast<uint8_t>(v / 4 + 128);
        }
}

cv::Mat raw_bgr(const std::vector<uint8_t> &luma, const std::vector<uint8_t> &chroma)
{
    cv::Mat out(H, W, CV_8UC3);
    YCbCr420ToBgr{}(luma.data(), chroma.data(), W, H, out.data);
    return out;
}

double psnr(const cv::Mat &a, const cv::Mat &b)
{
    return cv::PSNR(a, b);
}

}

TEST(AuxJpeg, MessageKeepsTheTopicContractOfTheReencodeItReplaces)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = 12;
    stamp.nanosec = 345;
    const std::vector<uint8_t> jpeg{0xFF, 0xD8, 0xFF, 0xD9};
    const auto msg = make_aux_jpeg_message(jpeg, "carnegie_front/aux_rectified_optical_frame", stamp);
    EXPECT_EQ(msg->format, "bgr8; jpeg compressed bgr8");
    EXPECT_EQ(msg->header.frame_id, "carnegie_front/aux_rectified_optical_frame");
    EXPECT_EQ(msg->header.stamp, stamp);
    EXPECT_EQ(msg->data, jpeg);
    EXPECT_EQ(aux_jpeg_qos().get_rmw_qos_profile().reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(aux_jpeg_qos().get_rmw_qos_profile().history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(aux_jpeg_qos().get_rmw_qos_profile().depth, 10u);
    EXPECT_EQ(aux_jpeg_qos().get_rmw_qos_profile().durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(AuxJpeg, DemandFollowsEachTopicOnItsOwn)
{
    EXPECT_FALSE(aux_colour_demand(0, 0).bgr);
    EXPECT_FALSE(aux_colour_demand(0, 0).jpeg);
    EXPECT_TRUE(aux_colour_demand(1, 0).bgr);
    EXPECT_FALSE(aux_colour_demand(1, 0).jpeg);
    // Howler alone must not pay for the BGR conversion.
    EXPECT_FALSE(aux_colour_demand(0, 2).bgr);
    EXPECT_TRUE(aux_colour_demand(0, 2).jpeg);
}

TEST(AuxJpeg, ChromaIsRescaledFromTheCameraScaleToJfif)
{
    const auto &lut = jfif_chroma_tables();
    EXPECT_EQ(lut.cb[128], 128);
    EXPECT_EQ(lut.cr[128], 128);
    // JFIF B = Y + 1.772 (Cb - 128) has to equal the camera's Y + 2.03211 (U - 128).
    EXPECT_NEAR(1.772 * (lut.cb[178] - 128), 2.03211 * 50, 1.0);
    EXPECT_NEAR(1.402 * (lut.cr[78] - 128), 1.13983 * -50, 1.0);
    EXPECT_EQ(lut.cb[0], 0);
    EXPECT_EQ(lut.cb[255], 255);
}

TEST(AuxJpeg, I420FillCopiesLumaAndSplitsChroma)
{
    const std::vector<uint8_t> luma{1, 2, 3, 4, 5, 6, 7, 8};    // 4x2
    const std::vector<uint8_t> chroma{128, 128, 178, 78};       // 2x1 blocks
    std::vector<uint8_t> i420(4 * 2 * 3 / 2, 0);
    fill_i420(luma.data(), chroma.data(), 4, 2, i420.data());
    const auto &lut = jfif_chroma_tables();
    EXPECT_EQ(std::vector<uint8_t>(i420.begin(), i420.begin() + 8), luma);
    EXPECT_EQ(i420[8], 128);
    EXPECT_EQ(i420[9], lut.cb[178]);
    EXPECT_EQ(i420[10], 128);
    EXPECT_EQ(i420[11], lut.cr[78]);
}

class EachEncoder : public ::testing::TestWithParam<std::string>
{
protected:
    void SetUp() override
    {
        if (!JpegEncoder::available() || !JpegEncoder::has_encoder(GetParam()))
        {
            GTEST_SKIP() << GetParam() << " is not installed here";
        }
    }
};

TEST_P(EachEncoder, DecodesAtLeastAsCloseToTheRawImageAsTheReencodeDid)
{
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    const cv::Mat raw = raw_bgr(luma, chroma);

    // What pod_hl_image_jpeg published: the raw BGR through OpenCV at quality 75.
    std::vector<uint8_t> old_jpeg;
    ASSERT_TRUE(cv::imencode(".jpg", raw, old_jpeg, {cv::IMWRITE_JPEG_QUALITY, 75}));
    const double old_psnr = psnr(cv::imdecode(old_jpeg, cv::IMREAD_COLOR), raw);

    JpegEncoder encoder{"test", 75, {GetParam()}};
    const auto jpeg = encoder.encode(luma.data(), chroma.data(), W, H);
    ASSERT_NE(jpeg, nullptr);
    EXPECT_EQ(encoder.encoder_element(), GetParam());
    const cv::Mat decoded = cv::imdecode(*jpeg, cv::IMREAD_COLOR);
    ASSERT_EQ(decoded.cols, W);
    ASSERT_EQ(decoded.rows, H);
    EXPECT_GE(psnr(decoded, raw), old_psnr - 0.5) << "old " << old_psnr;
    EXPECT_GT(psnr(decoded, raw), 30.0);
    // No more than a fifth larger than the re-encode, so Howler's budget holds.
    EXPECT_LT(static_cast<double>(jpeg->size()), 1.2 * old_jpeg.size());
}

TEST_P(EachEncoder, ReturnsEveryFrameItIsHanded)
{
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    JpegEncoder encoder{"test", 75, {GetParam()}};
    for (int frame = 0; frame < 5; ++frame)
    {
        std::fill(luma.begin(), luma.begin() + W * 8, static_cast<uint8_t>(frame * 40));
        const auto jpeg = encoder.encode(luma.data(), chroma.data(), W, H);
        ASSERT_NE(jpeg, nullptr) << frame;
        const cv::Mat decoded = cv::imdecode(*jpeg, cv::IMREAD_GRAYSCALE);
        ASSERT_EQ(decoded.cols, W);
        // The frame handed in, not one queued before it.
        EXPECT_NEAR(decoded.at<uint8_t>(2, W / 2), frame * 40, 6) << frame;
    }
}

INSTANTIATE_TEST_SUITE_P(Encoders, EachEncoder, ::testing::Values("nvjpegenc", "jpegenc"));

TEST(AuxJpeg, QualityOutsideOneToHundredIsRefused)
{
    if (!JpegEncoder::available())
    {
        GTEST_SKIP() << "built without GStreamer";
    }
    EXPECT_THROW(JpegEncoder("test", 0), std::invalid_argument);
    EXPECT_THROW(JpegEncoder("test", 101), std::invalid_argument);
}

namespace {

constexpr int BIG_W = 1920;
constexpr int BIG_H = 1200;

// Never delivers a JPEG: a candidate that starts but gives nothing back.
const EncoderCandidate SILENT{"silent", "valve drop=true ! jpegenc"};

JpegEncoderOptions quick()
{
    JpegEncoderOptions options;
    options.first_frame_timeout = std::chrono::milliseconds{300};
    options.frame_timeout = std::chrono::milliseconds{100};
    options.timeout_after_miss = std::chrono::milliseconds{5};
    return options;
}

size_t resident_bytes()
{
    std::ifstream statm{"/proc/self/statm"};
    size_t pages = 0, resident = 0;
    statm >> pages >> resident;
    return resident * static_cast<size_t>(sysconf(_SC_PAGESIZE));
}

class Gst : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!JpegEncoder::available() || !JpegEncoder::has_encoder("jpegenc"))
        {
            GTEST_SKIP() << "jpegenc is not installed here";
        }
    }
};

}

TEST(AuxJpeg, OnlyThePushedFrameIsAccepted)
{
    EXPECT_TRUE(accept_sample(7, 7, 1));
    EXPECT_FALSE(accept_sample(6, 7, 2)) << "an older frame queued from a late encode";
    // An encoder that drops timestamps: the sample can only be ours when ours is the only one out.
    EXPECT_TRUE(accept_sample(std::nullopt, 7, 1));
    EXPECT_FALSE(accept_sample(std::nullopt, 7, 2));
}

TEST_F(Gst, NoInstalledCandidateFailsAtConstruction)
{
    EXPECT_THROW(JpegEncoder("test", 75, std::vector<std::string>{"no_such_jpeg_encoder"}), std::runtime_error);
}

TEST_F(Gst, AStalledEncoderKeepsMemoryFlat)
{
    // An encoder that has delivered, then falls far behind: every later push must be dropped
    // or bounded, never queued: a 1920x1200 frame is 3.5 MB.
    auto options = quick();
    options.first_frame_timeout = std::chrono::milliseconds{3000};
    options.misses_before_fallback = 1000;
    JpegEncoder encoder{"test", {{"stalled", "identity sleep-time=1000000 ! jpegenc"}}, options, nullptr};
    std::vector<uint8_t> luma(BIG_W * BIG_H, 90), chroma(BIG_W * BIG_H / 2, 128);
    ASSERT_NE(encoder.encode(luma.data(), chroma.data(), BIG_W, BIG_H), nullptr);
    const size_t before = resident_bytes();
    for (int i = 0; i < 60; ++i)
    {
        encoder.encode(luma.data(), chroma.data(), BIG_W, BIG_H);
    }
    EXPECT_LE(encoder.queued_frames(), options.max_queued_frames);
    EXPECT_LT(resident_bytes(), before + 30u * 1024 * 1024);
}

TEST_F(Gst, ASlowFirstFrameDoesNotGiveUpTheEncoder)
{
    // NVJPG or CUDA set-up can take a second on the first frame.
    std::vector<std::string> warnings;
    auto options = quick();
    options.first_frame_timeout = std::chrono::milliseconds{2000};
    JpegEncoder encoder{"test",
                        {{"slow", "identity sleep-time=600000 ! jpegenc"}, {"jpegenc", "jpegenc"}},
                        options, [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma(W * H, 90), chroma(W * H / 2, 128);
    EXPECT_NE(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
    EXPECT_EQ(encoder.encoder_element(), "slow");
    EXPECT_TRUE(warnings.empty());
}

TEST_F(Gst, FallsBackOnlyAfterSeveralMissesAndSaysSo)
{
    std::vector<std::string> warnings;
    auto options = quick();
    options.misses_before_fallback = 3;
    JpegEncoder encoder{"test", {SILENT, {"jpegenc", "jpegenc"}}, options,
                        [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma(W * H, 90), chroma(W * H / 2, 128);
    for (int miss = 0; miss < 2; ++miss)
    {
        EXPECT_EQ(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
        EXPECT_EQ(encoder.encoder_element(), "silent") << miss;
    }
    EXPECT_EQ(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
    EXPECT_NE(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
    EXPECT_EQ(encoder.encoder_element(), "jpegenc");
    ASSERT_EQ(warnings.size(), 1u);
    EXPECT_NE(warnings[0].find("silent"), std::string::npos);
    EXPECT_NE(warnings[0].find("jpegenc"), std::string::npos);
}

TEST_F(Gst, ANewFrameSizeRetriesEveryCandidate)
{
    std::vector<std::string> warnings;
    auto options = quick();
    options.misses_before_fallback = 1;
    JpegEncoder encoder{"test", {SILENT, {"jpegenc", "jpegenc"}}, options,
                        [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma(W * H, 90), chroma(W * H / 2, 128);
    encoder.encode(luma.data(), chroma.data(), W, H);
    ASSERT_NE(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
    ASSERT_EQ(warnings.size(), 1u);
    // The new mode starts from the preferred encoder again, which here fails again.
    encoder.encode(luma.data(), chroma.data(), W / 2, H / 2);
    EXPECT_NE(encoder.encode(luma.data(), chroma.data(), W / 2, H / 2), nullptr);
    EXPECT_EQ(warnings.size(), 2u);
}
