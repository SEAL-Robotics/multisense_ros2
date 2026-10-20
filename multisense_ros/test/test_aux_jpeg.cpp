/**
 * @file test_aux_jpeg.cpp
 *
 * The driver's aux JPEG takes the place of pod_hl_image_jpeg's re-encode of the
 * raw BGR image: same topic contract, a picture at least as close to the raw
 * image as that re-encode was.
 **/

#include <gtest/gtest.h>

#include <dlfcn.h>
#include <link.h>

#include <algorithm>
#include <cmath>
#include <new>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <memory>
#include <random>
#include <set>
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

TEST(AuxJpeg, DecodesAtLeastAsCloseToTheRawImageAsTheReencodeDid)
{
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    const cv::Mat raw = raw_bgr(luma, chroma);

    // What pod_hl_image_jpeg published: the raw BGR through OpenCV at quality 75.
    std::vector<uint8_t> old_jpeg;
    ASSERT_TRUE(cv::imencode(".jpg", raw, old_jpeg, {cv::IMWRITE_JPEG_QUALITY, 75}));
    const double old_psnr = psnr(cv::imdecode(old_jpeg, cv::IMREAD_COLOR), raw);

    JpegEncoder encoder{75};
    const auto jpeg = encoder.encode(luma.data(), chroma.data(), W, H);
    ASSERT_NE(jpeg, nullptr);
    const cv::Mat decoded = cv::imdecode(*jpeg, cv::IMREAD_COLOR);
    ASSERT_EQ(decoded.cols, W);
    ASSERT_EQ(decoded.rows, H);
    EXPECT_GE(psnr(decoded, raw), old_psnr - 0.5) << "old " << old_psnr;
    EXPECT_GT(psnr(decoded, raw), 30.0);
    // No more than a fifth larger than the re-encode, so Howler's budget holds.
    EXPECT_LT(static_cast<double>(jpeg->size()), 1.2 * old_jpeg.size());
}

TEST(AuxJpeg, ReturnsEveryFrameItIsHanded)
{
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    JpegEncoder encoder{75};
    for (int frame = 0; frame < 5; ++frame)
    {
        std::fill(luma.begin(), luma.begin() + W * 8, static_cast<uint8_t>(frame * 40));
        const auto jpeg = encoder.encode(luma.data(), chroma.data(), W, H);
        ASSERT_NE(jpeg, nullptr) << frame;
        const cv::Mat decoded = cv::imdecode(*jpeg, cv::IMREAD_GRAYSCALE);
        ASSERT_EQ(decoded.cols, W);
        EXPECT_NEAR(decoded.at<uint8_t>(2, W / 2), frame * 40, 6) << frame;
    }
}

TEST(AuxJpeg, QualityOutsideOneToHundredIsRefused)
{
    EXPECT_THROW(JpegEncoder{0}, std::invalid_argument);
    EXPECT_THROW(JpegEncoder{101}, std::invalid_argument);
}

TEST(AuxJpeg, AnOddSizeIsRefusedNotEncodedWithShearedChroma)
{
    JpegEncoder encoder{75};
    std::vector<uint8_t> luma(5 * 4, 90), chroma(5 * 4, 128);
    EXPECT_THROW(encoder.encode(luma.data(), chroma.data(), 5, 4), std::invalid_argument);
    EXPECT_THROW(encoder.encode(luma.data(), chroma.data(), 4, 5), std::invalid_argument);
}

TEST(AuxJpeg, ASizeOffTheMcuGridEncodesTheRightPicture)
{
    // 644 x 202 leaves 4 columns and 10 rows of a 16 x 16 MCU over: libjpeg's raw input reads
    // the padding, so it must be there and must not bleed into the picture.
    constexpr int w = 644;
    constexpr int h = 202;
    std::vector<uint8_t> luma(w * h), chroma(w * h / 2);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            luma[y * w + x] = static_cast<uint8_t>(40 + (x * 150) / w);
    for (int y = 0; y < h / 2; ++y)
        for (int x = 0; x < w / 2; ++x)
        {
            chroma[2 * (y * w / 2 + x)] = static_cast<uint8_t>(x < w / 4 ? 90 : 170);
            chroma[2 * (y * w / 2 + x) + 1] = static_cast<uint8_t>(y < h / 4 ? 90 : 170);
        }
    cv::Mat raw(h, w, CV_8UC3);
    YCbCr420ToBgr{}(luma.data(), chroma.data(), w, h, raw.data);
    JpegEncoder encoder{90};
    const auto jpeg = encoder.encode(luma.data(), chroma.data(), w, h);
    const cv::Mat decoded = cv::imdecode(*jpeg, cv::IMREAD_COLOR);
    ASSERT_EQ(decoded.cols, w);
    ASSERT_EQ(decoded.rows, h);
    EXPECT_GT(cv::PSNR(decoded, raw), 30.0);
    const cv::Rect edge{w - 8, h - 8, 8, 8};
    EXPECT_GT(cv::PSNR(decoded(edge), raw(edge)), 30.0);
}

namespace {

std::string base_name(const char *path)
{
    const std::string name = path == nullptr ? "" : path;
    return name.substr(name.find_last_of('/') + 1);
}

// Every loaded object that exports libjpeg's entry point under its own name. Copies with
// renamed symbols (gdcm's gdcmjpeg8_*) cannot collide and are not counted.
std::set<std::string> libjpeg_providers()
{
    std::vector<std::string> loaded;
    dl_iterate_phdr(
        [](dl_phdr_info *info, size_t, void *data)
        {
            if (info->dlpi_name != nullptr && info->dlpi_name[0] != '\0')
            {
                static_cast<std::vector<std::string> *>(data)->push_back(info->dlpi_name);
            }
            return 0;
        },
        &loaded);
    std::set<std::string> out;
    for (const auto &path : loaded)
    {
        void *handle = dlopen(path.c_str(), RTLD_LAZY | RTLD_NOLOAD);
        if (handle == nullptr)
        {
            continue;
        }
        Dl_info info{};
        if (void *symbol = dlsym(handle, "jpeg_CreateCompress"); symbol != nullptr &&
            dladdr(symbol, &info) != 0)
        {
            out.insert(base_name(info.dli_fname));
        }
        dlclose(handle);
    }
    return out;
}

// The library any normally linked caller of libjpeg reaches: OpenCV, image_transport's
// plugins, and GStreamer's nvjpegenc.
std::string global_libjpeg()
{
    Dl_info info{};
    void *symbol = dlsym(RTLD_DEFAULT, "jpeg_CreateCompress");
    return symbol != nullptr && dladdr(symbol, &info) != 0 ? base_name(info.dli_fname) : "";
}

bool nvjpg_here()
{
    std::string error;
    return NvJpegEncoder::create(75, W, H, &error) != nullptr;
}

}

TEST(AuxJpeg, TheDriversEncoderLeavesOpenCvItsLibjpegInTheSameProcess)
{
    // The driver also holds OpenCV's imgcodecs. NVJPG's libjpeg 9 bound in its place kills
    // the driver on the first subscriber ("JPEG parameter struct mismatch: library thinks
    // size is 584, caller expects 1008"), so OpenCV runs first here, then the driver's
    // encoder, NVJPG where this host has it, then OpenCV again.
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    std::vector<uint8_t> opencv_jpeg;
    ASSERT_TRUE(cv::imencode(".jpg", raw_bgr(luma, chroma), opencv_jpeg));

    std::vector<std::string> warnings;
    AuxJpegEncoder encoder{75, AuxJpegEncoder::nvjpg(75), "NVJPG",
                           [&](const std::string &w) { warnings.push_back(w); }};
    for (int frame = 0; frame < 3; ++frame)
    {
        const auto jpeg = encoder.encode(luma.data(), chroma.data(), W, H);
        ASSERT_EQ(cv::imdecode(*jpeg, cv::IMREAD_COLOR).cols, W);
    }
    ASSERT_FALSE(cv::imdecode(opencv_jpeg, cv::IMREAD_COLOR).empty());
    std::vector<uint8_t> again;
    ASSERT_TRUE(cv::imencode(".jpg", raw_bgr(luma, chroma), again));

    EXPECT_EQ(global_libjpeg().rfind("libjpeg.so.8", 0), 0u) << global_libjpeg();
    for (const auto &provider : libjpeg_providers())
    {
        EXPECT_TRUE(provider.rfind("libjpeg.so.8", 0) == 0 || provider == "libnvmm_jpeg.so") << provider;
    }
    if (nvjpg_here())
    {
        // Where NVJPG exists it must be the one running, not quietly given up.
        EXPECT_EQ(encoder.encoder_name(), "NVJPG") << ::testing::PrintToString(warnings);
        EXPECT_TRUE(warnings.empty()) << ::testing::PrintToString(warnings);
    }
    else
    {
        EXPECT_EQ(encoder.encoder_name(), "libjpeg-turbo");
    }
}

TEST(AuxJpeg, NvjpgDecodesAsCloseToTheRawImageAsLibjpegTurbo)
{
    std::string error;
    auto nvjpg = NvJpegEncoder::create(75, W, H, &error);
    if (!nvjpg)
    {
        GTEST_SKIP() << "no NVJPG here: " << error;
    }
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    const cv::Mat raw = raw_bgr(luma, chroma);
    const auto turbo = JpegEncoder{75}.encode(luma.data(), chroma.data(), W, H);
    for (int frame = 0; frame < 3; ++frame)
    {
        const auto jpeg = nvjpg->encode(luma.data(), chroma.data(), W, H);
        ASSERT_NE(jpeg, nullptr);
        EXPECT_GT(psnr(cv::imdecode(*jpeg, cv::IMREAD_COLOR), raw),
                  psnr(cv::imdecode(*turbo, cv::IMREAD_COLOR), raw) - 1.0) << frame;
    }
    EXPECT_THROW(nvjpg->encode(luma.data(), chroma.data(), W / 2, H / 2), std::invalid_argument);
}

namespace {

// What a hardware encoder can get wrong while still running.
enum class Fault
{
    none,
    blank_surface,      // the CPU's writes never reached the engine
    swapped_chroma,     // Cb and Cr planes in each other's place
    wrong_pitch,        // rows written 8 bytes short of the surface's pitch
    not_jpeg_later,     // a right check frame, then bytes that are no JPEG
};

// A stand-in for NVJPG: libjpeg-turbo, with the fault applied to what it encodes.
AuxJpegEncoder::HardwareFactory fake_hardware(int *made, Fault fault = Fault::none)
{
    return [made, fault](int, int, std::string *) -> JpegFrameEncoder
    {
        ++*made;
        auto cpu = std::make_shared<JpegEncoder>(75);
        auto calls = std::make_shared<int>(0);
        return [cpu, calls, fault](const uint8_t *luma, const uint8_t *chroma, int w, int h)
            -> std::shared_ptr<std::vector<uint8_t>>
        {
            const size_t pixels = static_cast<size_t>(w) * h;
            std::vector<uint8_t> y(luma, luma + pixels);
            std::vector<uint8_t> c(chroma, chroma + pixels / 2);
            switch (fault)
            {
            case Fault::none:
                break;
            case Fault::blank_surface:
                std::fill(y.begin(), y.end(), 0);
                std::fill(c.begin(), c.end(), 128);
                break;
            case Fault::swapped_chroma:
                for (size_t i = 0; i + 1 < c.size(); i += 2)
                {
                    std::swap(c[i], c[i + 1]);
                }
                break;
            case Fault::wrong_pitch:
                for (size_t row = 0; row < static_cast<size_t>(h); ++row)
                {
                    std::copy(luma + row * (w - 8), luma + row * (w - 8) + w, y.begin() + row * w);
                }
                break;
            case Fault::not_jpeg_later:
                if (++*calls > 1)
                {
                    return std::make_shared<std::vector<uint8_t>>(std::vector<uint8_t>{0xFF, 0xD8, 0x00, 0x00});
                }
                break;
            }
            return cpu->encode(y.data(), c.data(), w, h);
        };
    };
}

}

TEST(AuxJpeg, AWorkingHardwareEncoderKeepsEveryFrame)
{
    int made = 0;
    std::vector<std::string> warnings;
    AuxJpegEncoder encoder{75, fake_hardware(&made), "fake",
                           [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    for (int frame = 0; frame < 3; ++frame)
    {
        ASSERT_NE(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
        EXPECT_EQ(encoder.encoder_name(), "fake");
    }
    EXPECT_EQ(made, 1);
    EXPECT_TRUE(warnings.empty());
    // A new frame size gets an encoder of its own.
    encoder.encode(luma.data(), chroma.data(), W / 2, H / 2);
    EXPECT_EQ(made, 2);
}

TEST(AuxJpeg, AHardwareEncoderThatCannotStartLeavesLibjpegTurboWithOneWarning)
{
    std::vector<std::string> warnings;
    int asked = 0;
    AuxJpegEncoder encoder{75,
                           [&](int, int, std::string *error) -> JpegFrameEncoder
                           {
                               ++asked;
                               *error = "no engine";
                               return nullptr;
                           },
                           "fake", [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    for (int frame = 0; frame < 3; ++frame)
    {
        ASSERT_NE(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
        EXPECT_EQ(encoder.encoder_name(), "libjpeg-turbo");
    }
    EXPECT_EQ(asked, 1);
    ASSERT_EQ(warnings.size(), 1u);
    EXPECT_NE(warnings[0].find("fake unavailable: no engine"), std::string::npos) << warnings[0];
}

TEST(AuxJpeg, AHardwareEncoderThatThrowsIsGivenUpAndItsFrameStillEncoded)
{
    std::vector<std::string> warnings;
    int calls = 0;
    AuxJpegEncoder encoder{75,
                           [&](int, int, std::string *) -> JpegFrameEncoder
                           {
                               return [&](const uint8_t *, const uint8_t *, int, int)
                                   -> std::shared_ptr<std::vector<uint8_t>>
                               {
                                   ++calls;
                                   throw std::runtime_error("engine wedged");
                               };
                           },
                           "fake", [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    const auto jpeg = encoder.encode(luma.data(), chroma.data(), W, H);
    ASSERT_NE(jpeg, nullptr);
    EXPECT_EQ(cv::imdecode(*jpeg, cv::IMREAD_COLOR).cols, W);
    encoder.encode(luma.data(), chroma.data(), W, H);
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(encoder.encoder_name(), "libjpeg-turbo");
    ASSERT_EQ(warnings.size(), 1u);
    EXPECT_NE(warnings[0].find("engine wedged"), std::string::npos) << warnings[0];
}

class WrongPicture : public ::testing::TestWithParam<Fault> {};

TEST_P(WrongPicture, NeverReachesASubscriberEvenWhenTheCameraStartsOnAFlatFrame)
{
    // A flat frame looks the same through every one of these faults, so the hardware is
    // judged on its check frame, not on what the camera sends first.
    int made = 0;
    std::vector<std::string> warnings;
    AuxJpegEncoder encoder{75, fake_hardware(&made, GetParam()), "fake",
                           [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> flat_luma(W * H, 120), flat_chroma(W * H / 2, 128);
    const auto jpeg = encoder.encode(flat_luma.data(), flat_chroma.data(), W, H);
    const auto turbo = JpegEncoder{75}.encode(flat_luma.data(), flat_chroma.data(), W, H);
    EXPECT_EQ(*jpeg, *turbo);
    EXPECT_EQ(encoder.encoder_name(), "libjpeg-turbo");
    ASSERT_EQ(warnings.size(), 1u);
    EXPECT_NE(warnings[0].find("check frame"), std::string::npos) << warnings[0];
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    encoder.encode(luma.data(), chroma.data(), W, H);
    EXPECT_EQ(made, 1);
    EXPECT_EQ(warnings.size(), 1u);
}

INSTANTIATE_TEST_SUITE_P(Faults, WrongPicture,
                         ::testing::Values(Fault::blank_surface, Fault::swapped_chroma, Fault::wrong_pitch));

TEST(AuxJpeg, AHardwareEncoderThatLaterReturnsNoJpegIsGivenUp)
{
    int made = 0;
    std::vector<std::string> warnings;
    AuxJpegEncoder encoder{75, fake_hardware(&made, Fault::not_jpeg_later), "fake",
                           [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    const auto jpeg = encoder.encode(luma.data(), chroma.data(), W, H);
    EXPECT_EQ(cv::imdecode(*jpeg, cv::IMREAD_COLOR).cols, W);
    EXPECT_EQ(encoder.encoder_name(), "libjpeg-turbo");
    ASSERT_EQ(warnings.size(), 1u);
    EXPECT_NE(warnings[0].find("returned no JPEG"), std::string::npos) << warnings[0];
}

TEST(AuxJpeg, AFactoryThatThrowsIsGivenUpForGood)
{
    int asked = 0;
    std::vector<std::string> warnings;
    AuxJpegEncoder encoder{75,
                           [&](int, int, std::string *) -> JpegFrameEncoder
                           {
                               ++asked;
                               throw std::bad_alloc();
                           },
                           "fake", [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    for (int frame = 0; frame < 3; ++frame)
    {
        ASSERT_NE(encoder.encode(luma.data(), chroma.data(), W, H), nullptr);
    }
    EXPECT_EQ(asked, 1);
    EXPECT_EQ(warnings.size(), 1u);
}

TEST(AuxJpeg, AnOddSizeDropsTheFrameButNotTheHardwareEncoder)
{
    int made = 0;
    std::vector<std::string> warnings;
    AuxJpegEncoder encoder{75, fake_hardware(&made), "fake",
                           [&](const std::string &w) { warnings.push_back(w); }};
    std::vector<uint8_t> luma(5 * 4, 90), chroma(5 * 4, 128);
    EXPECT_THROW(encoder.encode(luma.data(), chroma.data(), 5, 4), std::invalid_argument);
    std::vector<uint8_t> big_luma, big_chroma;
    camera_planes(big_luma, big_chroma);
    encoder.encode(big_luma.data(), big_chroma.data(), W, H);
    EXPECT_EQ(encoder.encoder_name(), "fake");
    EXPECT_TRUE(warnings.empty());
}

TEST(AuxJpeg, TheSameJpegDecodesFarAboveTheHardwareCheckAndGarbageAtZero)
{
    std::vector<uint8_t> luma, chroma;
    camera_planes(luma, chroma);
    const auto jpeg = JpegEncoder{75}.encode(luma.data(), chroma.data(), W, H);
    EXPECT_GT(decoded_psnr(*jpeg, *jpeg), 60.0);
    EXPECT_EQ(decoded_psnr(*jpeg, std::vector<uint8_t>{0xFF, 0xD8, 0x00}), 0.0);
}
