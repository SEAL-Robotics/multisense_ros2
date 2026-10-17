/**
 * @file test_ycbcr.cpp
 *
 * The aux colour image must stay byte for byte what LibMultiSense's own
 * create_bgr_from_ycbcr420 produces, since the line detector thresholds it.
 **/

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include <MultiSense/MultiSenseUtilities.hh>

#include <multisense_ros/ycbcr.h>

namespace lms = multisense;
using multisense_ros::YCbCr420ToBgr;

namespace {

struct Planes
{
    int width;
    int height;
    std::vector<uint8_t> luma;
    std::vector<uint8_t> chroma;  // interleaved Cb, Cr at half width and height
};

Planes random_planes(int width, int height, uint32_t seed)
{
    std::mt19937 rng{seed};
    std::uniform_int_distribution<int> byte{0, 255};
    Planes p{width, height, std::vector<uint8_t>(width * height), std::vector<uint8_t>(width * height / 2)};
    for (auto &v : p.luma) v = static_cast<uint8_t>(byte(rng));
    for (auto &v : p.chroma) v = static_cast<uint8_t>(byte(rng));
    return p;
}

std::vector<uint8_t> library_bgr(const Planes &p)
{
    const lms::Image luma{std::make_shared<std::vector<uint8_t>>(p.luma), 0, p.luma.size(),
                          lms::Image::PixelFormat::MONO8, p.width, p.height, {}, {},
                          lms::DataSource::AUX_LUMA_RECTIFIED_RAW, {}};
    const lms::Image chroma{std::make_shared<std::vector<uint8_t>>(p.chroma), 0, p.chroma.size(),
                            lms::Image::PixelFormat::MONO16, p.width / 2, p.height / 2, {}, {},
                            lms::DataSource::AUX_CHROMA_RECTIFIED_RAW, {}};
    const auto bgr = lms::create_bgr_from_ycbcr420(luma, chroma, lms::DataSource::AUX_RECTIFIED_RAW);
    return std::vector<uint8_t>(bgr->raw_data->begin(), bgr->raw_data->end());
}

std::vector<uint8_t> fast_bgr(const YCbCr420ToBgr &convert, const Planes &p)
{
    std::vector<uint8_t> out(p.luma.size() * 3, 0xA5);
    convert(p.luma.data(), p.chroma.data(), p.width, p.height, out.data());
    return out;
}

const YCbCr420ToBgr &converter()
{
    static const YCbCr420ToBgr convert;
    return convert;
}

}

TEST(YCbCr420ToBgr, MatchesTheLibraryOnACameraSizedFrame)
{
    const auto planes = random_planes(1920, 1200, 7);
    EXPECT_EQ(fast_bgr(converter(), planes), library_bgr(planes));
}

TEST(YCbCr420ToBgr, MatchesTheLibraryOnSmallAndNonSquareFrames)
{
    for (const auto &[w, h] : std::vector<std::pair<int, int>>{{2, 2}, {6, 4}, {4, 10}, {960, 600}})
    {
        const auto planes = random_planes(w, h, static_cast<uint32_t>(w * 31 + h));
        EXPECT_EQ(fast_bgr(converter(), planes), library_bgr(planes)) << w << "x" << h;
    }
}

TEST(YCbCr420ToBgr, MatchesTheLibraryForEveryLumaAndChromaValue)
{
    // Every (y, cb, cr) once, shuffled across blocks so no input sits where the
    // tables were built from it.
    const int width = 4096;
    const int height = 4096;
    std::vector<uint32_t> order(1u << 22);
    for (uint32_t i = 0; i < order.size(); ++i) order[i] = i;
    std::shuffle(order.begin(), order.end(), std::mt19937{11});

    Planes p{width, height, std::vector<uint8_t>(width * height), std::vector<uint8_t>(width * height / 2)};
    const int blocks_per_row = width / 2;
    for (uint32_t b = 0; b < order.size(); ++b)
    {
        const uint32_t code = order[b];         // cb:8 cr:8 luma group:6
        const uint8_t cb = static_cast<uint8_t>(code >> 14);
        const uint8_t cr = static_cast<uint8_t>(code >> 6);
        const int group = static_cast<int>(code & 63);
        const int bx = static_cast<int>(b) % blocks_per_row;
        const int by = static_cast<int>(b) / blocks_per_row;
        p.chroma[2 * (by * blocks_per_row + bx)] = cb;
        p.chroma[2 * (by * blocks_per_row + bx) + 1] = cr;
        for (int k = 0; k < 4; ++k)
        {
            p.luma[(2 * by + k / 2) * width + 2 * bx + k % 2] = static_cast<uint8_t>(4 * group + k);
        }
    }
    EXPECT_EQ(fast_bgr(converter(), p), library_bgr(p));
}
