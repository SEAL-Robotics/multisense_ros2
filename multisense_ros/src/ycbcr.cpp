/**
 * @file ycbcr.cpp
 **/

#include <multisense_ros/ycbcr.h>

#include <memory>
#include <stdexcept>

#include <MultiSense/MultiSenseUtilities.hh>

namespace multisense_ros {

namespace {

namespace lms = multisense;

// One 2x2 block per (Cb, Cr, group of four luma values): 2^22 blocks.
constexpr int TABLE_SIDE = 4096;
constexpr int BLOCKS_PER_ROW = TABLE_SIDE / 2;

}

bool valid_ycbcr420(const lms::Image &luma, const lms::Image &chroma)
{
    if (luma.format != lms::Image::PixelFormat::MONO8 || chroma.format != lms::Image::PixelFormat::MONO16 ||
        !luma.raw_data || !chroma.raw_data || luma.width <= 0 || luma.height <= 0 ||
        luma.width % 2 != 0 || luma.height % 2 != 0 ||
        chroma.width != luma.width / 2 || chroma.height != luma.height / 2 ||
        luma.image_data_offset < 0 || chroma.image_data_offset < 0)
    {
        return false;
    }
    const size_t luma_bytes = static_cast<size_t>(luma.width) * luma.height;
    const size_t chroma_bytes = luma_bytes / 2;
    return luma.raw_data->size() >= static_cast<size_t>(luma.image_data_offset) + luma_bytes &&
           chroma.raw_data->size() >= static_cast<size_t>(chroma.image_data_offset) + chroma_bytes;
}

YCbCr420ToBgr::YCbCr420ToBgr():
    blue_(1u << 16),
    green_(1u << 24),
    red_(1u << 16)
{
    auto luma = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(TABLE_SIDE) * TABLE_SIDE);
    auto chroma = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(TABLE_SIDE) * TABLE_SIDE / 2);
    for (uint32_t block = 0; block < (1u << 22); ++block)
    {
        const size_t bx = block % BLOCKS_PER_ROW;
        const size_t by = block / BLOCKS_PER_ROW;
        (*chroma)[2 * (by * BLOCKS_PER_ROW + bx)] = static_cast<uint8_t>(block >> 14);
        (*chroma)[2 * (by * BLOCKS_PER_ROW + bx) + 1] = static_cast<uint8_t>(block >> 6);
        for (size_t k = 0; k < 4; ++k)
        {
            (*luma)[(2 * by + k / 2) * TABLE_SIDE + 2 * bx + k % 2] = static_cast<uint8_t>(4 * (block & 63) + k);
        }
    }

    const lms::Image luma_image{luma, 0, luma->size(), lms::Image::PixelFormat::MONO8,
                                TABLE_SIDE, TABLE_SIDE, {}, {}, lms::DataSource::AUX_LUMA_RAW, {}};
    const lms::Image chroma_image{chroma, 0, chroma->size(), lms::Image::PixelFormat::MONO16,
                                  TABLE_SIDE / 2, TABLE_SIDE / 2, {}, {}, lms::DataSource::AUX_CHROMA_RAW, {}};
    const auto bgr = lms::create_bgr_from_ycbcr420(luma_image, chroma_image, lms::DataSource::AUX_RAW);
    if (!bgr || bgr->raw_data->size() != luma->size() * 3)
    {
        throw std::runtime_error("LibMultiSense refused to build the YCbCr tables");
    }

    const uint8_t *pixels = bgr->raw_data->data();
    for (size_t i = 0; i < luma->size(); ++i)
    {
        const size_t bx = (i % TABLE_SIDE) / 2;
        const size_t by = (i / TABLE_SIDE) / 2;
        const size_t c = 2 * (by * BLOCKS_PER_ROW + bx);
        const uint32_t cb = (*chroma)[c];
        const uint32_t cr = (*chroma)[c + 1];
        const uint32_t y = (*luma)[i];
        blue_[(cb << 8) | y] = pixels[3 * i];
        green_[(cb << 16) | (cr << 8) | y] = pixels[3 * i + 1];
        red_[(cr << 8) | y] = pixels[3 * i + 2];
    }
}

void YCbCr420ToBgr::operator()(const uint8_t *luma,
                               const uint8_t *chroma,
                               int width,
                               int height,
                               uint8_t *out) const
{
    const size_t w = static_cast<size_t>(width);
    for (size_t row = 0; row < static_cast<size_t>(height); ++row)
    {
        const uint8_t *y_row = luma + row * w;
        const uint8_t *c_row = chroma + (row / 2) * w;   // w / 2 pairs of bytes
        uint8_t *o = out + row * w * 3;
        for (size_t col = 0; col < w; col += 2)
        {
            const uint32_t cb = c_row[col];
            const uint32_t cr = c_row[col + 1];
            const uint8_t *b = blue_.data() + (cb << 8);
            const uint8_t *g = green_.data() + ((cb << 16) | (cr << 8));
            const uint8_t *r = red_.data() + (cr << 8);
            const uint8_t y0 = y_row[col];
            const uint8_t y1 = y_row[col + 1];
            o[0] = b[y0]; o[1] = g[y0]; o[2] = r[y0];
            o[3] = b[y1]; o[4] = g[y1]; o[5] = r[y1];
            o += 6;
        }
    }
}

}
