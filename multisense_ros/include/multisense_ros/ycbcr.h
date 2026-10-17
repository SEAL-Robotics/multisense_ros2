/**
 * @file ycbcr.h
 *
 * The aux camera's YCbCr 4:2:0 frames to BGR8, byte for byte what
 * LibMultiSense's create_bgr_from_ycbcr420 returns, at a fraction of its cost.
 **/

#pragma once

#include <cstdint>
#include <vector>

namespace multisense_ros {

class YCbCr420ToBgr
{
public:
    ///
    /// @brief Builds the tables by running LibMultiSense's own conversion once over every
    ///        (luma, Cb, Cr) value, so they agree with it whatever the compiler did to its
    ///        float arithmetic. Takes a moment and about 16 MB; build it once.
    ///
    YCbCr420ToBgr();

    ///
    /// @brief Convert one frame. luma is width x height MONO8, chroma width/2 x height/2
    ///        interleaved Cb, Cr; out receives width x height x 3 bytes of BGR8.
    ///
    void operator()(const uint8_t *luma,
                    const uint8_t *chroma,
                    int width,
                    int height,
                    uint8_t *out) const;

private:
    std::vector<uint8_t> blue_;    // [cb][y]
    std::vector<uint8_t> green_;   // [cb][cr][y]
    std::vector<uint8_t> red_;     // [cr][y]
};

}
