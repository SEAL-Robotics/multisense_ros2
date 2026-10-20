/**
 * @file test_aux_jpeg_topic.cpp
 *
 * A subscriber to aux/image_rect_color/compressed is what made the driver encode,
 * and the driver links OpenCV's imgcodecs: this process does both, through the
 * driver's own worker, encoder (NVJPG where the host has it) and message.
 **/

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <multisense_ros/aux_jpeg.h>

using namespace multisense_ros;
using namespace std::chrono_literals;

namespace {

constexpr int W = 960;
constexpr int H = 600;

class Ros : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

}

TEST_F(Ros, ASubscriberToTheCompressedTopicGetsDecodableFrames)
{
    // libjpeg-turbo in use through OpenCV before the first subscriber turns the encoder on,
    // as the driver's image_transport plugins have it.
    std::vector<uint8_t> warm;
    ASSERT_TRUE(cv::imencode(".jpg", cv::Mat(16, 16, CV_8UC3, cv::Scalar(10, 20, 30)), warm));

    auto driver = std::make_shared<rclcpp::Node>("driver", "carnegie_front/aux");
    auto pub = driver->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_rect_color/compressed", aux_jpeg_qos());

    auto howler = std::make_shared<rclcpp::Node>("howler");
    std::promise<cv::Mat> first;
    bool done = false;
    auto sub = howler->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/carnegie_front/aux/image_rect_color/compressed", aux_jpeg_qos(),
        [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
        {
            if (done)
            {
                return;
            }
            done = true;
            EXPECT_EQ(msg->format, AUX_JPEG_FORMAT);
            EXPECT_EQ(msg->header.frame_id, "carnegie_front/aux_rectified_optical_frame");
            first.set_value(cv::imdecode(msg->data, cv::IMREAD_COLOR));
        });

    std::vector<std::string> warnings;
    AuxJpegEncoder encoder{75, AuxJpegEncoder::nvjpg(75), "NVJPG",
                           [&](const std::string &warning) { warnings.push_back(warning); }};
    AuxJpegWorker worker{
        [&](const AuxJpegJob &job)
        {
            return encoder.encode(job.luma->data(), job.chroma->data(), job.width, job.height);
        },
        [&](std::vector<uint8_t> jpeg, const builtin_interfaces::msg::Time &stamp)
        {
            pub->publish(make_aux_jpeg_message(std::move(jpeg), "carnegie_front/aux_rectified_optical_frame",
                                               stamp));
        },
        [](const std::string &error) { ADD_FAILURE() << error; }};

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(driver);
    executor.add_node(howler);
    auto decoded = first.get_future();
    const auto luma = std::make_shared<const std::vector<uint8_t>>(W * H, 120);
    const auto chroma = std::make_shared<const std::vector<uint8_t>>(W * H / 2, 128);
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (decoded.wait_for(0s) != std::future_status::ready && std::chrono::steady_clock::now() < deadline)
    {
        if (pub->get_subscription_count() > 0)
        {
            AuxJpegJob job;
            job.luma = luma;
            job.chroma = chroma;
            job.width = W;
            job.height = H;
            worker.post(job);
        }
        executor.spin_some(50ms);
    }
    ASSERT_EQ(decoded.wait_for(0s), std::future_status::ready);
    const cv::Mat image = decoded.get();
    ASSERT_EQ(image.cols, W);
    ASSERT_EQ(image.rows, H);
    EXPECT_NEAR(image.at<cv::Vec3b>(H / 2, W / 2)[1], 120, 4);

    worker.wait_idle();
    std::string error;
    if (NvJpegEncoder::create(75, W, H, &error))
    {
        EXPECT_EQ(encoder.encoder_name(), "NVJPG") << ::testing::PrintToString(warnings);
    }
    else
    {
        EXPECT_EQ(encoder.encoder_name(), "libjpeg-turbo");
    }
}
