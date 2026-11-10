#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vln_agent/utils/base64.hpp>
#include <vln_agent/utils/json.hpp>
#include "vln_behavior_tree/event_node_base.hpp"
namespace vln_behavior_tree::test
{
using namespace std::chrono_literals;
class SurroundingImagesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("surrounding_images_test");
    blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    blackboard->set("robot_pose", Pose2D{"map", 1.2, 2.3, 0.5, ""});
    subscription = node->create_subscription<std_msgs::msg::String>(
      "/vln/robot_event", 10, [this](std_msgs::msg::String::ConstSharedPtr value) {
        events.push_back(Json::parse(value->data));
      });
    factory.registerFromPlugin(VLN_MOSAIC_PLUGIN_PATH);
  }
  BT::Tree tree(const std::string & attributes)
  {
    return factory.createTreeFromText(
      "<root><BehaviorTree ID='Main'><SubmitSurroundingImages pose='{robot_pose}' " +
      attributes + "/></BehaviorTree></root>", blackboard);
  }
  bool awaitEvent(std::size_t count = 1)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      if (events.size() >= count) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }
  ImageContent pictures()
  {
    ImageContent content;
    for (std::size_t i = 0; i < colors.size(); ++i) {
      cv::Mat source(180, 320, CV_8UC3, colors.at(i));
      cv::putText(
        source, "View " + std::to_string(i + 1), {90, 45},
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
      ImageContent::Image image;
      cv::imencode(".png", source, image.bytes);
      image.mime_type = "image/png";
      image.metadata = {{"custom_label", std::string("view_") + std::to_string(i)}};
      image.time_stamp = "2026-11-08T02:30:00.080Z";
      content.images.push_back(std::move(image));
    }
    return content;
  }
  Json payload(std::size_t index = 0)
  {
    Json accepted = Json::array();
    vln_agent::utils::appendInputs(accepted, events.at(index));
    return Json::parse(accepted.at(0).at("content").at(0).at("text").get<std::string>());
  }
  cv::Mat output(std::size_t index = 0)
  {
    const auto url = events.at(index).at("content").at(1).at("image_url").get<std::string>();
    EXPECT_EQ(url.substr(0, 23), "data:image/jpeg;base64,");
    return cv::imdecode(
      vln_agent::utils::decodeBase64(
        url.substr(
          url.find(
            ',') + 1)), cv::IMREAD_COLOR);
  }
  void expectLayout(const cv::Mat & image, bool clockwise, int size, int gap = 12)
  {
    ASSERT_EQ(image.cols, size * 3 + gap * 2);
    ASSERT_EQ(image.rows, size * 3 + gap * 2);
    const int stride = size + gap;
    const std::array<cv::Point, 8> ring{{{1, 0}, {2, 0}, {2, 1}, {2, 2},
      {1, 2}, {0, 2}, {0, 1}, {0, 0}}};
    for (std::size_t i = 0; i < colors.size(); ++i) {
      const auto cell = ring.at(clockwise ? i : (8 - i) % 8);
      const auto pixel =
        image.at<cv::Vec3b>(cell.y * stride + size / 2, cell.x * stride + size / 2);
      for (int channel = 0; channel < 3; ++channel) {
        EXPECT_NEAR(pixel[channel], colors.at(i)[channel], 8) << "view " << i;
      }
    }
  }
  std::set<std::filesystem::path> imageLogs()
  {
    const auto directory = std::filesystem::path(rclcpp::get_logging_directory().string()) /
      "vln_behavior_tree" / "surrounding_images";
    std::set<std::filesystem::path> files;
    if (std::filesystem::is_directory(directory)) {
      for (const auto & item : std::filesystem::directory_iterator(directory)) {
        if (item.path().extension() == ".jpg") {files.insert(item.path());}
      }
    }
    return files;
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::vector<Json> events;
  const std::array<cv::Scalar, 8> colors{{{40, 60, 210}, {40, 160, 210}, {40, 180, 60},
    {160, 180, 40}, {200, 100, 40}, {180, 40, 100}, {140, 50, 180}, {90, 130, 160}}};
};
TEST_F(SurroundingImagesTest, PublishesSingleJpegWithClockwiseViewsAndCenterYaw)
{
  const auto logs_before = imageLogs();
  blackboard->set("pictures", pictures());
  blackboard->set("robot_pose", Pose2D{"map", 1.2, 2.3, std::acos(-1) / 2, ""});
  auto bt = tree("image_content='{pictures}' direction='clockwise'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  ASSERT_EQ(events.at(0).at("content").size(), 2);
  const auto image = output();
  expectLayout(image, true, 256);
  const auto report = payload();
  EXPECT_NEAR(report.at("data").at("yaw").get<double>(), std::acos(-1) / 2, 1e-12);
  EXPECT_EQ(report.at("images").size(), 1);
  EXPECT_EQ(report.at("images").at(0).at("view_count"), 8);
  EXPECT_FALSE(report.at("images").at(0).contains("metadata"));
  EXPECT_EQ(report.at("pose").at("x"), 1.2);
  cv::Mat gray;
  cv::cvtColor(image(cv::Rect(268, 268, 256, 256)), gray, cv::COLOR_BGR2GRAY);
  EXPECT_GT(cv::countNonZero(gray > 100), 100);
  // 中央下方只保留背景，避免把 map 朝向箭头误认为外圈图片的方向。
  double maximum = 0;
  cv::minMaxLoc(gray(cv::Rect(8, 192, 240, 56)), nullptr, &maximum);
  EXPECT_LT(maximum, 60);
  EXPECT_EQ(imageLogs(), logs_before);
  const auto path = std::filesystem::path(VLN_MOSAIC_TEST_OUTPUT_DIR);
  std::filesystem::create_directories(path);
  ASSERT_TRUE(cv::imwrite((path / "surrounding_clockwise.jpg").string(), image));
}
TEST_F(SurroundingImagesTest, CounterclockwisePreservesSourceMetadataAndReadsFreshInputs)
{
  auto original = pictures();
  original.images.at(2).pose = Pose2D{"odom", 4, 5, 0.6, ""};
  blackboard->set("pictures", original);
  blackboard->set("robot_pose", Pose2D{"map", 1.2, 2.3, -0.5, ""});
  auto bt = tree("image_content='{pictures}' tile_size='96'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  expectLayout(output(), false, 96);
  const auto views = payload().at("data").at("views");
  ASSERT_EQ(views.size(), 8);
  EXPECT_EQ(views.at(1).at("row"), 0);
  EXPECT_EQ(views.at(1).at("column"), 0);
  EXPECT_EQ(views.at(2).at("custom_label"), "view_2");
  EXPECT_EQ(views.at(2).at("pose").at("frame_id"), "odom");
  EXPECT_EQ(views.at(2).at("time_stamp"), original.images.at(2).time_stamp);
  const auto retained = blackboard->get<ImageContent>("pictures");
  ASSERT_EQ(retained.images.size(), 8);
  EXPECT_EQ(retained.images.at(0).bytes, original.images.at(0).bytes);
  blackboard->set("robot_pose", Pose2D{"map", 1.2, 2.3, 0.25, ""});
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent(2));
  EXPECT_DOUBLE_EQ(payload(1).at("data").at("yaw").get<double>(), 0.25);
}
TEST_F(SurroundingImagesTest, ShrinksImagesWithoutDistortingAspectRatio)
{
  blackboard->set("pictures", pictures());
  auto bt = tree("image_content='{pictures}' tile_size='96'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  const auto image = output();
  // 源图 320x180 缩到 96x54，上下留边各 21 像素。
  EXPECT_LT(image.at<cv::Vec3b>(5, 108 + 48)[2], 30);
  EXPECT_NEAR(image.at<cv::Vec3b>(48, 108 + 48)[2], colors.at(0)[2], 8);
  EXPECT_LT(image.at<cv::Vec3b>(90, 108 + 48)[2], 30);
}
TEST_F(SurroundingImagesTest, SeparatesAdjacentTilesWithConfigurableGaps)
{
  blackboard->set("pictures", pictures());
  auto bt = tree("image_content='{pictures}' tile_size='96' gap_size='20'");
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  const auto image = output();
  expectLayout(image, false, 96, 20);
  for (const auto & point : {cv::Point(106, 48), cv::Point(164, 106)}) {
    const auto pixel = image.at<cv::Vec3b>(point);
    for (int channel = 0; channel < 3; ++channel) {
      EXPECT_NEAR(pixel[channel], 180, 5);
    }
  }
  EXPECT_EQ(payload().at("images").at(0).at("gap_size"), 20);
  EXPECT_EQ(payload().at("images").at(0).at("tile_size"), 96);
  auto no_gap = tree("image_content='{pictures}' tile_size='96' gap_size='0'");
  ASSERT_EQ(no_gap.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent(2));
  expectLayout(output(1), false, 96, 0);
}

TEST_F(SurroundingImagesTest, SavesExactSubmittedJpegToDistinctLogFilesWhenEnabled)
{
  blackboard->set("pictures", pictures());
  auto bt = tree("image_content='{pictures}' save_to_log='true'");
  auto previous = imageLogs();
  for (std::size_t index = 0; index < 2; ++index) {
    ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(awaitEvent(index + 1));
    const auto current = imageLogs();
    std::vector<std::filesystem::path> created;
    for (const auto & path : current) {
      if (!previous.count(path)) {created.push_back(path);}
    }
    ASSERT_EQ(created.size(), 1);
    std::ifstream file(created.front(), std::ios::binary);
    const std::vector<unsigned char> saved(
      (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const auto url = events.at(index).at("content").at(1).at("image_url").get<std::string>();
    EXPECT_EQ(saved, vln_agent::utils::decodeBase64(url.substr(url.find(',') + 1)));
    previous = current;
  }
}

TEST_F(SurroundingImagesTest, LogWriteFailureStillPublishesEvent)
{
  blackboard->set("pictures", pictures());
  auto bt = tree("image_content='{pictures}' save_to_log='true'");
  const auto blocker = std::filesystem::path(VLN_MOSAIC_TEST_OUTPUT_DIR) / "log_directory_blocker";
  std::filesystem::create_directories(blocker.parent_path());
  std::ofstream(blocker).put('x');
  const auto environment = std::getenv("ROS_LOG_DIR");
  const bool was_set = environment != nullptr;
  const std::string previous = was_set ? environment : "";
  ASSERT_EQ(setenv("ROS_LOG_DIR", blocker.c_str(), 1), 0);
  const auto status = bt.tickRoot();
  if (was_set) {setenv("ROS_LOG_DIR", previous.c_str(), 1);} else {unsetenv("ROS_LOG_DIR");}
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  EXPECT_EQ(payload().at("images").size(), 1);
}

TEST_F(SurroundingImagesTest, InvalidImagesAndParametersFailBeforePublishingAndCanRecover)
{
  auto value = pictures();
  value.images.pop_back();
  blackboard->set("pictures", value);
  auto bt = tree("image_content='{pictures}'");
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
  value = pictures();
  value.images.push_back(value.images.at(0));
  blackboard->set("pictures", value);
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
  value = pictures();
  value.images.at(3).bytes = {1, 2, 3};
  blackboard->set("pictures", value);
  EXPECT_EQ(bt.tickRoot(), BT::NodeStatus::FAILURE);
  blackboard->set("pictures", pictures());
  auto direction = tree("image_content='{pictures}' direction='bad'");
  EXPECT_EQ(direction.tickRoot(), BT::NodeStatus::FAILURE);
  auto size = tree("image_content='{pictures}' tile_size='0'");
  EXPECT_EQ(size.tickRoot(), BT::NodeStatus::FAILURE);
  auto gap = tree("image_content='{pictures}' gap_size='-1'");
  EXPECT_EQ(gap.tickRoot(), BT::NodeStatus::FAILURE);
  auto overflow = tree("image_content='{pictures}' gap_size='2147483647'");
  EXPECT_EQ(overflow.tickRoot(), BT::NodeStatus::FAILURE);
  blackboard->set("robot_pose", Pose2D{"map", 0, 0, std::numeric_limits<double>::quiet_NaN(), ""});
  auto yaw = tree("image_content='{pictures}'");
  EXPECT_EQ(yaw.tickRoot(), BT::NodeStatus::FAILURE);
  blackboard->set("robot_pose", Pose2D{"map", 1.2, 2.3, 0.5, ""});
  rclcpp::spin_some(node);
  EXPECT_TRUE(events.empty());
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
}

TEST_F(SurroundingImagesTest, UsesBaseTfPoseForCenterYawAndEventWithoutYawInput)
{
  blackboard->set("pictures", pictures());
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  blackboard->set("tf_buffer", buffer);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 9;
  transform.transform.rotation.z = std::sin(0.8 / 2);
  transform.transform.rotation.w = std::cos(0.8 / 2);
  ASSERT_TRUE(buffer->setTransform(transform, "test", true));
  auto bt = factory.createTreeFromText(
    "<root><BehaviorTree ID='Main'><SubmitSurroundingImages image_content='{pictures}'/>"
    "</BehaviorTree></root>", blackboard);
  EXPECT_FALSE(factory.manifests().at("SubmitSurroundingImages").ports.count("yaw"));
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  EXPECT_NEAR(payload().at("data").at("yaw").get<double>(), 0.8, 1e-12);
  EXPECT_EQ(payload().at("pose").at("yaw"), payload().at("data").at("yaw"));
  EXPECT_EQ(payload().at("pose").at("x"), 9);
}

TEST_F(SurroundingImagesTest, StillReportsImagesWhenTfPoseIsUnavailable)
{
  blackboard->set("pictures", pictures());
  blackboard->set("tf_buffer", std::make_shared<tf2_ros::Buffer>(node->get_clock()));
  auto bt = factory.createTreeFromText(
    "<root><BehaviorTree ID='Main'><SubmitSurroundingImages image_content='{pictures}'/>"
    "</BehaviorTree></root>", blackboard);
  ASSERT_EQ(bt.tickRoot(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(awaitEvent());
  EXPECT_FALSE(payload().contains("pose"));
  EXPECT_FALSE(payload().at("data").contains("yaw"));
  EXPECT_EQ(events.at(0).at("content").size(), 2);
}

}  // namespace vln_behavior_tree::test
