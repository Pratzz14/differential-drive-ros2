#include <gtest/gtest.h>
#include "../src/predicted_obstacle_layer.cpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

class TestLayer : public differential_drive_robot_dynamic_navigation::PredictedObstacleLayer
{
public:
  using PredictedObstacleLayer::cloudCallback;
  void expire() {last_received_ = std::chrono::steady_clock::time_point{};}
  void disable() {enabled_ = false;}
};

class Predictions : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<nav2_util::LifecycleNode>("prediction_test");
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", true, false);
    map->resizeMap(40, 40, 0.1, -2.0, -2.0);
    layer = std::make_shared<TestLayer>();
    layer->initialize(map.get(), "prediction", nullptr, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    map->addPlugin(layer);
  }

  void cloud(float x = 1.0, float y = 0.0, std::string frame = "odom", bool empty = false)
  {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->header.frame_id = frame;
    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(empty ? 0 : 1);
    if (!empty) {
      sensor_msgs::PointCloud2Iterator<float> iter(*msg, "x");
      iter[0] = x; iter[1] = y; iter[2] = 0.0;
    }
    layer->cloudCallback(msg);
  }

  unsigned char cost(double x = 1.0, double y = 0.0)
  {
    unsigned int mx, my;
    EXPECT_TRUE(map->getCostmap()->worldToMap(x, y, mx, my));
    return map->getCostmap()->getCost(mx, my);
  }
  nav2_util::LifecycleNode::SharedPtr node;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  std::shared_ptr<TestLayer> layer;
};

TEST_F(Predictions, RollingOriginKeepsWorldCoordinates)
{
  cloud(); map->updateMap(0, 0, 0);
  ASSERT_EQ(cost(), LETHAL_OBSTACLE);
  map->updateMap(0.8, 0.5, 0);
  EXPECT_EQ(cost(), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(1.8, 0.5), FREE_SPACE);
}

TEST_F(Predictions, EmptyCloudClearsOldDisk)
{
  cloud(); map->updateMap(0, 0, 0);
  cloud(0, 0, "odom", true); map->updateMap(0, 0, 0);
  EXPECT_EQ(cost(), FREE_SPACE);
  EXPECT_EQ(cost(1.0, 0.1), FREE_SPACE);
}

TEST_F(Predictions, TimeoutClearsOldDisk)
{
  cloud(); map->updateMap(0, 0, 0);
  layer->expire(); map->updateMap(0, 0, 0);
  EXPECT_EQ(cost(), FREE_SPACE);
}

TEST_F(Predictions, ResetInvalidatesPreviouslyMarkedBounds)
{
  cloud(); map->updateMap(0, 0, 0);
  layer->reset(); map->updateMap(0, 0, 0);
  EXPECT_EQ(cost(), FREE_SPACE);
}

TEST_F(Predictions, RejectsWrongFrame)
{
  cloud(1, 0, "map"); map->updateMap(0, 0, 0);
  EXPECT_EQ(cost(), FREE_SPACE);
}

TEST_F(Predictions, DisableClearsPredictions)
{
  cloud(); map->updateMap(0, 0, 0);
  layer->disable(); map->updateMap(0, 0, 0);
  EXPECT_EQ(cost(), FREE_SPACE);
}

TEST_F(Predictions, CloudBetweenBoundsAndCostsWaitsForNextCycle)
{
  cloud();
  double min_x = 1e10, min_y = 1e10, max_x = -1e10, max_y = -1e10;
  layer->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
  cloud(-1, 0);
  layer->updateCosts(*map->getCostmap(), 0, 0, 40, 40);
  EXPECT_EQ(cost(), LETHAL_OBSTACLE);
  EXPECT_EQ(cost(-1, 0), FREE_SPACE);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(cost(), FREE_SPACE);
  EXPECT_EQ(cost(-1, 0), LETHAL_OBSTACLE);
}
