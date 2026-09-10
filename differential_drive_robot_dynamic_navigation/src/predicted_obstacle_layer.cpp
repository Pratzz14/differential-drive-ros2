#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace differential_drive_robot_dynamic_navigation
{

// Mark directly into the master grid: a private, fixed-origin grid cannot be
// merged by cell index into a rolling local costmap.
class PredictedObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    predicted_.clear();
    cycle_.clear();
    // Retain rendered_ until updateBounds invalidates every old marked cell.
    current_ = true;
  }

  bool isClearable() override {return true;}

  void updateBounds(double, double, double, double * min_x, double * min_y,
                    double * max_x, double * max_y) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_received_).count() > timeout_) {
      predicted_.clear();
    }
    // Snapshot once per cycle so callbacks cannot mark outside these bounds.
    cycle_ = predicted_;
    const double padding = radius_ + layered_costmap_->getCostmap()->getResolution();
    for (const auto * points : {&rendered_, &cycle_}) {
      for (const auto & point : *points) {
        *min_x = std::min(*min_x, point.first - padding);
        *min_y = std::min(*min_y, point.second - padding);
        *max_x = std::max(*max_x, point.first + padding);
        *max_y = std::max(*max_y, point.second + padding);
      }
    }
    current_ = true;
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
                   int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double resolution = master_grid.getResolution();
    const int radius = std::max(0, static_cast<int>(std::ceil(radius_ / resolution)));
    if (enabled_) {
      for (const auto & point : cycle_) {
        // The centre can be just outside the rolling window while its disk
        // overlaps it. Do not discard the entire disk in that case.
        int mx, my;
        master_grid.worldToMapNoBounds(point.first, point.second, mx, my);
        for (int x = std::max({min_i, 0, mx - radius});
             x < std::min({max_i, static_cast<int>(master_grid.getSizeInCellsX()), mx + radius + 1}); ++x) {
          for (int y = std::max({min_j, 0, my - radius});
               y < std::min({max_j, static_cast<int>(master_grid.getSizeInCellsY()), my + radius + 1}); ++y) {
            if ((x - mx) * (x - mx) + (y - my) * (y - my) <= radius * radius) {
              master_grid.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
            }
          }
        }
      }
    }
    // LayeredCostmap resets the bounds before applying all layers, so old
    // predictions disappear without erasing obstacles owned by other layers.
    rendered_ = cycle_;
    current_ = true;
  }

protected:
  void onInitialize() override
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("topic", rclcpp::ParameterValue(std::string("/dynamic_obstacles/predicted_cloud_odom")));
    declareParameter("prediction_timeout", rclcpp::ParameterValue(0.5));
    declareParameter("mark_radius", rclcpp::ParameterValue(0.08));
    node->get_parameter(getFullName("enabled"), enabled_);
    node->get_parameter(getFullName("topic"), topic_);
    node->get_parameter(getFullName("prediction_timeout"), timeout_);
    node->get_parameter(getFullName("mark_radius"), radius_);
    if (timeout_ <= 0.0 || radius_ < 0.0) {
      throw std::invalid_argument("Prediction timeout must be positive and mark radius nonnegative");
    }
    subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS(),
      std::bind(&PredictedObstacleLayer::cloudCallback, this, std::placeholders::_1));
    current_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id != layered_costmap_->getGlobalFrameID()) {
      return;  // Never silently interpret map coordinates as odom coordinates.
    }
    std::vector<std::pair<double, double>> incoming;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      for (; x != x.end(); ++x, ++y) {
        if (std::isfinite(*x) && std::isfinite(*y)) {
          incoming.emplace_back(*x, *y);
        }
      }
    } catch (const std::runtime_error &) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    predicted_ = std::move(incoming);
    last_received_ = std::chrono::steady_clock::now();
  }

  std::string topic_;
  double timeout_{0.5};
  double radius_{0.08};
  std::vector<std::pair<double, double>> predicted_, rendered_, cycle_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point last_received_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace differential_drive_robot_dynamic_navigation

PLUGINLIB_EXPORT_CLASS(
  differential_drive_robot_dynamic_navigation::PredictedObstacleLayer,
  nav2_costmap_2d::Layer)
