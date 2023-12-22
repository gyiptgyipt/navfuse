// my_line_strip_pkg_cpp/src/publisher_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

class LineStripPublisher : public rclcpp::Node {
public:
  LineStripPublisher() : Node("line_strip_publisher") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("line_strip_marker", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LineStripPublisher::publishLineStrip, this));
  }

private:
  void publishLineStrip() {
    auto markerArray = std::make_shared<visualization_msgs::msg::MarkerArray>();

    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;  // Line width
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Populate the line strip with points
    // for (int i = 0; i < 10; ++i) {
      auto point = geometry_msgs::msg::Point();
      point.x = 2;
      point.y = 2;
      point.z = 0;
      marker.points.push_back(point);
    // }

    markerArray->markers.push_back(marker);

    publisher_->publish(*markerArray);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineStripPublisher>());
  rclcpp::shutdown();
  return 0;
}