#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

class SquareMarkerPublisher : public rclcpp::Node {
public:
    SquareMarkerPublisher() : Node("square_marker_publisher") {
        marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("square_marker", 10);

        // Set up the square marker
        line_striper();

    }

private:
    
    void line_striper(){
        auto square_marker_ = visualization_msgs::msg::Marker();  

        square_marker_.header.frame_id = "base_link"; // Set the frame ID according to your needs
        square_marker_.header.stamp = this->get_clock()->now();
        square_marker_.id = 0;
        square_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        square_marker_.action = visualization_msgs::msg::Marker::ADD;
        square_marker_.pose.orientation.w = 1.0;
        square_marker_.scale.x = 0.05; // Line width
        square_marker_.color.r = 0.0;
        square_marker_.color.g = 1.0;
        square_marker_.color.b = 0.0;
        square_marker_.color.a = 1.0;

        // Define square vertices
        geometry_msgs::msg::Point point;
        point.x = 1.0;
        point.y = 1.0;
        point.z = 0.0;
        square_marker_.points.push_back(point);

        point.x = 1.0;
        point.y = -1.0;
        point.z = 0.0;
        square_marker_.points.push_back(point);

        point.x = -1.0;
        point.y = -1.0;
        point.z = 0.0;
        square_marker_.points.push_back(point);

        point.x = -1.0;
        point.y = 1.0;
        point.z = 0.0;
        square_marker_.points.push_back(point);

        // Closing the loop
        point.x = 1.0;
        point.y = 1.0;
        point.z = 0.0;
        square_marker_.points.push_back(point);

        // Publish the square marker
        marker_publisher_->publish(square_marker_);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    visualization_msgs::msg::Marker square_marker_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SquareMarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
