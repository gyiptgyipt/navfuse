#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"



class Gps_transformer_node : public rclcpp::Node {
public:
    Gps_transformer_node(): Node("Gps_transformer_node") {

 
        // LocalCartesian proj_geo;

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/micro", 10, std::bind(&Gps_transformer_node::gps_callback, this, std::placeholders::_1));


        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/gps_marker", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);



    }

private:


    void gps_callback( const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {

        double lat_origin, lon_origin, alt_origin;
        bool first_frame = true;

        GeographicLib::LocalCartesian local_geo;
        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                GeographicLib::Constants::WGS84_f());

        if (first_frame) {
            lat_origin = msg->latitude;
            lon_origin = msg->longitude;
            alt_origin = msg->altitude;
            local_geo = GeographicLib::LocalCartesian(lat_origin,
                                                    lon_origin,
                                                    alt_origin,
                                                    earth);
        first_frame = false;
            
        }


         double x, y, z;
  // Warning: if the measurement of altitude is wrong,
  // then the result of projection will be wrong.
        local_geo.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);


        std::cout << "Status: " << static_cast<int>(msg->status.status) << std::endl;
        std::cout << "(latitude, longitude, altitude)(deg, m) => (x, y, z)(m) = ("
            << std::to_string(msg->latitude) << ", "
            << std::to_string(msg->longitude) << ", "
            << std::to_string(msg->altitude) << ") => ("
            << x << ", "
            << y << ", "
            << z << ")" << std::endl;
    


        auto line_strip = visualization_msgs::msg::Marker();    

        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;

        line_strip.points.push_back(p);
        if (line_strip.points.size() > 10000)
        line_strip.points.clear();

        // std::cout<<p.x<<std::endl;


        
        line_strip.header.frame_id = "linestrip";
        line_strip.ns = "linestrip_ns";
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.header.stamp = this->now();
        line_strip.pose = geometry_msgs::msg::Pose();
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.scale.x = 1.0;
        line_strip.scale.y = 1.0;
        line_strip.scale.z = 1.0;


        line_strip.pose.position.x = 0;
        line_strip.pose.position.y = 0;
        line_strip.pose.position.z = 0;

        line_strip.color.a = 1.0;
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;

        marker_publisher_->publish(line_strip);


        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id ="map";  // Parent frame
        transform_stamped.child_frame_id = "linestrip";   // Child frame
        transform_stamped.transform.translation.x = 0;  
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.27; 

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);

     }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gps_transformer_node>());
    rclcpp::shutdown();
    return 0;
}