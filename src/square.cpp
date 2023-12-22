#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"


#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"



class SQUARE_NODE : public rclcpp::Node {
public:
    SQUARE_NODE(): Node("SQUARE_NODE") {

 
        // LocalCartesian proj_geo;

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/micro", 10, std::bind(&SQUARE_NODE::gps_callback, this, std::placeholders::_1));


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
        double X,Y,Z;
  // Warning: if the measurement of altitude is wrong,
  // then the result of projection will be wrong.
        local_geo.Forward(msg->latitude, msg->longitude,0.01, x, y, z);

        X = x * pow(10,11);
        Y = y * pow(10,11);
        Z = z;

        std::cout << "Status: " << static_cast<int>(msg->status.status) << std::endl;
        std::cout << "(latitude, longitude, altitude)(deg, m) => (x, y, z)(m) = ("
            << std::to_string(msg->latitude) << ", "
            << std::to_string(msg->longitude) << ", "
            << std::to_string(msg->altitude) << ") => ("
            << X << ", "
            << Y << ", "
            << Z << ")" << std::endl;


        int counter = 0;
        double parameter_size = 3.0;

        double newX,newY;

        if (counter % 4 == 0){
            newX = X + parameter_size;
            newY = Y;
        }

        if (counter % 4 == 1){
            newX = X;
            newY = Y + parameter_size;
        }


        if (counter % 4 == 2){
            newX = X - parameter_size;
            newY = Y;
        }


        if (counter % 4 == 3){
            newX = X ;
            newY = Y - parameter_size;
        }

        counter++;

    



// MARKER PART

        auto line_strip = visualization_msgs::msg::Marker();
        auto markerArray = std::make_shared<visualization_msgs::msg::MarkerArray>();

   
        // std::cout<<p.x<<std::endl;


        line_strip.header.frame_id = "base_link";
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;

        line_strip.scale.x = 0.5;  // Line width
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;


        auto p = geometry_msgs::msg::Point();
        p.x = newX;
        p.y = newY;
        p.z = 0;

        line_strip.points.push_back(p);
        // if (line_strip.points.size() > 10000)
        // line_strip.points.clear(); 


        marker_publisher_->publish(line_strip);
        

        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id ="map";  // Parent frame
        transform_stamped.child_frame_id = "base_link";   // Child frame
        transform_stamped.transform.translation.x = newX;  
        transform_stamped.transform.translation.y = newY;
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
    rclcpp::spin(std::make_shared<SQUARE_NODE>());
    rclcpp::shutdown();
    return 0;
}