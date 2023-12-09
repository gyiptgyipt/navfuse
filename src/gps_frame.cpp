#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>



class Gps_transformer_node : public rclcpp::Node {
public:
    Gps_transformer_node(): Node("Gps_transformer_node") {

        double lat_origin, lon_origin, alt_origin;
        bool first_frame = true;    
              

        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                GeographicLib::Constants::WGS84_f());
        GeographicLib::LocalCartesian proj_geo;

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&Gps_transformer_node::gps_callback, this, std::placeholders::_1));


        visualization_msgs::msg::Marker line_strip;



    }

private:

     void gps_callback( const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {
       

     }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gps_transformer_node>());
    rclcpp::shutdown();
    return 0;
}