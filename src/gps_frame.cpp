#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>


#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <swri_transform_util/transform_util.h>


LocalXyWgs84UtilPtr local_xy_util; 

class GPS_FRAME : public rclcpp::Node {
public:
    GPS_FRAME() : Node("GPS_FRAME") {

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/micro", 10, std::bind(&GPS_FRAME::gps_frame_pub, this, std::placeholders::_1));

        


    }

private:
    // void rec_data(const sensor_msgs::msg::Imu::SharedPtr msg) {
    
    // }

    void gps_frame_pub( const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {
       
        // Extract orientation from the IMU message
        // tf2::Quaternion tf_orientation;
        // tf2::fromMsg(msg->orientation, tf_orientation);

        // Create a transformation message
        // geometry_msgs::msg::TransformStamped transform_stamped;
        
        // transform_stamped.header.stamp = this->get_clock()->now();
        // transform_stamped.header.frame_id = "imu_link";  // Parent frame
        // transform_stamped.child_frame_id = "base_link";   // Child frame
        // transform_stamped.transform.translation.x = 0.0;  // Assuming no translation
        // transform_stamped.transform.translation.y = 0.0;
        // transform_stamped.transform.translation.z = 0.0;

        // tf2::Quaternion q;
        // q.setRPY(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        // transform_stamped.transform.rotation.x = q.x();
        // transform_stamped.transform.rotation.y = q.y();
        // transform_stamped.transform.rotation.z = q.z();
        // transform_stamped.transform.rotation.w = q.w();
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->angular_velocity.y);

        
        // transform_stamped.transform.rotation = tf2::toMsg(tf_orientation);

        // Broadcast the transformation
        // tf_broadcaster_->sendTransform(transform_stamped);

    }
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPS_FRAME>());
    rclcpp::shutdown();
    return 0;
}