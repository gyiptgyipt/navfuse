// testing  with 



#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>


#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <swri_transform_util/transform_util.h>
#include <swri_transform_util/transform_manager.h>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


// LocalXyWgs84UtilPtr local_xy_util; 
// swri_transform_util::LocalXyFromWgs84


class GPS_FRAME : public rclcpp::Node {
public:
    GPS_FRAME() : Node("GPS_FRAME") {

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/micro", 10, std::bind(&GPS_FRAME::gps_frame_pub, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);


    }

private:


    void gps_frame_pub( const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {


        swri_transform_util::Transform transform;
// Assuming msg.header.frame_id is "/wgs84" if the data is actually lat/lon
        if (tf_manager.GetTransform("map", msg.header.frame_id, msg.header.stamp, transform)
    {
                tf::Vector3 point(
                object_boxes.markers[0].pose.position.x, 
                object_boxes.markers[0].pose.position.y, 
                object_boxes.markers[0].pose.position.z);
                point = transform * point;
}



        // double x, y; // points for map
        // swri_transform_util::LocalXyFromWgs84(msg->latitude, // latitude of point
        //                                       msg->longitude, // longitude of point
        //                                       pose_origin.pose.position.y, // latitude for localXYorigin
        //                                       pose_origin.pose.position.x, // longitude localXYorigin
        //                                       x,
        //                                       y);

        //         geometry_msgs::msg::TransformStamped transform_stamped;
        
        // transform_stamped.header.stamp = this->get_clock()->now();
        // transform_stamped.header.frame_id = "map";  // Parent frame
        // transform_stamped.child_frame_id = "gps_link";   // Child frame
        // transform_stamped.transform.translation.x = msg->longitude;  // Assuming no translation
        // transform_stamped.transform.translation.y = msg->latitude;
        // transform_stamped.transform.translation.z = 0.0;

        // tf2::Quaternion q;
        // q.setRPY(0, 0, 0);
        // transform_stamped.transform.rotation.x = q.x();
        // transform_stamped.transform.rotation.y = q.y();
        // transform_stamped.transform.rotation.z = q.z();
        // transform_stamped.transform.rotation.w = q.w();
        // // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->angular_velocity.y);

        
        // // transform_stamped.transform.rotation = tf2::toMsg(tf_orientation);

        // // Broadcast the transformation
        // tf_broadcaster_->sendTransform(transform_stamped);

       

    }
    // rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPS_FRAME>());
    rclcpp::shutdown();
    return 0;
}