// testing  with 



#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <swri_transform_util/transform_util.h>
#include <swri_transform_util/transform_manager.h>
#include <swri_transform_util/frames.h>
#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



// LocalXyWgs84UtilPtr local_xy_util; 
// swri_transform_util::LocalXyFromWgs84


class GpsTransformPublisher : public rclcpp::Node {
public:
    GpsTransformPublisher() : Node("GpsTransformPublisher") {

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&GpsTransformPublisher::HandleGps, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);


    }


private:

  void HandleGps(const std::shared_ptr<sensor_msgs::msg::NavSatFix> gps_fix)
  {
    tf2::Transform transform;
    std::shared_ptr<swri_transform_util::TransformManager> tf_manager_;


    // Get the orientation from the GPS track.
    // NOTE: This will be unreliable when the vehicle is stopped or moving at low
    //       speed.
    // double yaw = (90.0 - 0.0) * swri_math_util::_deg_2_rad;
    // yaw = swri_math_util::WrapRadians(yaw, swri_math_util::_pi);
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    transform.setRotation(orientation);

    // if (!tf_manager_)
    // {
    //   swri_transform_util::TransformManager::Initialize(20.0);
    // }

    // Get the position by converting lat/lon to LocalXY.
    swri_transform_util::Transform to_local_xy;
    std::string global_frame = "map";
    if (tf_manager_->GetTransform(global_frame, swri_transform_util::_wgs84_frame, tf2::TimePointZero, to_local_xy))
    {
      tf2::Vector3 position(gps_fix->longitude, gps_fix->latitude, 0.0);
      position = position;
      transform.setOrigin(position);

      geometry_msgs::msg::TransformStamped tf_stamped;
      tf_stamped.transform = tf2::toMsg(transform);
      tf_stamped.child_frame_id = "gps_frame";
      tf_stamped.header.frame_id = global_frame;
      tf_stamped.header.stamp = gps_fix->header.stamp;


      tf_broadcaster_->sendTransform(tf_stamped);
      RCLCPP_INFO(this->get_logger(), "pos is : '%f'", gps_fix->longitude);

    }
  }
  // namespace swri_transform_util

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsTransformPublisher>());
    rclcpp::shutdown();
    return 0;
}