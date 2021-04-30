#ifndef CH_ROS_NODE_H
#define CH_ROS_NODE_H

#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/impl/point_cloud2_iterator.hpp>
#include "rclcpp/rclcpp.hpp"
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>

#include "ChRosVehicle.h"



using namespace rapidjson;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono;
using namespace chrono::irrlicht;
using namespace std::chrono_literals;
// TODO: get rid of this as soon as aidriver is available for json
using namespace chrono::vehicle::sedan;

namespace chrono {
namespace chronoros {

class CHROS_API ChRosNode : public rclcpp::Node {
  public:
    ChRosNode();

private:
    void timer_callback();

    void OnActuationMsg(const autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr _msg){
        // Chrono expects meters, Autoware uses miles
        myvehicle->target_acc = _msg->long_accel_mps2 * 1609,34;
        myvehicle->target_wheelang = _msg->front_wheel_angle_rad;

    }

    void OnStateCommandMsg(const autoware_auto_msgs::msg::VehicleStateCommand::SharedPtr _msg);
public:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_publisher_;
    rclcpp::Publisher<autoware_auto_msgs::msg::VehicleStateReport>::SharedPtr VSR_publisher_;
    rclcpp::Publisher<autoware_auto_msgs::msg::VehicleOdometry>::SharedPtr VOdo_publisher_;
    // nav msg odom
    // vehicle kinematic state
    rclcpp::Subscription<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr actuation_sub_;
    rclcpp::Subscription<autoware_auto_msgs::msg::VehicleStateCommand>::SharedPtr VSC_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr lidarscan;
    std::shared_ptr<RosVehicle> myvehicle;
    /// Command line arguments
    std::string vehicle_file;
    std::string lidar_file;
    std::string terrain_file;
    bool irr_render;
};

}  // end namespace chronoros
}  // end namespace chrono

#endif
