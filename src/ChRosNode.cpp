
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChAIDriver.h"
#include "chrono_models/vehicle/sedan/Sedan_AIDriver.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_sensor/Sensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"



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

#include "ChRosNode.h"
#include "ChRosUtils.h"

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


ChRosNode::ChRosNode() : Node("chrono_sim") {

    /// Declare and get command line arguments
    this->declare_parameter<std::string>("vehicle_json_path", "/fullvehiclejson.json");
    this->get_parameter<std::string>("vehicle_json_path", vehicle_file);
    this->declare_parameter<std::string>("lidar_json_path", "/Lidar.json");
    this->get_parameter<std::string>("lidar_json_path", lidar_file);
    this->declare_parameter<std::string>("terrain_json_path", "/RigidPlane.json");
    this->get_parameter<std::string>("terrain_json_path", terrain_file);
    this->declare_parameter<bool>("irr_render", true);
    this->get_parameter<bool>("irr_render", irr_render);
    //std::cout<< "\n" + parameter_string_ + "\n";
    ///

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    myvehicle = std::
    <RosVehicle>(lidar_file, vehicle_file, terrain_file, irr_render);
    actuation_sub_ = this->create_subscription<autoware_auto_msgs::msg::VehicleControlCommand>(
            "control_cmd", default_qos,

            std::bind(&ChRosNode::OnActuationMsg, this, std::placeholders::_1));
    VSC_sub_ = this->create_subscription<autoware_auto_msgs::msg::VehicleStateCommand>(
            "state_command", default_qos,
            std::bind(&ChRosNode::OnStateCommandMsg, this, std::placeholders::_1));
    pcl2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", 10);
    VSR_publisher_ = this->create_publisher<autoware_auto_msgs::msg::VehicleStateReport>("state_report", 10);
    VOdo_publisher_ = this->create_publisher<autoware_auto_msgs::msg::VehicleOdometry>("vehicle_odom", 10);
    NavOdo_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("gnss_odom", 10);
    VehKinState_publisher_ = this->create_publisher<autoware_auto_msgs::msg::VehicleKinematicState>("vehicle_kinematic_state", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&ChRosNode::timer_callback, this));
    start_time = this->get_clock()->now().seconds();
}

void ChRosNode::timer_callback() {
    // TODO SB This is very bad design. I should launch this in a std::async process and make it run in a separate thread
    myvehicle->advance_sim(.1);
    double mtime = this->get_clock()->now().seconds() - start_time;
    //myvehicle->driver->Synchronize(mtime, mtime, 0, 0);

    ////////////// Publish Lidar Points /////////////////////////////
    if(!lidar_file.empty()) {
        lidarscan = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::PointCloud2Modifier modifier(*lidarscan);

        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "i", 1, sensor_msgs::msg::PointField::FLOAT32);

        lidarscan->header.frame_id = "map";
        lidarscan->header.stamp = now();

        UserXYZIBufferPtr lidar_data = myvehicle->lidar_sensor->GetMostRecentBuffer<UserXYZIBufferPtr>();

        if (lidar_data->Buffer) {
            /// Get lidar data and pass them to a ROS2 pointcloud
            int npoints = lidar_data->Width * lidar_data->Height;
            //modifier.resize(npoints);
            //lidarscan->data.resize(npoints);
            ////lidarscan->header.frame_id=sOutTwoDLidar.id; //topic name to be published for lidar
            lidarscan->width = lidar_data->Width;
            lidarscan->height = lidar_data->Height;
            lidarscan->point_step = 4 * sizeof(float); //calculate the no of bytes in point cloud for each point
            lidarscan->row_step = lidarscan->width * lidarscan->point_step;
            modifier.resize(npoints);
            sensor_msgs::PointCloud2Iterator<float> iter_x(*lidarscan, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*lidarscan, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*lidarscan, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(*lidarscan, "i");
            for (int i = 0; i < npoints; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
                *iter_x = lidar_data->Buffer[i].x;
                *iter_y = lidar_data->Buffer[i].y;
                *iter_z = lidar_data->Buffer[i].z;
                *iter_i = lidar_data->Buffer[i].intensity;
            }

        }

        pcl2_publisher_->publish(*lidarscan);
    }


    ////////////// Publish State Report /////////////////////////////
    auto staterep = std::make_shared<autoware_auto_msgs::msg::VehicleStateReport>();
    staterep->fuel = 100;
    staterep->blinker = 0;
    staterep->headlight = 0;
    staterep->wiper = 0;
    staterep->mode = 2;
    staterep->hand_brake = false;
    staterep->horn = false;
    ChPowertrain::DriveMode dmode = myvehicle->node_vehicle->GetPowertrain()->GetDriveMode();
    switch(dmode) {
        case ChPowertrain::DriveMode::FORWARD: staterep->gear = 1;
            break;
        case ChPowertrain::DriveMode::NEUTRAL: staterep->gear = 5;
            break;
        case ChPowertrain::DriveMode::REVERSE: staterep->gear = 2;
            break;
        default:
            std::cout << "Error in returning gear\n";
            break;

    }
    VSR_publisher_->publish(*staterep);

    ////////////// Publish Vehicle Odometry /////////////////////////////
    auto vehodom_msg = std::make_shared<autoware_auto_msgs::msg::VehicleOdometry>();
    // Chrono uses meters, Autoware expects miles
    vehodom_msg->velocity_mps = myvehicle->node_vehicle->GetVehicleSpeed() / 1609,34;
    vehodom_msg->rear_wheel_angle_rad = 0;

    ChQuaternion<> spindle_rel_rotL = myvehicle->node_vehicle->GetChassisBody()->GetRot().GetConjugate() % myvehicle->node_vehicle->GetSpindleRot(0, LEFT);
    double spindle_relangleL = atan2(spindle_rel_rotL.GetYaxis().y(), spindle_rel_rotL.GetYaxis().x()) - CH_C_PI_2;

    ChQuaternion<> spindle_rel_rotR = myvehicle->node_vehicle->GetChassisBody()->GetRot().GetConjugate() % myvehicle->node_vehicle->GetSpindleRot(0, RIGHT);
    double spindle_relangleR = atan2(spindle_rel_rotR.GetYaxis().y(), spindle_rel_rotR.GetYaxis().x()) - CH_C_PI_2;
    double front_wheel_ang_rad = (spindle_relangleL + spindle_relangleR) / 2;
    vehodom_msg->front_wheel_angle_rad = front_wheel_ang_rad;

    VOdo_publisher_->publish(*vehodom_msg);


    ////////// Publish Navigation Odometry    ////////////////////
    auto navodom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    navodom_msg->header.stamp = this->get_clock()->now();
    ChVector<>      vehpos =  myvehicle->node_vehicle->GetChassisBody()->GetPos();
    ChQuaternion<> vehrot =  myvehicle->node_vehicle->GetChassisBody()->GetRot();
    navodom_msg->pose.pose.position.x = vehpos.x();
    navodom_msg->pose.pose.position.y = vehpos.y();
    navodom_msg->pose.pose.position.z = vehpos.z();
    navodom_msg->pose.pose.orientation.x = vehrot.e1();
    navodom_msg->pose.pose.orientation.y = vehrot.e2();
    navodom_msg->pose.pose.orientation.z = vehrot.e3();
    navodom_msg->pose.pose.orientation.w = vehrot.e0();

    ChVector<>   vehvel =  myvehicle->node_vehicle->GetChassisBody()->GetPos_dt();
    ChVector<> vehomega =  myvehicle->node_vehicle->GetChassisBody()->GetWvel_loc();
    navodom_msg->twist.twist.linear.x = vehvel.x();
    navodom_msg->twist.twist.linear.y = vehvel.y();
    navodom_msg->twist.twist.linear.z = vehvel.z();
    navodom_msg->twist.twist.angular.x = vehomega.x();
    navodom_msg->twist.twist.angular.y = vehomega.y();
    navodom_msg->twist.twist.angular.z = vehomega.z();

    navodom_msg->twist.covariance = { };
    navodom_msg->pose.covariance = { };

    navodom_msg->header.frame_id = "/odom";
    navodom_msg->child_frame_id = "/base_link";

    NavOdo_publisher_->publish(*navodom_msg);


    ////////////// Publish Vehicle Kinematic State /////////////////////////////
    auto vehkinstate_msg = std::make_shared<autoware_auto_msgs::msg::VehicleKinematicState>();
    vehkinstate_msg->header.stamp = this->get_clock()->now();
    const auto sim_odom_child_frame = "map";
    vehkinstate_msg->header.frame_id = sim_odom_child_frame;
    auto t_point = vehkinstate_msg->state;
    auto v_delta = vehkinstate_msg->delta;
    auto cog_frame = myvehicle->node_vehicle->GetChassisBody()->GetFrame_COG_to_abs();
    auto delta_frame = myvehicle->node_vehicle->GetChassisBody()->GetFrame_REF_to_COG();
    /// TODO
    //vehkinstate_msg.time_from_start.sec;
    //vehkinstate_msg.time_from_start.nanosec;
    t_point.x = cog_frame.GetPos().x();
    t_point.y = cog_frame.GetPos().y();
    //t_point.heading = cog_frame.GetRot().Q_to_Euler123().z();
    t_point.heading.real = cog_frame.GetA().Get_A_Xaxis().x();
    t_point.heading.imag = cog_frame.GetA().Get_A_Xaxis().y();
    t_point.longitudinal_velocity_mps = cog_frame.GetRot().RotateBack(cog_frame.GetPos_dt()).x() / 1609,34;
    t_point.lateral_velocity_mps = cog_frame.GetRot().RotateBack(cog_frame.GetPos_dt()).y() / 1609,34;
    t_point.acceleration_mps2 = cog_frame.GetPos_dtdt().Length() / 1609,34;
    t_point.heading_rate_rps = cog_frame.GetWvel_par().z();
    t_point.front_wheel_angle_rad = front_wheel_ang_rad;
    t_point.rear_wheel_angle_rad = 0;
    //RCLCPP_ERROR(this->get_logger(), "Frame is is");
    //RCLCPP_ERROR(this->get_logger(), vehkinstate_msg->header.frame_id);

    /*v_delta.translation.x = delta_frame.GetPos().x();
    v_delta.translation.y = delta_frame.GetPos().y();
    v_delta.translation.z = delta_frame.GetPos().z();
    v_delta.rotation.w = delta_frame.GetRot().e0();
    v_delta.rotation.x = delta_frame.GetRot().e1();
    v_delta.rotation.y = delta_frame.GetRot().e2();
    v_delta.rotation.z = delta_frame.GetRot().e3();
    v_delta.header.frame_id = "cog_to_ref";*/

    VehKinState_publisher_->publish(*vehkinstate_msg);
}

void ChRosNode::OnStateCommandMsg(const autoware_auto_msgs::msg::VehicleStateCommand::SharedPtr _msg){
        int gear = _msg->gear;
        switch(gear) {
            case 1:
                myvehicle->node_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
                break;
            case 2:
                myvehicle->node_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
                break;
            case 5:
                myvehicle->node_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::NEUTRAL);
                break;
            default:
                std::cout << "Gear not managed by Chrono\n";
                break;
        }
        return;

    }

}  // end namespace chronoros
}  // end namespace chrono
