#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

/// CHRONO HEADERS



#include "ChronoRos.h"
///


using namespace std::chrono_literals;
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::chronoros;

class SimNode : public rclcpp::Node {
public:
    SimNode() : Node("chrono_sim") {

        /// Declare and get command line arguments
        this->declare_parameter<std::string>("my_parameter", "world");
        this->get_parameter<std::string>("my_parameter", parameter_string_);
        std::cout<< "\n" + parameter_string_ + "\n";
        ///

        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        myvehicle = std::make_shared<RosVehicle>();
        actuation_sub_ = this->create_subscription<autoware_auto_msgs::msg::VehicleControlCommand>(
                "/chrono/vehicle_control_cmd", default_qos,
                std::bind(&SimNode::OnActuationMsg, this, std::placeholders::_1));
        VSC_sub_ = this->create_subscription<autoware_auto_msgs::msg::VehicleStateCommand>(
                "/chrono/vehicle_state_cmd", default_qos,
                std::bind(&SimNode::OnStateCommandMsg, this, std::placeholders::_1));
        pcl2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);
        VSR_publisher_ = this->create_publisher<autoware_auto_msgs::msg::VehicleStateReport>("/chrono/state_report", 10);
        VOdo_publisher_ = this->create_publisher<autoware_auto_msgs::msg::VehicleOdometry>("/chrono/vehicle_odom", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&SimNode::timer_callback, this));
    }


private:
    void timer_callback() {
        myvehicle->advance_sim(.5);

        lidarscan = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::PointCloud2Modifier modifier(*lidarscan);

        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "i", 1, sensor_msgs::msg::PointField::FLOAT32);

        lidarscan->header.frame_id = "/lidar_front";
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
            lidarscan->point_step = 4*sizeof(float); //calculate the no of bytes in point cloud for each point
            lidarscan->row_step = lidarscan->width * lidarscan->point_step;
            modifier.resize(npoints);
            sensor_msgs::PointCloud2Iterator<float> iter_x(*lidarscan, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*lidarscan, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*lidarscan, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(*lidarscan, "i");
            for(int i=0;i<npoints;++i,++iter_x, ++iter_y, ++iter_z, ++iter_i)
            {
                *iter_x = lidar_data->Buffer[i].x;
                *iter_y = lidar_data->Buffer[i].y;
                *iter_z = lidar_data->Buffer[i].z;
                *iter_i = lidar_data->Buffer[i].intensity;
            }

        }

        pcl2_publisher_->publish(*lidarscan);


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


        auto odomrep = std::make_shared<autoware_auto_msgs::msg::VehicleOdometry>();
        // Chrono uses meters, Autoware expects miles
        odomrep->velocity_mps = myvehicle->node_vehicle->GetVehicleSpeed() / 1609,34;
        odomrep->rear_wheel_angle_rad = 0;

        ChQuaternion<> spindle_rel_rotL = myvehicle->node_vehicle->GetChassisBody()->GetRot().GetConjugate() % myvehicle->node_vehicle->GetSpindleRot(0, LEFT);
        double spindle_relangleL = atan2(spindle_rel_rotL.GetYaxis().y(), spindle_rel_rotL.GetYaxis().x()) - CH_C_PI_2;

        ChQuaternion<> spindle_rel_rotR = myvehicle->node_vehicle->GetChassisBody()->GetRot().GetConjugate() % myvehicle->node_vehicle->GetSpindleRot(0, RIGHT);
        double spindle_relangleR = atan2(spindle_rel_rotR.GetYaxis().y(), spindle_rel_rotR.GetYaxis().x()) - CH_C_PI_2;

        odomrep->front_wheel_angle_rad = (spindle_relangleL + spindle_relangleR) / 2;

        VOdo_publisher_->publish(*odomrep);

    }

    void OnActuationMsg(const autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr _msg){
        // Chrono expects meters, Autoware uses miles
        myvehicle->target_acc = _msg->long_accel_mps2 * 1609,34;
        myvehicle->target_wheelang = _msg->front_wheel_angle_rad;

    }

    void OnStateCommandMsg(const autoware_auto_msgs::msg::VehicleStateCommand::SharedPtr _msg){
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
    std::string parameter_string_;
};


void shutnode(SimNode& node){
    delete node.myvehicle.get();
}

int main(int argc, char *argv[]) {

    //std::this_thread::sleep_for(std::chrono::milliseconds(30000));
    rclcpp::init(argc, argv);
    auto mnode = std::make_shared<SimNode>();
    rclcpp::spin(mnode);
    std::function<void()> f_shutnode = [mnode]() {  shutnode(*mnode);};
    rclcpp::on_shutdown( f_shutnode);
    rclcpp::shutdown();
    return 0;
}
