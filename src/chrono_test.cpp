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
        actuation_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "vehicle_actuation", default_qos,
                std::bind(&SimNode::OnActuationMsg, this, std::placeholders::_1));
        pcl2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_front/points_raw", 10);
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
            case ChPowertrain::DriveMode::FORWARD: staterep->gear = 0;
                    break;
            case ChPowertrain::DriveMode::NEUTRAL: staterep->gear = 4;
                    break;
            case ChPowertrain::DriveMode::REVERSE: staterep->gear = 1;
                    break;
            default:
                     std::cout << "Error in returning gear\n";
                     break;

}
        staterep->wiper = 0;

    }

    void OnActuationMsg(const geometry_msgs::msg::Twist::SharedPtr _msg){
        myvehicle->driver->SetThrottle(_msg->linear.x);
        myvehicle->driver->SetSteering(_msg->angular.z);

    }
public:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_publisher_;
    rclcpp::Publisher<autoware_auto_msgs::msg::VehicleStateReport>::SharedPtr VSR_publisher_;
    rclcpp::Publisher<autoware_auto_msgs::msg::VehicleOdometry>::SharedPtr VOdo_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr actuation_sub_;
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
