#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
//#include <sensor_msgs/msg/point_field__struct.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/impl/point_cloud2_iterator.hpp>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"

/// CHRONO HEADERS
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_sensor/Sensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"

#include "ChronoRos.h"
///


using namespace std::chrono_literals;

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;


struct RosVehicle {

    // =============================================================================

    RosVehicle() {
        GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
        // Set the data directory because we are outside of the chrono demos folder
        SetChronoDataPath(CHRONO_DATA_DIR);
        vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "vehicle/");
        //synchrono::SetDataPath(std::string(CHRONO_DATA_DIR) + "synchrono/");
        // Initial vehicle location and orientation
        ChVector<> initLoc(0, 0, 1.6);
        ChQuaternion<> initRot(1, 0, 0, 0);

        // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
        VisualizationType chassis_vis_type = VisualizationType::MESH;
        VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
        VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
        VisualizationType wheel_vis_type = VisualizationType::MESH;
        VisualizationType tire_vis_type = VisualizationType::MESH;

        // Collision type for chassis (PRIMITIVES, MESH, or NONE)
        CollisionType chassis_collision_type = CollisionType::NONE;

        // Drive type (FWD, RWD, or AWD)
        DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

        // Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
        SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

        // Type of tire model (RIGID, RIGID_MESH, TMEASY, PACEJKA, LUGRE, FIALA, PAC89, PAC02)
        TireModelType tire_model = TireModelType::TMEASY;

        // Point on chassis tracked by the camera
        ChVector<> trackPoint(0.0, 0.0, 1.75);

        // Contact method
        ChContactMethod contact_method = ChContactMethod::SMC;

        // Simulation step sizes
        step_size = 3e-3;
        tire_step_size = 1e-3;

        // Simulation end time
        t_end = 200;

        // Time interval between two render frames
        render_step_size = 1.0 / 50;  // FPS = 50

        //const std::string pov_dir = out_dir + "/POVRAY";


        // POV-Ray output
        bool povray_output = false;
        // --------------
        // Create systems
        // --------------

        // Create the HMMWV vehicle, set parameters, and initialize
        auto vehicle_model = Full_JSON("/home/simonebenatti/codes/VariousProjects/ros_miscellaneous/fullvehiclejson.json");
        std::cout<<vehicle_model.VehicleJSON() << '\n';
        std::cout<<vehicle_model.PowertrainJSON() << '\n';
        std::cout<<vehicle_model.TireJSON() << '\n';
        std::string rigidterrain_file("terrain/RigidPlane.json");
        node_vehicle = std::make_shared<WheeledVehicle>(vehicle::GetDataFile(vehicle_model.VehicleJSON()), ChContactMethod::NSC);
        node_vehicle->Initialize(ChCoordsys<>(initLoc, initRot));
        node_vehicle->GetChassis()->SetFixed(false);
        node_vehicle->SetChassisVisualizationType(VisualizationType::MESH);
        node_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        node_vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        node_vehicle->SetWheelVisualizationType(VisualizationType::MESH);

        auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile(vehicle_model.PowertrainJSON()));
        node_vehicle->InitializePowertrain(powertrain);

        // Create and initialize the tires
        for (auto& axle : node_vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(vehicle::GetDataFile(vehicle_model.TireJSON()));
                node_vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
            }
        }

        node_vehicle->SetChassisVisualizationType(chassis_vis_type);
        node_vehicle->SetSuspensionVisualizationType(suspension_vis_type);
        node_vehicle->SetSteeringVisualizationType(steering_vis_type);
        node_vehicle->SetWheelVisualizationType(wheel_vis_type);
        // Create the terrain
        terrain = std::make_shared<RigidTerrain>(node_vehicle->GetSystem(), vehicle::GetDataFile(rigidterrain_file));


        // Create the vehicle Irrlicht interface
        app = std::make_shared<ChWheeledVehicleIrrApp>(node_vehicle.get(), L"HMMWV Demo");
        app->SetSkyBox();
        app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                             130);
        app->SetChaseCamera(trackPoint, 6.0, 0.5);
        app->SetTimestep(step_size);
        app->AssetBindAll();
        app->AssetUpdateAll();

        // ------------------------
        // Create the driver system
        // ------------------------

        // Create the interactive driver system
        driver = std::make_shared<ChDriver>(*node_vehicle);
        driver->Initialize();

        // Set the time response for steering and throttle keyboard inputs.
        double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;  // time to go from 0 to +1
        double braking_time = 0.3;   // time to go from 0 to +1

        node_vehicle->LogSubsystemTypes();

        // Number of simulation steps between miscellaneous events
        render_steps = (int) std::ceil(render_step_size / step_size);

        // -----------------------------------------------
        // Sensors
        // -----------------------------------------------
        sens_manager = chrono_types::make_shared<ChSensorManager>(node_vehicle->GetSystem());
        lidar_sensor = std::dynamic_pointer_cast<ChLidarSensor>(Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Lidar.json"), node_vehicle->GetChassisBody(),
                                              ChFrame<>({-5, 0, .5}, Q_from_AngZ(0))));
        // add sensor to the manager
        sens_manager->AddSensor(lidar_sensor);

        ChRealtimeStepTimer realtime_timer;
        utils::ChRunningAverage RTF_filter(50);
        app->GetDevice()->run();
    }

    ~RosVehicle() {
        delete lidar_sensor.get();
        delete sens_manager.get();
        delete this;

    }

    void advance_sim(double deltaT) {
        double partial = 0;
        while(partial <= deltaT) {
            double time = node_vehicle->GetSystem()->GetChTime();

            // End simulation
            if (time >= t_end)
                return;

            // Render scene and output POV-Ray data
            if (step_number % render_steps == 0) {
                app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
                app->DrawAll();
                app->EndScene();
                render_frame++;
            }

            // Driver inputs
            ChDriver::Inputs driver_inputs = driver->GetInputs();

            // Update modules (process inputs from other modules)
            driver->Synchronize(time);
            terrain->Synchronize(time);
            node_vehicle->Synchronize(time, driver_inputs, *terrain);


            // Advance simulation for one timestep for all modules
            driver->Advance(step_size);
            terrain->Advance(step_size);
            node_vehicle->Advance(step_size);
            app->Advance(step_size);
            app->Synchronize("", driver_inputs);

            // Update the sensors
            sens_manager->Update();

            // Increment frame number
            step_number++;
            partial += step_size;
        }
    }

    double t_end;
    std::shared_ptr<ChWheeledVehicleIrrApp> app;
    int step_number = 0;
    int render_frame = 0;
    int render_steps;
    int debug_steps;
    double step_size;
    double tire_step_size;
    double render_step_size;
    std::shared_ptr<ChDriver> driver;
    std::shared_ptr<RigidTerrain> terrain;
    std::shared_ptr<WheeledVehicle> node_vehicle;
    std::shared_ptr<ChLidarSensor> lidar_sensor;
    std::shared_ptr<ChSensorManager> sens_manager;
};

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
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("vehicle_pose", 10);
      timer_ = this->create_wall_timer(20ms, std::bind(&SimNode::timer_callback, this));
  }


  private:
    void timer_callback() {
          myvehicle->advance_sim(.5);
          auto message = std::make_shared<geometry_msgs::msg::Twist>();

          message->linear.x = myvehicle->node_vehicle->GetChassisBody()->GetPos().x();
          message->linear.z =  myvehicle->node_vehicle->GetChassisBody()->GetPos().z();
          message->angular.z =  myvehicle->node_vehicle->GetChassisBody()->GetRot().Q_to_Euler123().z();

          lidarscan = std::make_shared<sensor_msgs::msg::PointCloud2>();
          sensor_msgs::PointCloud2Modifier modifier(*lidarscan);
          modifier.resize(1440000);
          modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "i", 1, sensor_msgs::msg::PointField::FLOAT32);

          lidarscan->header.frame_id = "map";
          lidarscan->header.stamp = now();

          UserXYZIBufferPtr lidar_data = myvehicle->lidar_sensor->GetMostRecentBuffer<UserXYZIBufferPtr>();

          if (lidar_data->Buffer) {
                //num_lidar_updates++;
                //std::cout << "Data recieved from lidar. Frame: "  << std::endl;

                /// Get lidar data and pass them to a ROS2 pointcloud
                //float* sensdata = reinterpret_cast<float*>( lidar_data->Buffer.get());
                int npoints = lidar_data->Width * lidar_data->Height;

                //modifier.resize(npoints);
                //lidarscan->data.resize(npoints);
                ////lidarscan->header.frame_id=sOutTwoDLidar.id; //topic name to be published for lidar
                lidarscan->width = lidar_data->Width;
                lidarscan->height = lidar_data->Height;
                lidarscan->point_step = 4*sizeof(float); //calculate the no of bytes in point cloud for each point
                lidarscan->row_step = lidarscan->width * lidarscan->point_step;
                ////std::cout<<__LINE__<<" Printing the 2d lidar data "<<std::endl;
                sensor_msgs::PointCloud2Iterator<float> iter_x(*lidarscan, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(*lidarscan, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(*lidarscan, "z");
                sensor_msgs::PointCloud2Iterator<float> iter_i(*lidarscan, "i");
                for(int i=0;i<npoints;++i,++iter_x, ++iter_y, ++iter_z, ++iter_i)
                {
                    //*iter_x = .5;//segmentation  fault here
                    *iter_x = lidar_data->Buffer[i].x;//segmentation  fault here
                    //std::cout<<"\n Writing the 2d lidar data, iteration  "<< i <<std::endl;
                    *iter_y = lidar_data->Buffer[i].y;
                    //std::cout<<__LINE__<<" Printing the 2d lidar data "<<std::endl;
                    *iter_z = lidar_data->Buffer[i].z;
                    //std::cout<<__LINE__<<" Printing the 2d lidar data "<<std::endl;
                    *iter_i = lidar_data->Buffer[i].intensity;
                    //std::cout<<__LINE__<<" Printing the 2d lidar data "<<std::endl;

                }

            }

          publisher_->publish(*message);
    }

    void OnActuationMsg(const geometry_msgs::msg::Twist::SharedPtr _msg){
        myvehicle->driver->SetThrottle(_msg->linear.x);
        myvehicle->driver->SetSteering(_msg->angular.z);

      }
  public:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
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
