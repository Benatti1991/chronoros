#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
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
#include "ChronoRos.h"
///


using namespace std::chrono_literals;

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

struct RosVehicle {

    // =============================================================================

    RosVehicle() {
        GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
        SetChronoDataPath("/home/simonebenatti/codes/chronosensor/chrono/data/");
        SetDataPath("/home/simonebenatti/codes/chronosensor/chrono/data/vehicle/");
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
        bool contact_vis = false;

        // Simulation step sizes
        step_size = 3e-3;
        tire_step_size = 1e-3;

        // Simulation end time
        t_end = 200;

        // Time interval between two render frames
        render_step_size = 1.0 / 50;  // FPS = 50

        // Output directories
        const std::string out_dir = GetChronoOutputPath() + "HMMWV";
        const std::string pov_dir = out_dir + "/POVRAY";

        // Debug logging
        bool debug_output = false;
        debug_step_size = 1.0 / 1;  // FPS = 1

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
        debug_steps = (int) std::ceil(debug_step_size / step_size);

        if (contact_vis) {
            app->SetSymbolscale(1e-4);
            app->SetContactsDrawMode(IrrContactsDrawMode::CONTACT_FORCES);
        }

        ChRealtimeStepTimer realtime_timer;
        utils::ChRunningAverage RTF_filter(50);
        app->GetDevice()->run();
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
            app->Synchronize("", driver_inputs);

            // Advance simulation for one timestep for all modules
            driver->Advance(step_size);
            terrain->Advance(step_size);
            node_vehicle->Advance(step_size);
            app->Advance(step_size);

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
    double debug_step_size;
    std::shared_ptr<ChDriver> driver;
    std::shared_ptr<RigidTerrain> terrain;
    std::shared_ptr<WheeledVehicle> node_vehicle;
};

class SimNode : public rclcpp::Node {
  public:
    SimNode() : Node("chrono_sim") {
      this->declare_parameter<std::string>("my_parameter", "world");
      this->get_parameter<std::string>("my_parameter", parameter_string_);
      std::cout<< "\n" + parameter_string_ + "\n";
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      myvehicle = std::make_shared<RosVehicle>();
      actuation_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "vehicle_actuation", default_qos,
                std::bind(&SimNode::OnActuationMsg, this, std::placeholders::_1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("vehicle_pose", 10);
      timer_ = this->create_wall_timer(20ms, std::bind(&SimNode::timer_callback, this));
  }

  private:
    std::string parameter_string_;

private:
  void timer_callback() {
        myvehicle->advance_sim(.5);
        auto message = std::make_shared<geometry_msgs::msg::Twist>();

        message->linear.x = myvehicle->node_vehicle->GetChassisBody()->GetPos().x();
        message->linear.z =  myvehicle->node_vehicle->GetChassisBody()->GetPos().z();
        message->angular.z =  myvehicle->node_vehicle->GetChassisBody()->GetRot().Q_to_Euler123().z();

        publisher_->publish(*message);
  }

  void OnActuationMsg(const geometry_msgs::msg::Twist::SharedPtr _msg){
      myvehicle->driver->SetThrottle(_msg->linear.x);
      myvehicle->driver->SetSteering(_msg->angular.z);

    }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr actuation_sub_;
  std::shared_ptr<RosVehicle> myvehicle;
};

int main(int argc, char *argv[]) {


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
