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
        // ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
        // ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
        // ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
        // ChQuaternion<> initRot(0, 0, 0, 1);

        // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
        VisualizationType chassis_vis_type = VisualizationType::MESH;
        VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
        VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
        VisualizationType wheel_vis_type = VisualizationType::MESH;
        VisualizationType tire_vis_type = VisualizationType::MESH;

        // Collision type for chassis (PRIMITIVES, MESH, or NONE)
        CollisionType chassis_collision_type = CollisionType::NONE;

        // Type of powertrain model (SHAFTS, SIMPLE)
        PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

        // Drive type (FWD, RWD, or AWD)
        DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

        // Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
        SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

        // Type of tire model (RIGID, RIGID_MESH, TMEASY, PACEJKA, LUGRE, FIALA, PAC89, PAC02)
        TireModelType tire_model = TireModelType::TMEASY;

        // Rigid terrain
        RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
        double terrainHeight = 0;      // terrain height (FLAT terrain only)
        double terrainLength = 100.0;  // size in X direction
        double terrainWidth = 100.0;   // size in Y direction

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
        my_hmmwv = std::make_shared<HMMWV_Full>();
        my_hmmwv->SetContactMethod(contact_method);
        my_hmmwv->SetChassisCollisionType(chassis_collision_type);
        my_hmmwv->SetChassisFixed(false);
        my_hmmwv->SetInitPosition(ChCoordsys<>(initLoc, initRot));
        my_hmmwv->SetPowertrainType(powertrain_model);
        my_hmmwv->SetDriveType(drive_type);
        my_hmmwv->SetSteeringType(steering_type);
        my_hmmwv->SetTireType(tire_model);
        my_hmmwv->SetTireStepSize(tire_step_size);
        my_hmmwv->Initialize();

        if (tire_model == TireModelType::RIGID_MESH)
            tire_vis_type = VisualizationType::MESH;

        my_hmmwv->SetChassisVisualizationType(chassis_vis_type);
        my_hmmwv->SetSuspensionVisualizationType(suspension_vis_type);
        my_hmmwv->SetSteeringVisualizationType(steering_vis_type);
        my_hmmwv->SetWheelVisualizationType(wheel_vis_type);
        my_hmmwv->SetTireVisualizationType(tire_vis_type);
        // Create the terrain
        terrain = std::make_shared<RigidTerrain>(my_hmmwv->GetSystem());

        MaterialInfo minfo;
        minfo.mu = 0.9f;
        minfo.cr = 0.01f;
        minfo.Y = 2e7f;
        auto patch_mat = minfo.CreateMaterial(contact_method);
        std::shared_ptr<RigidTerrain::Patch> patch;
        patch = terrain->AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength,
                                         terrainWidth);
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

        terrain->Initialize();

        // Create the vehicle Irrlicht interface
        app = std::make_shared<ChWheeledVehicleIrrApp>(&(my_hmmwv->GetVehicle()), L"HMMWV Demo");
        app->SetSkyBox();
        app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                             130);
        app->SetChaseCamera(trackPoint, 6.0, 0.5);
        app->SetTimestep(step_size);
        app->AssetBindAll();
        app->AssetUpdateAll();

        // -----------------
        // Initialize output
        // -----------------

        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
        }
        if (povray_output) {
            if (!filesystem::create_directory(filesystem::path(pov_dir))) {
                std::cout << "Error creating directory " << pov_dir << std::endl;
            }
            terrain->ExportMeshPovray(out_dir);
        }

        // Initialize output file for driver inputs
        std::string driver_file = out_dir + "/driver_inputs.txt";
        utils::CSV_writer driver_csv(" ");

        // Set up vehicle output
        my_hmmwv->GetVehicle().SetChassisOutput(true);
        my_hmmwv->GetVehicle().SetSuspensionOutput(0, true);
        my_hmmwv->GetVehicle().SetSteeringOutput(0, true);
        my_hmmwv->GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

        // Generate JSON information with available output channels
        my_hmmwv->GetVehicle().ExportComponentList(out_dir + "/component_list.json");

        // ------------------------
        // Create the driver system
        // ------------------------

        // Create the interactive driver system
        driver = std::make_shared<ChIrrGuiDriver>(*app);

        // Set the time response for steering and throttle keyboard inputs.
        double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;  // time to go from 0 to +1
        double braking_time = 0.3;   // time to go from 0 to +1
        driver->SetSteeringDelta(render_step_size / steering_time);
        driver->SetThrottleDelta(render_step_size / throttle_time);
        driver->SetBrakingDelta(render_step_size / braking_time);

        // If in playback mode, attach the data file to the driver system and
        // force it to playback the driver inputs.
        driver->Initialize();

        // ---------------
        // Simulation loop
        // ---------------

        my_hmmwv->GetVehicle().LogSubsystemTypes();

        if (debug_output) {
            GetLog() << "\n\n============ System Configuration ============\n";
            my_hmmwv->LogHardpointLocations();
        }

        // Number of simulation steps between miscellaneous events
        render_steps = (int) std::ceil(render_step_size / step_size);
        debug_steps = (int) std::ceil(debug_step_size / step_size);

        // Initialize simulation frame counters
        //int step_number = 0;
        //int render_frame = 0;

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
            double time = my_hmmwv->GetSystem()->GetChTime();

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

            // Debug logging
            //if (debug_output && step_number % debug_steps == 0) {
            //    GetLog() << "\n\n============ System Information ============\n";
            //    GetLog() << "Time = " << time << "\n\n";
            //    my_hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
            //
            //    auto marker_driver = my_hmmwv.GetChassis()->GetMarkers()[0]->GetAbsCoord().pos;
            //    auto marker_com = my_hmmwv.GetChassis()->GetMarkers()[1]->GetAbsCoord().pos;
            //    GetLog() << "Markers\n";
            //    std::cout << "  Driver loc:      " << marker_driver.x() << " " << marker_driver.y() << " "
            //              << marker_driver.z() << std::endl;
            //    std::cout << "  Chassis COM loc: " << marker_com.x() << " " << marker_com.y() << " " << marker_com.z()
            //              << std::endl;
            //}

            // Driver inputs
            ChDriver::Inputs driver_inputs = driver->GetInputs();

            // Update modules (process inputs from other modules)
            driver->Synchronize(time);
            terrain->Synchronize(time);
            my_hmmwv->Synchronize(time, driver_inputs, *terrain);
            app->Synchronize(driver->GetInputModeAsString(), driver_inputs);

            // Advance simulation for one timestep for all modules
            driver->Advance(step_size);
            terrain->Advance(step_size);
            my_hmmwv->Advance(step_size);
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
    std::shared_ptr<ChIrrGuiDriver> driver;
    std::shared_ptr<RigidTerrain> terrain;
    std::shared_ptr<HMMWV_Full> my_hmmwv;
};

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher") {
    myvehicle = std::make_shared<RosVehicle>();
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        20ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = 0.3;
    message->angular.z = 0.3;
    myvehicle->advance_sim(.5);
    publisher_->publish(*message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::shared_ptr<RosVehicle> myvehicle;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
