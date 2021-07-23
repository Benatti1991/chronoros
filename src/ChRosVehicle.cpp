
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

#include "ChRosVehicle.h"
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


Full_JSON::Full_JSON(std::string json_path) {
    Document d;
    ReadFileJSON(json_path, d);
    if (d.IsNull()) {
        std::cout << "No JSON file found";
        return;
    }
    // Read top-level data
    assert(d.HasMember("Vehicle"));
    assert(d.HasMember("Powertrain"));
    assert(d.HasMember("Tire"));
    assert(d.HasMember("Init Location"));
    assert(d.HasMember("Init Rotation"));
    VehicleJSONstr = d["Vehicle"].GetString();
    TireJSONstr = d["Tire"].GetString();
    PowertrainJSONstr = d["Powertrain"].GetString();
    InitPos = ReadVectorJSON(d["Init Location"]);
    InitRot = ReadQuaternionJSON(d["Init Rotation"]);
    }


RosVehicle::RosVehicle(const std::string& lidar_json, const std::string& vehicle_path,
        const std::string& terrain_file, bool render, double timestep) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // Set the data directory because we are outside of the chrono demos folder
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "vehicle/");
    irr_render = render;

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
    step_size = timestep;
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

    // Create the vehicle, set parameters, and initialize
    auto vehicle_model = Full_JSON(GetChronoRosDataFile(vehicle_path));
    std::cout<<vehicle_model.VehicleJSON() << '\n';
    std::cout<<vehicle_model.PowertrainJSON() << '\n';
    std::cout<<vehicle_model.TireJSON() << '\n';
    node_vehicle = std::make_shared<WheeledVehicle>(vehicle::GetDataFile(vehicle_model.VehicleJSON()), ChContactMethod::NSC);
    node_vehicle->Initialize(ChCoordsys<>(vehicle_model.InitPos, vehicle_model.InitRot));
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
    terrain = std::make_shared<RigidTerrain>(node_vehicle->GetSystem(), GetChronoRosDataFile(terrain_file));


    // Create the vehicle Irrlicht interface
    if (irr_render) {
        app = std::make_shared<ChWheeledVehicleIrrApp>(node_vehicle.get(), L"HMMWV Demo");
        app->SetSkyBox();
        app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                              130);
        app->SetChaseCamera(trackPoint, 6.0, 0.5);
        app->SetTimestep(step_size);
        app->AssetBindAll();
        app->AssetUpdateAll();
    }
    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    driver = std::make_shared<Sedan_AIDriver>(*node_vehicle);
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
    if(!lidar_json.empty()) {
        lidar_sensor = std::dynamic_pointer_cast<ChLidarSensor>(
                Sensor::CreateFromJSON(GetChronoRosDataFile(lidar_json), node_vehicle->GetChassisBody(),
                                       ChFrame<>({-5, 0, .5}, Q_from_AngZ(0))));
        // add sensor to the manager
        sens_manager->AddSensor(lidar_sensor);
    }
    ChRealtimeStepTimer realtime_timer;
    utils::ChRunningAverage RTF_filter(50);
    if (irr_render) app->GetDevice()->run();
}



void RosVehicle::advance_sim(double deltaT) {
    double partial = 0;
    while(partial <= deltaT) {
        double time = node_vehicle->GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            return;

        // Render scene and output POV-Ray data
        if (irr_render && step_number % render_steps == 0) {
            app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app->DrawAll();
            app->EndScene();
            render_frame++;
        }

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        driver->Synchronize(time, target_acc, target_wheelang, 0);
        terrain->Synchronize(time);
        node_vehicle->Synchronize(time, driver_inputs, *terrain);


        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain->Advance(step_size);
        node_vehicle->Advance(step_size);
        if (irr_render) {
            app->Advance(step_size);
            app->Synchronize("", driver_inputs);
        }
        // Update the sensors
        sens_manager->Update();

        // Increment frame number
        step_number++;
        partial += step_size;
    }
}

}  // end namespace chronoros
}  // end namespace chrono
