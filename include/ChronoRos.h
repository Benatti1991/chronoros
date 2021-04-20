#ifndef CHRONO_ROS_H
#define CHRONO_ROS_H

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
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
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
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>

using namespace rapidjson;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono;
using namespace chrono::irrlicht;
using namespace std::chrono_literals;

namespace chrono {
namespace chronoros {

/// Function to get files in chronoros data directory
std::string GetChronoRosDataFile(const std::string& filename){
    return CHRONOROS_DATA_DIR + filename;
}

/// base class gof JSON vehicle model
class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string PowertrainJSON() const = 0;
    virtual double CameraDistance() const = 0;
};

class HMMWV_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual std::string VehicleJSON() const override {
        return "hmmwv/vehicle/HMMWV_Vehicle.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_4WD.json";
    }
    virtual std::string TireJSON() const override {
        return "hmmwv/tire/HMMWV_Pac02Tire.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "hmmwv/powertrain/HMMWV_ShaftsPowertrain.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
};

class Sedan_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Sedan"; }
    virtual std::string VehicleJSON() const override {
        return "sedan/vehicle/Sedan_Vehicle.json";
    }
    virtual std::string TireJSON() const override {
        return "sedan/tire/Sedan_TMeasyTire.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "sedan/powertrain/Sedan_SimpleMapPowertrain.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
};

class UAZ_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "UAZ"; }
    virtual std::string VehicleJSON() const override {
        return "uaz/vehicle/UAZBUS_SAEVehicle.json";
    }
    virtual std::string TireJSON() const override {
        return "uaz/tire/UAZBUS_TMeasyTireFront.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "uaz/powertrain/UAZBUS_SimpleMapPowertrain.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
};

class CityBus_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "CityBus"; }
    virtual std::string VehicleJSON() const override {
        return "citybus/vehicle/CityBus_Vehicle.json";
    }
    virtual std::string TireJSON() const override {
        return "citybus/tire/CityBus_TMeasyTire.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "citybus/powertrain/CityBus_SimpleMapPowertrain.json";
    }
    virtual double CameraDistance() const override { return 14.0; }
};

class MAN_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "MAN"; }
    virtual std::string VehicleJSON() const override {
        return "MAN_Kat1/vehicle/MAN_10t_Vehicle_8WD.json";
    }
    virtual std::string TireJSON() const override {
        return "MAN_Kat1/tire/MAN_5t_TMeasyTire.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "MAN_Kat1/powertrain/MAN_7t_SimpleCVTPowertrain.json";
    }
    virtual double CameraDistance() const override { return 12.0; }
};

/// Class for a wheeled vehicle entirely defined by a JSON file
class Full_JSON : public Vehicle_Model {
public:
    Full_JSON(std::string json_path) {
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
        VehicleJSONstr = d["Vehicle"].GetString();
        TireJSONstr = d["Tire"].GetString();
        PowertrainJSONstr = d["Powertrain"].GetString();
    }
    virtual std::string ModelName() const override { return "Custom"; }
    virtual std::string VehicleJSON() const override {
        return VehicleJSONstr;
    }
    virtual std::string TireJSON() const override {
        return TireJSONstr;
    }
    virtual std::string PowertrainJSON() const override {
        return PowertrainJSONstr;
    }
    virtual double CameraDistance() const override { return 6.0; }

    std::string VehicleJSONstr;
    std::string TireJSONstr;
    std::string PowertrainJSONstr;
};

ChVector<> DefLoc(0, 0, 1.6);
ChQuaternion<> DefRot(1, 0, 0, 0);
struct RosVehicle {

    // =============================================================================

    RosVehicle(ChVector<> initLoc=DefLoc, ChQuaternion<> initRot=DefRot, double timestep = 3e-3, const std::string& vehicle_path = "/fullvehiclejson.json", bool render = true) {
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

        // Create the HMMWV vehicle, set parameters, and initialize
        auto vehicle_model = Full_JSON(GetChronoRosDataFile(vehicle_path));
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
        if (irr_render) app->GetDevice()->run();
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
            if (irr_render && step_number % render_steps == 0) {
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

    double t_end;
    std::shared_ptr<ChWheeledVehicleIrrApp> app;
    bool irr_render;
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



}  // end namespace chronoros
}  // end namespace chrono

#endif
