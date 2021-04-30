#ifndef CH_ROS_VEHICLE_H
#define CH_ROS_VEHICLE_H

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

#include "ChRosApi.h"
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

/// base class gof JSON vehicle model
class CHROS_API Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string PowertrainJSON() const = 0;
    virtual double CameraDistance() const = 0;
};

/// Class for a wheeled vehicle entirely defined by a JSON file
class CHROS_API Full_JSON : public Vehicle_Model {
  public:
    Full_JSON(std::string json_path);
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
ChVector<> InitPos;
ChQuaternion<> InitRot;
};
/// Vehicle simulated by the Node, entirely defined (physics & sensors) through JSON files
class CHROS_API RosVehicle {
  public:
    // =============================================================================

    RosVehicle(const std::string& lidar_json = "/Lidar.json",
             const std::string& vehicle_path = "/fullvehiclejson.json",
             const std::string& terrain_file = "/RigidPlane.json",
             bool render = true,
             double timestep = 3e-3);

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
    // Driver inputs:
    double target_acc = 0;
    double target_wheelang = 0;
    std::shared_ptr<Sedan_AIDriver> driver;
    std::shared_ptr<RigidTerrain> terrain;
    std::shared_ptr<WheeledVehicle> node_vehicle;
    std::shared_ptr<ChLidarSensor> lidar_sensor;
    std::shared_ptr<ChSensorManager> sens_manager;
};


}  // end namespace chronoros
}  // end namespace chrono

#endif
