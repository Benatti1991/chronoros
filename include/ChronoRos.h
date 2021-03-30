#ifndef CHRONO_ROS_H
#define CHRONO_ROS_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {
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


class Full_JSON : public Vehicle_Model {
public:
    Full_JSON(std::string json_path) {
        Document d = ReadFileJSON(json_path);
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


}  // end namespace vehicle
}  // end namespace chrono
#endif