#ifndef CHRONO_ROS_H
#define CHRONO_ROS_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

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
        ////return "hmmwv/tire/HMMWV_RigidTire.json";
        ////return "hmmwv/tire/HMMWV_FialaTire.json";
        ////return "hmmwv/tire/HMMWV_TMeasyTire.json";
        ////return "hmmwv/tire/HMMWV_Pac89Tire.json";
        return "hmmwv/tire/HMMWV_Pac02Tire.json";
    }
    virtual std::string PowertrainJSON() const override {
        return "hmmwv/powertrain/HMMWV_ShaftsPowertrain.json";
        ////return "hmmwv/powertrain/HMMWV_SimpleCVTPowertrain.json";
        ////return "hmmwv/powertrain/HMMWV_SimplePowertrain.json";
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
        ////return "sedan/tire/Sedan_RigidTire.json";
        return "sedan/tire/Sedan_TMeasyTire.json";
        ////return "sedan/tire/Sedan_Pac02Tire.json";
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
        ////return "uaz/vehicle/UAZBUS_Vehicle.json"; 
        ////return "uaz/vehicle/UAZ469_Vehicle.json";
        return "uaz/vehicle/UAZBUS_SAEVehicle.json";
    }
    virtual std::string TireJSON() const override {
        return "uaz/tire/UAZBUS_TMeasyTireFront.json";
        ////return "uaz/tire/UAZBUS_Pac02Tire.json";
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
        ////return "citybus/tire/CityBus_RigidTire.json";
        return "citybus/tire/CityBus_TMeasyTire.json";
        ////return "citybus/tire/CityBus_Pac02Tire.json";
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
        ////return "MAN_Kat1/vehicle/MAN_5t_Vehicle_4WD.json"; 
        ////return "MAN_Kat1/vehicle/MAN_7t_Vehicle_6WD.json";
        return "MAN_Kat1/vehicle/MAN_10t_Vehicle_8WD.json";
    }
    virtual std::string TireJSON() const override {
        return "MAN_Kat1/tire/MAN_5t_TMeasyTire.json";
    }
    virtual std::string PowertrainJSON() const override {
        ////return "MAN_Kat1/powertrain/MAN_5t_SimpleCVTPowertrain.json";
        return "MAN_Kat1/powertrain/MAN_7t_SimpleCVTPowertrain.json";
    }
    virtual double CameraDistance() const override { return 12.0; }
};



}  // end namespace vehicle
}  // end namespace chrono
#endif