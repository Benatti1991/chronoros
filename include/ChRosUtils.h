#ifndef CH_ROS_UTILS_H
#define CH_ROS_UTILS_H


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

/// Function to get files in chronoros data directory
std::string GetChronoRosDataFile(const std::string& filename){
    return CHRONOROS_DATA_DIR + filename;
}

/// Function to get files in assets directory
std::string GetAssetFile(const std::string& filename){
    return ASSETS_FOLDER_PATH + filename;
}

/*
void AddSceneMeshes(ChSystem* chsystem, RigidTerrain* terrain, ChVector&<> center, double radius, road_only = false) {
    // load all meshes in input file, using instancing where possible
    std::string base_path = GetChronoDataFile("/Environments/SanFrancisco/components_new/");
    std::string input_file = base_path + "instance_map_03.csv";
    // std::string input_file = base_path + "instance_map_roads_only.csv";

    std::ifstream infile(input_file);
    if (!infile.is_open())
        throw std::runtime_error("Could not open file " + input_file);
    std::string line, col;
    std::vector<std::string> result;

    std::unordered_map<std::string, std::shared_ptr<ChTriangleMeshConnected>> mesh_map;

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetBodyFixed(true);
    mesh_body->SetCollide(false);
    chsystem->Add(mesh_body);

    int meshes_added = 0;
    int mesh_offset = 0;
    int num_meshes = 20000;

    if (infile.good()) {
        int mesh_count = 0;
        int mesh_limit = mesh_offset + num_meshes;
        while (std::getline(infile, line) && mesh_count < mesh_limit) {
            if (mesh_count < mesh_offset) {
                mesh_count++;
            } else {
                mesh_count++;
                result.clear();
                std::stringstream ss(line);
                while (std::getline(ss, col, ',')) {
                    result.push_back(col);
                }
                // std::cout << "Name: " << result[0] << ", mesh: " << result[1] << std::endl;
                std::string mesh_name = result[0];
                std::string mesh_obj = base_path + result[1] + ".obj";

                // std::cout << mesh_name << std::endl;
                if (mesh_name.find("EmissionOn") == std::string::npos) {  // exlude items with
                                                                          // emission on

                    if (!load_roads_only || mesh_name.find("Road") != std::string::npos) {
                        ChVector<double> pos = {std::stod(result[2]), std::stod(result[3]), std::stod(result[4])};

                        if ((pos - simulation_center).Length() < loading_radius) {
                            // check if mesh is in map
                            bool instance_found = false;
                            std::shared_ptr<ChTriangleMeshConnected> mmesh;
                            if (mesh_map.find(mesh_obj) != mesh_map.end()) {
                                mmesh = mesh_map[mesh_obj];
                                instance_found = true;
                            } else {
                                mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
                                mmesh->LoadWavefrontMesh(mesh_obj, false, true);
                                mesh_map[mesh_obj] = mmesh;
                            }

                            ChQuaternion<double> rot = {std::stod(result[5]), std::stod(result[6]),
                                                        std::stod(result[7]), std::stod(result[8])};
                            ChVector<double> scale = {std::stod(result[9]), std::stod(result[10]),
                                                      std::stod(result[11])};

                            // if not road, only add visualization with new pos,rot,scale
                            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
                            trimesh_shape->SetMesh(mmesh);
                            trimesh_shape->SetName(mesh_name);
                            trimesh_shape->SetStatic(true);
                            trimesh_shape->SetScale(scale);
                            trimesh_shape->Pos = pos;
                            trimesh_shape->Rot = ChMatrix33<>(rot);

                            mesh_body->AddAsset(trimesh_shape);

                            meshes_added++;
                        }
                    }
                }
            }
        }
        std::cout << "Total meshes: " << meshes_added << " | Unique meshes: " << mesh_map.size() << std::endl;
    }
}*/

}  // end namespace chronoros
}  // end namespace chrono

#endif
