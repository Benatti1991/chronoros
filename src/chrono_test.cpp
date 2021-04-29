#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

/// CHRONO HEADERS

#include "ChRosNode.h"
///


using namespace std::chrono_literals;
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::chronoros;




void shutnode(ChRosNode& node){
    delete node.myvehicle.get();
}

int main(int argc, char *argv[]) {

    //std::this_thread::sleep_for(std::chrono::milliseconds(30000));
    rclcpp::init(argc, argv);
    auto mnode = std::make_shared<ChRosNode>();
    rclcpp::spin(mnode);
    std::function<void()> f_shutnode = [mnode]() {  shutnode(*mnode);};
    rclcpp::on_shutdown( f_shutnode);
    rclcpp::shutdown();
    return 0;
}
