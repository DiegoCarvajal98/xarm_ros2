#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"

class XarmCommand : public rclcpp::Node
{
private:
    /* data */
public:
    XarmCommand();
};

XarmCommand::XarmCommand() : Node("xarm_command_pub")
{
    this->declare_parameter("min_pos", std::vector<int>{3000, 3000, 3000, 3000, 3000, 3000});
}


void main(){

}