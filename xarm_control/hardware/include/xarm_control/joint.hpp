#ifndef XARM_CONTROL_JOINT_HPP
#define XARM_CONTROL_JOINT_HPP

#include <string>
#include <cmath>


class Joint
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double reduction = 0;

    Joint() = default;

    Joint( int reduction)
    {
      setup(reduction);
    }

    
    void setup(const int &joint_reduction)
    {
      reduction = joint_reduction;
    }

};


#endif // XARM_CONTROL_JOINT_HPP