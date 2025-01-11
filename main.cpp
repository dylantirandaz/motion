// jan 10 2025
#include <iostream>
#include <array>
#include <cmath>

using JointAngles = std::array<double, 6>;

struct RobotState
{
    JointAngles jointAngles;
};

struct Matrix4x4
{
    double m[16];
};

Matrix4x4 IdentityMatrix()
{
    Matrix4x4 I;
    for (int i = 0; i < 16; i++)
    {
        I.m[i] = (i % 5 == 0) ? 1.0 : 0.0;
    }
    return I;
}

//placeholder fowrard kinematics func,
// ill add denvait-hartenbeg later

Matrix4x4 forwardKinematics(const RobotState& state)
{
    //just placeholder for some numeric output rn
    Matrix4x4 T = IdentityMatrix();
    double xOffset = 0.5 * std::cos(state.jointAngles[0]);
    double yOffset = 0.5 * std::sin(state.jointAngles[0]);
    
    T.m[12] = xOffset;
    T.m[13] = yOffset;
    T.m[14] = 0.3; //constant offset
    
    return T;
}


// a placeholder axis alligned bounding box obstacle
struct Obstacle
{
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
};


int main(int argc, const char * argv[]) {
    RobotState robot;
    robot.jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    
    //compute forward kinematics
    Matrix4x4 endEffectorPose = forwardKinematics(robot);
    
    
    std::cout << "End-Effector Transform:\n";
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            std::cout << endEffectorPose.m[i*4 + j] << " ";
        }
        std::cout << "\n";
    }
    
    return 0;
}
