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

// check if the end effector is colliding with box
// will be filled as T.m[12], T.m[13], T.m[14]

bool inCollision(const RobotState& state, const Obstacle& obs)
{
    Matrix4x4 T = forwardKinematics(state);
    double x =T.m[12];
    double y = T.m[13];
    double z = T.m[14];
    
    bool insideX = (x >= obs.minX && x <= obs.maxX);
    bool insideY = (y >= obs.minY && y <= obs.maxY);
    bool insideZ = (z >= obs.minZ && z <= obs.maxZ);
    
    return (insideX && insideY && insideZ);
}



int main() {
    //simulating a box
    Obstacle boxObstacle { 0.2, 0.2, 0.0,  0.4, 0.4, 0.5};
    RobotState testState;
    
    //test for PoC
    testState.jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool collision = inCollision(testState, boxObstacle);
    std::cout << "Collision at jointAngles[0] = 0.0 ? "
    << (collision ? "YES" : "NO") << std::endl;
    
    //try moving
    testState.jointAngles = {0.785, 0.0, 0.0, 0.0, 0.0, 0.0}; //45 degree ang
    collision = inCollision(testState, boxObstacle);
    std::cout << "Collision at jointAngles[0 = 0.785 rad? "
    << (collision ? "YES" : "NO") << std::endl;
    
    return 0;
}
