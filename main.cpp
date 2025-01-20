#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

using JointAngles = std::array<double, 6>;

struct RobotState
{
    JointAngles jointAngles;
};

struct Matrix4x4
{
    double m[16];
};

Matrix4x4 identityMatrix()
{
    Matrix4x4 I;
    for(int i = 0; i < 16; i++)
        I.m[i] = (i % 5 == 0) ? 1.0 : 0.0;
    return I;
}

//placeholding forward kinematics
Matrix4x4 forwardKinematics(const RobotState& state)
{
    Matrix4x4 T = identityMatrix();
    double xOffset = 0.5 * std::cos(state.jointAngles[0]);
    double yOffset = 0.5 * std::sin(state.jointAngles[0]);
    T.m[12] = xOffset;
    T.m[13] = yOffset;
    T.m[14] = 0.3;
    return T;
}

struct Obstacle
{
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
};

bool inCollision(const RobotState& state, const Obstacle& obs)
{
    Matrix4x4 T = forwardKinematics(state);
    double x = T.m[12];
    double y = T.m[13];
    double z = T.m[14];
    
    bool insideX = (x >= obs.minX && x <= obs.maxX);
    bool insideY = (y >= obs.minY && y <= obs.maxY);
    bool insideZ = (z >= obs.minZ && z <= obs.maxZ);
    
    return (insideX && insideY && insideZ);
}


// Minimal 1D RRT for jointAngles[0] only

struct Node
{
    RobotState state;
    int parent; // index of parent in tree
};

double distance1D(const RobotState& a, const RobotState& b)
{
    // We only care about jointAngles[0]
    return std::fabs(a.jointAngles[0] - b.jointAngles[0]);
}

RobotState steer(const RobotState& from, const RobotState& to, double maxStep)
{
    // We only steer in jointAngles[0]
    double dist = distance1D(from, to);
    if(dist < maxStep)
    {
        return to;
    }
    else
    {
        RobotState newState = from;
        double dir = (to.jointAngles[0] - from.jointAngles[0]) > 0 ? 1.0 : -1.0;
        newState.jointAngles[0] += dir * maxStep;
        return newState;
    }
}

int main()
{
    Obstacle boxObstacle { 0.2, 0.2, 0.0,   0.4, 0.4, 0.5 };
    
    RobotState start;
    start.jointAngles = {0.0, 0, 0, 0, 0, 0};
    
    RobotState goal;
    goal.jointAngles = {1.57, 0, 0, 0, 0, 0}; // ~90 degrees in rad
    
    // RRT Parameters
    const int maxIterations = 1000;
    const double maxStep = 0.05;
    const double goalThreshold = 0.05;
    
    std::vector<Node> rrt;
    
    Node root;
    root.state = start;
    root.parent = -1;
    rrt.push_back(root);
    
    // Random engine for sampling
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distSampler(-3.14, 3.14); // random angle range
    
    bool reachedGoal = false;
    int goalNodeIndex = -1;
    
    for(int iter = 0; iter < maxIterations; iter++)
    {
        // sampling a random state
        RobotState randState;
        randState.jointAngles = { distSampler(gen), 0, 0, 0, 0, 0 };
        
        // find nearest node in RRT
        double bestDist = std::numeric_limits<double>::max();
        int nearestIndex = 0;
        for(int i = 0; i < (int)rrt.size(); i++)
        {
            double d = distance1D(rrt[i].state, randState);
            if(d < bestDist)
            {
                bestDist = d;
                nearestIndex = i;
            }
        }
        // steer it
        RobotState newState = steer(rrt[nearestIndex].state, randState, maxStep);
        
        // check collision
        if(!inCollision(newState, boxObstacle))
        {
            
            Node newNode;
            newNode.state = newState;
            newNode.parent = nearestIndex;
            rrt.push_back(newNode);
            
            double dGoal = distance1D(newState, goal);
            if(dGoal < goalThreshold)
            {
                std::cout << "Reached goal at iteration " << iter << "\n";
                reachedGoal = true;
                goalNodeIndex = (int)rrt.size()-1;
                break;
            }
        }
    }
    
    if(reachedGoal)
    {
        // reconstruct path
        std::vector<RobotState> path;
        int current = goalNodeIndex;
        while(current != -1)
        {
            path.push_back(rrt[current].state);
            current = rrt[current].parent;
        }
        // path is in reverse (goal -> start), so reverse it
        std::reverse(path.begin(), path.end());
        
        std::cout << "Path found with " << path.size() << " states:\n";
        for(const auto& st : path)
        {
            std::cout << "  jointAngles[0] = " << st.jointAngles[0] << "\n";
        }
    }
    else
    {
        std::cout << "Failed to reach goal within " << maxIterations << " iterations.\n";
    }
    
    return 0;
}

