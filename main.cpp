#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>

using JointAngles = std::array<double, 6>;

struct DHParameters
{
    double alpha;  
    double a;      
    double d;     
    double thetaOffset; 
};

static const std::array<DHParameters, 6> MY_ROBOT_DH = {{
    // alpha,   a,     d,    thetaOffset
    {  M_PI_2,  0.0,   0.3,  0.0 },  
    {  0.0,     0.3,   0.0,  0.0 },  
    {  0.0,     0.3,   0.0,  0.0 },  
    { -M_PI_2,  0.0,   0.2,  0.0 }, 
    {  M_PI_2,  0.0,   0.0,  0.0 },  
    {  0.0,     0.0,   0.1,  0.0 }   
}};

struct Matrix4x4
{
    double m[16];
};


Matrix4x4 identityMatrix()
{
    Matrix4x4 I{};
    for(int i = 0; i < 16; i++)
        I.m[i] = (i % 5 == 0) ? 1.0 : 0.0;
    return I;
}

Matrix4x4 multiply(const Matrix4x4& A, const Matrix4x4& B)
{
    Matrix4x4 R{};
    for(int row = 0; row < 4; row++)
    {
        for(int col = 0; col < 4; col++)
        {
            double sum = 0.0;
            for(int k = 0; k < 4; k++)
            {
                sum += A.m[row*4 + k] * B.m[k*4 + col];
            }
            R.m[row*4 + col] = sum;
        }
    }
    return R;
}

Matrix4x4 dhTransform(const DHParameters& dh, double jointAngle)
{
    Matrix4x4 T = identityMatrix();
    
    double theta = dh.thetaOffset + jointAngle;
    double ca = std::cos(dh.alpha);
    double sa = std::sin(dh.alpha);
    double ct = std::cos(theta);
    double st = std::sin(theta);

    T.m[0]  = ct;              T.m[1]  = -st * ca;          T.m[2]  =  st * sa;          T.m[3]  = dh.a * ct;
    T.m[4]  = st;              T.m[5]  =  ct * ca;          T.m[6]  = -ct * sa;          T.m[7]  = dh.a * st;
    T.m[8]  = 0.0;             T.m[9]  =  sa;               T.m[10] =  ca;               T.m[11] = dh.d;
    T.m[12] = 0.0;             T.m[13] =  0.0;              T.m[14] =  0.0;              T.m[15] = 1.0;
    
    return T;
}


std::vector<Matrix4x4> forwardKinematics(const JointAngles& angles)
{
    std::vector<Matrix4x4> transforms(6);
    Matrix4x4 current = identityMatrix();
    
    for(int i = 0; i < 6; i++)
    {
        Matrix4x4 linkTf = dhTransform(MY_ROBOT_DH[i], angles[i]);
        current = multiply(current, linkTf);  
        transforms[i] = current;
    }
    return transforms;
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

struct Node
{
    RobotState state;
    int parent;
};

double distance6D(const RobotState& a, const RobotState& b)
{
    double sum = 0.0;
    for(int i = 0; i < 6; i++)
    {
        double diff = a.jointAngles[i] - b.jointAngles[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

RobotState steer(const RobotState& from, const RobotState& to, double maxStep)
{
    double dist = distance6D(from, to);
    if(dist < maxStep)
    {
        return to;
    }
    else
    {
        RobotState newState = from;
        for(int i = 0; i < 6; i++)
        {
            double diff = to.jointAngles[i] - from.jointAngles[i];
            newState.jointAngles[i] += (diff / dist) * maxStep;
        }
        return newState;
    }
}

int main()
{
    Obstacle boxObstacle { 0.2, 0.2, 0.0,   0.5, 0.5, 0.6 };
    
    RobotState start;
    start.jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    RobotState goal;
    //picke something random-ish
    goal.jointAngles = {1.0, 1.2, -0.5, 0.8, -1.0, 0.5};
    
    const int maxIterations = 2000;
    const double maxStep = 0.05;
    const double goalThreshold = 0.2; // allow some margin in 6D
    
    std::vector<Node> rrt;
    
    Node root;
    root.state = start;
    root.parent = -1;
    rrt.push_back(root);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distSampler(-3.14, 3.14); 
    
    bool reachedGoal = false;
    int goalNodeIndex = -1;
    
    for(int iter = 0; iter < maxIterations; iter++)
    {
        RobotState randState;
        for(int i = 0; i < 6; i++)
        {
            randState.jointAngles[i] = distSampler(gen);
        }
        
        double bestDist = std::numeric_limits<double>::max();
        int nearestIdx = 0;
        for(int i = 0; i < (int)rrt.size(); i++)
        {
            double d = distance6D(rrt[i].state, randState);
            if(d < bestDist)
            {
                bestDist = d;
                nearestIdx = i;
            }
        }
        
        RobotState newState = steer(rrt[nearestIdx].state, randState, maxStep);
        
        if(!inCollision(newState, boxObstacle))
        {
            Node newNode;
            newNode.state = newState;
            newNode.parent = nearestIdx;
            rrt.push_back(newNode);
            
            double dGoal = distance6D(newState, goal);
            if(dGoal < goalThreshold)
            {
                std::cout << "Reached goal at iteration " << iter << "\n";
                reachedGoal = true;
                goalNodeIndex = (int)rrt.size() - 1;
                break;
            }
        }
    }
    
    if(reachedGoal)
    {
        std::vector<RobotState> path;
        int current = goalNodeIndex;
        while(current != -1)
        {
            path.push_back(rrt[current].state);
            current = rrt[current].parent;
        }
        std::reverse(path.begin(), path.end());
        
        std::cout << "Path found with " << path.size() << " states.\n";
        for(const auto& st : path)
        {
            std::cout << "  [ ";
            for(int i = 0; i < 6; i++)
            {
                std::cout << st.jointAngles[i] << (i<5?", ":" ");
            }
            std::cout << "]\n";
        }
    }
    else
    {
        std::cout << "Failed to reach goal within " << maxIterations << " iterations.\n";
    }
    
    return 0;
}
