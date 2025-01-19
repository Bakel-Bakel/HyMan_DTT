#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#if __cplusplus >= 202002L
    #include <numbers>
    #define PI std::numbers::pi

#elif defined(_USE_MATH_DEFINES) || defined(M_PI)
    #define _USE_MATH_DEFINES
    #define PI M_PI  

#else
    #define PI 3.141592653589793
#endif

using namespace std;
using namespace Eigen;

struct DHParameter {
    double a;
    double alpha;
    double d;
    double theta;
};

// Forward Kinematics using DH Parameters
Matrix4d forwardKinematics(const vector<DHParameter>& dhParameters) {
    Matrix4d T = Matrix4d::Identity(); // Identity matrix for base frame

    for (const auto& param : dhParameters) {
        Matrix4d A;
        double c_theta = cos(param.theta), s_theta = sin(param.theta);
        double c_alpha = cos(param.alpha), s_alpha = sin(param.alpha);

        // Transformation matrix for each joint
        A << c_theta, -s_theta * c_alpha, s_theta * s_alpha, param.a * c_theta,
             s_theta, c_theta * c_alpha, -c_theta * s_alpha, param.a * s_theta,
             0,       s_alpha,            c_alpha,           param.d,
             0,       0,                  0,                 1;

        T *= A; // Chain transformations
    }

    return T;
}

int main(){
    //Define the DH table in form of array (vectors)
    vector<DHParameter> dhParameters = {
    // Planar Arm 
    {0, PI / 2, 0, -PI / 2},  // Link 1
    {0, 0, 750, 0},           // Link 2
    {0, 0, 750, 0},           // Link 3
    {0, 0, 750, 0},           // Link 4
    {0, 0, 750, 0},           // Link 5

    // Spatial Arm 
    {0, 0, 0, 0,},             // Link 6
    {600, 0, 0,0},           // Link 7
    {0, -PI / 2, 0, 0},       // Link 8
    {0, PI / 2, 0, 0},        // Link 9
    {0, -PI / 2, 0, 0},      // Link 10
    {0, PI / 2, 0, 0},       // Link 11
    {526, 0, 0, 0}           // Link 12
};


     // Compute Forward Kinematics
    Matrix4d T = forwardKinematics(dhParameters);
    cout << "Forward Kinematics Transformation Matrix:\n" << T << endl;


    //Compute the forward kinematics

}