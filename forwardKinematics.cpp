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
    double d;
    double alpha;
    double a;
    double theta;
};

// Function to approximate small values to zero
Matrix4d approximateToZero(const Matrix4d& matrix, double threshold = 1e-6) {
    Matrix4d result = matrix; // Copy the input matrix
    for (int i = 0; i < result.rows(); ++i) {
        for (int j = 0; j < result.cols(); ++j) {
            if (abs(result(i, j)) < threshold) {
                result(i, j) = 0; // Set to zero if below threshold
            }
        }
    }
    return result;
}

// Offset structure to store type and value
struct Offset {
    enum Type { ANGLE, DISTANCE } type; // Type of offset: ANGLE or DISTANCE
    double value;                      // Offset value
};

// Forward Kinematics function
Matrix4d forwardKinematics(const vector<DHParameter>& dhParameters, const vector<Offset>& offsets) {
    Matrix4d T = Matrix4d::Identity(); // Identity matrix for base frame

    for (size_t i = 0; i < dhParameters.size(); ++i) {
        const auto& param = dhParameters[i];
        const auto& offset = offsets[i];

        // Apply offset based on type
        double theta = param.theta;
        double a = param.a;
        double d = param.d;

        if (offset.type == Offset::ANGLE) {
            theta += offset.value; // Add angle offset to theta
        } else if (offset.type == Offset::DISTANCE) {
            a += offset.value; // Add distance offset to a
            //d += offset.value; // Add distance offset to d (adjust as necessary)
        }

        // Compute transformation matrix for this joint
        Matrix4d A;
        double c_theta = cos(theta), s_theta = sin(theta);
        double c_alpha = cos(param.alpha), s_alpha = sin(param.alpha);

        A << c_theta, -s_theta * c_alpha, s_theta * s_alpha, a * c_theta,
             s_theta, c_theta * c_alpha, -c_theta * s_alpha, a * s_theta,
             0,       s_alpha,            c_alpha,           d,
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
    {600, -PI/2, 0,0},           // Link 7
    {0, PI / 2, 0, 0},       // Link 8
    {0, 0, 0, 0},        // Link 9
    {0, -PI / 2, 0, 0},      // Link 10
    {0, PI / 2, 0, 0},       // Link 11
    {526, 0, 0, 0}           // Link 12
};


    // Define offsets for each joint
    vector<Offset> offsets = {
        {Offset::DISTANCE, 0},   // Joint 1: 30° angle offset
        {Offset::ANGLE, PI/2}, // Joint 2: 0.05m distance offset
        {Offset::ANGLE, 0},  
        {Offset::ANGLE, 0}, 
        {Offset::ANGLE, 0}, 
        {Offset::ANGLE, PI/2},
        {Offset::ANGLE, PI/2},
        {Offset::ANGLE, 0},
        {Offset::DISTANCE, 900},
        {Offset::ANGLE, 0},
        {Offset::ANGLE, 0},
        {Offset::ANGLE, 0}   
    };

    // Compute forward kinematics
    Matrix4d T = forwardKinematics(dhParameters, offsets);
    Matrix4d Tn = approximateToZero(T);
    cout << "Forward Kinematics Transformation Matrix:\n" << Tn << endl;
    


    //Compute the forward kinematics

}