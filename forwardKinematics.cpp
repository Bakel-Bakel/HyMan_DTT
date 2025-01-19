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



int main(){
    //Define the DH table in form of array (vectors)
    vector<DHParameter> DTT_dhtable = {

    };

    //Compute the forward kinematics

}