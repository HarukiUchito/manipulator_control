#ifndef DEF_MANIPULATOR_H
#define DEF_MANIPULATOR_H

#include <vector>
#include <string>

#include <Eigen/Dense>

enum IK_method {
    Exact,
    NewtonRaphson,
    LevenbergMarquardt,
};

struct DH_row
{
    int id;
    double alpha; // Twist in radian
    double a;     // Link length
    double d;     // Offset
};

class Manipulator
{
  public:
    // init parameters by the file
    Manipulator();
    ~Manipulator() {}

    int open(); // returns number of links
    int close();

    std::vector<double> inverseKinematics(
        Eigen::VectorXd &target,
        std::vector<double> &angles);
    Eigen::MatrixXd getJacobian(std::vector<double> &angles);
    Eigen::VectorXd kinematics(std::vector<double> &angles); // in radian
  private:
    int link_num;
    std::vector<double> j_angle; // in radian
    std::vector<Eigen::Matrix<double, 4, 4>> trfm_mats;
    std::vector<DH_row> dh_params;

    Eigen::Matrix<double, 4, 4> getTransform(const int i, const double th);
};

#endif