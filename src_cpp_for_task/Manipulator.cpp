#include "Manipulator.hpp"
#include "math_utils.hpp"

#include <fstream>
#include <iostream>

Manipulator::Manipulator() {
}

int Manipulator::open() {
    std::fstream ifs("../param_files/dh_params.txt");
    if (ifs.fail()) {
        std::cout << "Failed to open dh params file." << std::endl;
        return -1;
    }
    
    // read params
    std::string line;
    getline(ifs, line); // parameter description
    while (getline(ifs, line)) {
        std::stringstream ssm(line);
        DH_row dr;
        ssm >> dr.id >> dr.alpha >> dr.a >> dr.d;
        dh_params.push_back(dr);
    }
    link_num = dh_params.size();
    j_angle = std::vector<double>(link_num-1);

    return link_num;
}

int Manipulator::close() {
    return 0;
}

std::vector<double> Manipulator::inverseKinematics(
    Eigen::VectorXd &target,
    std::vector<double> &angles
) {
    std::vector<double> c_angles(angles.size());
    std::copy(angles.begin(), angles.end(), c_angles.begin());

    const int MAX_ITR = 10000;
    int itr_cnt = 0;
    const double threshold = 1e-6;

    Eigen::MatrixXd weight(3, 3);
    weight = Eigen::MatrixXd::Identity(3, 3);

    for (int itr_cnt = 0; itr_cnt < MAX_ITR; ++itr_cnt) {
        Eigen::MatrixXd jc = getJacobian(c_angles);

        Eigen::VectorXd cur_pos = kinematics(c_angles).head(3);
        Eigen::VectorXd diff = target - cur_pos;

        auto E = diff.transpose() * weight * diff;
        if (E <= threshold) {
            if (itr_cnt > MAX_ITR)
                std::cout << "COULD NOT BE SOLVED! E: " << E << std::endl;
            break;
        }

        Eigen::VectorXd da;

        IK_method ikMethod = IK_method::LevenbergMarquardt;
        switch (ikMethod) {
            case IK_method::NewtonRaphson: {
                da = jc.completeOrthogonalDecomposition().pseudoInverse() * diff;
            } break;
            case IK_method::LevenbergMarquardt: {
                auto gk = jc.transpose() * weight * diff;
                auto Wn = E * Eigen::MatrixXd::Identity(3, 3) + Eigen::MatrixXd::Identity(3, 3) * 0.001;
                auto Hk = jc.transpose() * weight * jc + Wn;
        
                //Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(Hk);
                //auto da = dec.solve(gk);
                da = Hk.completeOrthogonalDecomposition().pseudoInverse() * gk;
            } break;
        }

        for (int i = 0; i < c_angles.size(); ++i)
            c_angles[i] += da(i);
    }

    return c_angles;
}

// Too naive
Eigen::MatrixXd Manipulator::getJacobian(std::vector<double> &angles) {
    Eigen::MatrixXd ret(3, link_num-1);
    std::vector<double> dangles(angles.size());
    std::copy(angles.begin(), angles.end(), dangles.begin());

    double d_eps = 1e-7;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < angles.size(); ++j) {
            dangles[j] += d_eps;
            auto pos_d = kinematics(dangles);
            auto pos_o = kinematics(angles);

    //      std::cout << (pos_d - pos_o) / d_eps << std::endl;
            Eigen::VectorXd pd = (pos_d - pos_o) / d_eps;
            //std::cout << pd << std::endl;
            ret(i, j) = pd(i);
            dangles[j] -= d_eps;
        }
    }
    //std::cout << ret << std::endl;
    return ret;
}

// return [x; y; z; 1]
Eigen::VectorXd Manipulator::kinematics(std::vector<double> &angles) {
    Eigen::VectorXd ret(4);
    ret << 0, 0, 0, 1;
    //std::cout << ret << std::endl;
    Eigen::MatrixXd t_mat(4, 4);
    t_mat = Eigen::MatrixXd::Identity(4, 4);
    for (int i = 1; i < link_num; ++i) {
        t_mat = t_mat * getTransform(i, angles[i-1]);
    }
    return t_mat * ret;
}

Eigen::Matrix<double, 4, 4> Manipulator::getTransform(const int i, const double th) {
    Eigen::Matrix<double, 4, 4> ret;
    double al = deg2rad(dh_params[i].alpha);
    double d = dh_params[i].d;
    double a = dh_params[i].a;
    double cl = cos(al), sl = sin(al);
    double ct = cos(-th), st = sin(-th);
    //std::cout << "used a " << dh_params[i].a << std::endl;
    /*
    ret << 
        ct, -st, 0.0, dh_params[i].a,
        cl*st, cl*ct, -sl, -d*sl,
        sl*st, sl*ct, cl, d*cl,
        0.0, 0.0, 0.0, 1.0
    ; */
    ret << 
        ct, -st*cl, st*sl, a*ct,
        st, ct*cl, -ct*sl, a*st,
        0.0, sl, cl, d,
        0.0, 0.0, 0.0, 1.0
    ;
    return ret;
}