#include "Connection.hpp"
#include "Spline3.hpp"
#include "Manipulator.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <cmath>

int Connection::open() {
    std::fstream ifs("../param_files/input.in");
    if (ifs.fail()) {
        std::cout << "Failed to open file." << std::endl;
        return -1;
    }
    
    // read input
    std::string line;
    while (getline(ifs, line)) {
        std::stringstream ssm(line);
        double tmp, x, y, z;
        ssm >> x >> y >> z >> tmp;
        tx.push_back(x);
        ty.push_back(y);
        tz.push_back(z);
        ts.push_back(tmp);
    }

    return 0;
}

void Connection::calcTrajectory() {
    // Spline interpolation
    Spline3 sx(tx), sy(ty), sz(tz);
    for (int i = 0; i < ts.size()-1; ++i) {
        double time_difference = ts[i+1] - ts[i];
        int control_cnt = (int)(control_period * time_difference);
        for (int j = 0; j < control_cnt; ++j) {
            double t = (double)i + (double)j / (double)control_cnt;
            Eigen::VectorXd p(3);
            p << sx.calc(t), sy.calc(t), sz.calc(t);
            trj_points.push_back(p);
        }
    }
    double t = ts[ts.size()-1];
    Eigen::VectorXd p(3);
    p << sx.calc(t), sy.calc(t), sz.calc(t);
    trj_points.push_back(p);

    // Joint angle calculation
    Manipulator mp;
    int jnum = mp.open() - 1;
    std::vector<double> last(jnum, 0.0);
    for (int j = 0; j < trj_points.size(); ++j) {
        rangles.push_back(mp.inverseKinematics(trj_points[j], last));
        for (int i = 0; i < jnum; ++i)
            last[i] = rangles[rangles.size()-1][i];
    }
    rangle_idx = 0;
}

int Connection::receive(std::vector<double> &data) {
    double diff = 0.0;
    for (int i = 0; i < data.size(); ++i) {
        //std::cout << "data " << data[i] << std::endl;
        //std::cout << "rang " << rangles[rangle_idx][i] << std::endl;
        diff += fabs(data[i] - rangles[rangle_idx][i]);
    }
    if (diff < arrival_threshold)
        rangle_idx++;

    if (rangle_idx >= rangles.size())
        return -1; // to indicate the end of trajectory
    
    return 0;
}

int Connection::send(std::vector<double> &data) {
    if (rangle_idx >= rangles.size())
        return -1;

    for (int i = 0; i < data.size(); ++i) {
        data[i] = rangles[rangle_idx][i];
    }

    return 0;
}

int Connection::close() {
    return 0;
}

void Connection::out2DSpline(
    std::vector<double> &ox,
    std::vector<double> &oy,
    std::vector<double> &oz
) {
    for (const auto& p : trj_points) {
        ox.push_back(p(0));
        oy.push_back(p(1));
        oz.push_back(p(2));
    }
}

void Connection::outRangles(
    std::vector<double> &a1,
    std::vector<double> &a2,
    std::vector<double> &a3
) {
    for (const auto& angles : rangles) {
        a1.push_back(angles[0]);
        a2.push_back(angles[1]);
        a3.push_back(angles[2]);
    }
}