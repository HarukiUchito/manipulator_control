#include "Connection.hpp"
#include "Spline3.hpp"
#include "Manipulator.hpp"
#include "Math_utils.hpp"

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
    // Joint angle calculation for each given points
    Manipulator mp;
    mp.open();
    
    std::vector<double> th0, th1, th2;
    for (int i = 0; i < tx.size(); ++i) {
        Eigen::VectorXd target(3);
        target << tx[i], ty[i], tz[i];
        
        auto ret = mp.inverseKinematics(target, std::vector<double>({0,0,0}));
        th0.push_back(ret[0]);
        th1.push_back(ret[1]);
        th2.push_back(ret[2]);
    }

    Spline3 s0(th0), s1(th1), s2(th2);
    for (int i = 0; i < ts.size()-1; ++i) {
        double time_difference = ts[i+1] - ts[i];
        int control_cnt = (int)(control_period * time_difference);
        for (int j = 0; j < control_cnt; ++j) {
            double t = (double)i + (double)j / (double)control_cnt;
            rangles.push_back(std::vector<double>({s0.calc(t), s1.calc(t), s2.calc(t)}));
        }
    }
    rangle_idx = 0;
/*
    // Spline interpolation in cartesian space
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

    // Joint angle calculation for each position
    Manipulator mp;
    int jnum = mp.open() - 1;
    std::vector<double> last(jnum, 0.0);
    for (int j = 0; j < trj_points.size(); ++j) {
        rangles.push_back(mp.inverseKinematics(trj_points[j], last));
        for (int i = 0; i < jnum; ++i)
            last[i] = rangles[rangles.size()-1][i];
    }
    rangle_idx = 0;
*/
}

int Connection::receive(std::vector<unsigned char> &data) {
    std::vector<double> rdata(3);
    for (int i = 0; i < 3; ++i) {
        auto first = data.begin() + 4 * i;
        auto last = data.begin() + 4 * (i + 1);
        std::vector<unsigned char> v(first, last);
        rdata[i] = fix2double<10>(v);
    }
    double diff = 0.0;
    for (int i = 0; i < rdata.size(); ++i) {
        diff += fabs(rdata[i] - rangles[rangle_idx][i]);
        //std::cout << rdata[i] << " " << rangles[rangle_idx][i] <<  std::endl;
    }
    if (diff < arrival_threshold)
        rangle_idx++;

    if (rangle_idx >= rangles.size())
        return -1; // to indicate the end of trajectory
    
    return 0;
}

int Connection::send(std::vector<unsigned char> &data) {
    if (rangle_idx >= rangles.size())
        return -1;

    for (int i = 0; i < 3; ++i) {
        auto ucv = double2fix<10>(rangles[rangle_idx][i]);
        for (int j = 0; j < 4; ++j)
            data[i * 4 + j] = ucv[j];
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