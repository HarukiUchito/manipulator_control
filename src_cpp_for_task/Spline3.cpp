#include "Spline3.hpp"

Spline3::Spline3(std::vector<double> &v) {
    auto vn = v.size();
    cofA = std::vector<double>(vn);
    cofB = std::vector<double>(vn);
    cofC = std::vector<double>(vn);
    cofD = std::vector<double>(vn);

    for (int i = 0; i < vn; ++i)
        cofA[i] = v[i];
    
    cofC[0] = cofC[vn-1] = 0.0;
    for (int i = 1; i < vn-1; ++i)
        cofC[i] = 3.0 * (cofA[i-1] - 2.0 * cofA[i] + cofA[i+1]);
    
    double tmp;
    std::vector<double> w(vn-1);
    w[0] = 0.0;
    for (int i = 1; i < vn-1; ++i) {
        tmp = 4.0 - w[i-1];
        cofC[i] = (cofC[i] - cofC[i-1]) / tmp;
        w[i] = 1.0 / tmp;
    }

    for (int i = vn-1; i > 0; --i)
        cofC[i] = cofC[i] - cofC[i+1] * w[i];
    
    cofB[vn-1] = cofD[vn-1] = 0.0;
    for (int i = 0; i < vn-1; ++i) {
        cofD[i] = (cofC[i+1] - cofC[i]) / 3.0;
        cofB[i] = cofA[i+1] - cofA[i] - cofC[i] - cofD[i];
    }
}

Spline3::~Spline3() {

}

double Spline3::calc(double t) {
    int i = (int)floor(t);
    if (i < 0) i = 0;
    else if (i > cofA.size()-1) i = cofA.size()-1;

    double dt = t - (double)i;
    return cofA[i] + (cofB[i] + (cofC[i] + cofD[i] * dt) * dt) * dt;
}