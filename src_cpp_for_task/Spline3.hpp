#ifndef DEF_SPLINE3_H
#define DEF_SPLINE3_H

#include <vector>

class Spline3 {
  public:
    Spline3(std::vector<double> &v);
    ~Spline3();

    double calc(double t);
  private:
    std::vector<double> cofA, cofB, cofC, cofD;
};

#endif