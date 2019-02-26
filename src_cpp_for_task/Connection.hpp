#ifndef DEF_CONNECTION_H
#define DEF_CONNECTION_H

#include <vector>

#include <Eigen/Core>

class Connection
{
  public:
    Connection() {}
    ~Connection() {}

    int open(); // input.in
    int close(); // actually there's nothing to do for now

    void calcTrajectory();

    void out2DSpline(std::vector<double> &ox, std::vector<double> &oy, std::vector<double> &oz);
    void outRangles(std::vector<double> &a1, std::vector<double> &a2, std::vector<double> &a3);

    //send data to the robot. use explicit pointer convertion
    int send(std::vector<unsigned char> &data);

    //receive state of the robot. record to data. use explicit pointer convertion
    // this function increments the destination angles index if the difference between current angles and destination angles are small (below threshold "arrival_threshold").
    int receive(std::vector<unsigned char> &data);
    
  private:
    const double control_period = 50.0; // used to split trajectory.
    std::vector<double> tx, ty, tz, ts;
    std::vector<Eigen::VectorXd> trj_points;
    std::vector<std::vector<double>> rangles;
    int rangle_idx; // current destination angle idx. this value is incremented in receive function
    const double arrival_threshold = 1e-2;
};

#endif