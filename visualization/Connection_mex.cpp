#include "mex.h"
#include "class_handle.hpp"
#include "cppstream.hpp"

#include "../src_cpp_for_task/Connection.hpp"
#include "../src_cpp_for_task/Math_utils.hpp"

#include <imex/Core>
#include <imex/SparseCore>

#include <iostream>
#include <iomanip>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{	
    // Get the command string
    char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");
        
    // New
    if (!strcmp("new", cmd)) {
        // Check parameters
        if (nlhs != 1)
            mexErrMsgTxt("New: One output expected.");
        // Return a handle to a new C++ instance
        plhs[0] = convertPtr2Mat<Connection>(new Connection());
        return;
    }
    
    // Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<Connection>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    Connection* Connection_instance = convertMat2Ptr<Connection>(prhs[1]);
    
    // Be carefull !! 
    // First two arguments are already used to indicate command

    if (!strcmp("open", cmd)) {
        if (nlhs > 0 || nrhs < 2)
            mexErrMsgTxt("open: Unexpected arguments.");

        if (Connection_instance->open() == -1) {
            mexErrMsgTxt("open: File open error.");
            return;
        }

        return;
    }

    if (!strcmp("receive", cmd)) {
        if (nlhs > 1 || nrhs < 3)
            mexErrMsgTxt("receive: Unexpected arguments.");

        Eigen::Matrix<double, -1, -1> A = imex::Matrix<double>::InputWrapper(prhs[2]);
        std::vector<double> angles(3); // in radian
        for (int i = 0; i < 3; ++i) angles[i] = deg2rad(A(i, 0));
        
        double num = 3.1435869583394859248449392;
        std::cout << std::setprecision(30) << std::endl;
        std::cout << num << std::endl;
        std::vector<unsigned char> ucv = double2fix<10>(num);
        double tnum = fix2double<10>(ucv);
        std::cout << tnum << std::endl;

        Eigen::MatrixXd m(1, 1);
        m(0, 0) = Connection_instance->receive(angles);
        plhs[0] = imex::Matrix<double>::OutputWrapper(m);

        return;
    }

    if (!strcmp("send", cmd)) {
        if (nlhs > 1 || nrhs < 2)
            mexErrMsgTxt("send: Unexpected arguments.");

        std::vector<double> data(3);
        Connection_instance->send(data);

        Eigen::MatrixXd m(data.size(), 1);
        for (int i = 0; i < data.size(); ++i) {
            m(i, 0) = data[i];
        }
        plhs[0] = imex::Matrix<double>::OutputWrapper(m);
        return;
    }

    if (!strcmp("calcTrajectory", cmd)) {
        if (nlhs > 0 || nrhs < 2)
            mexErrMsgTxt("calcTrajectory: Unexpected arguments.");

        Connection_instance->calcTrajectory();
        return;
    }

    if (!strcmp("out2DSpline", cmd)) {
        if (nlhs > 1 || nrhs < 2)
            mexErrMsgTxt("out2DSpline: Unexpected arguments.");

        std::vector<double> x, y, z;
        Connection_instance->out2DSpline(x, y, z);
        Eigen::MatrixXd m(x.size(), 3);
        for (int i = 0; i < x.size(); ++i) {
            m(i, 0) = x[i];
            m(i, 1) = y[i];
            m(i, 2) = z[i];
        }
        plhs[0] = imex::Matrix<double>::OutputWrapper(m);
        return;
    }

    if (!strcmp("outRangles", cmd)) {
        if (nlhs > 1 || nrhs < 2)
            mexErrMsgTxt("outRangles: Unexpected arguments.");

        std::vector<double> a1, a2, a3;
        Connection_instance->outRangles(a1, a2, a3);
        Eigen::MatrixXd m(a1.size(), 3);
        for (int i = 0; i < a1.size(); ++i) {
            m(i, 0) = a1[i];
            m(i, 1) = a2[i];
            m(i, 2) = a3[i];
        }
        plhs[0] = imex::Matrix<double>::OutputWrapper(m);
        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}

static scoped_redirect_cout mycout_redirect;