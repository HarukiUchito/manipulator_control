

// data buffer for robot state
std::vector<unsigned char> data;

// Instances for each class
Connection cn;
RobotController rb(data); // can read and write data buffer to communicate with Connection class

// Initialization
cn.open(); // read trajectory points from input.in
rb.init();

// Trajectory calculation
cn.calcTrajectory(); // In this line, interpolated trajectory points and corresponding angles to make end-effector of manipulator be the point are calculated.

// cn.recieve(data) returns -1 in case that the end-effector already reached the last point of trajectory, 0 otherwise.
while (cn.recieve(data) >= 0) {
    // And receive function also manages the index of trajectory points to be sent by cn.send(data) function. For that, received current angles and current target angles are compared there.

    cn.send(data); // This function sends current target angles simply.

    sleep(); // until when 1 / freq[s] is passed in this loop
}

// Finalization
cn.close();
rb.finalize();