#ifndef MPC_NODE_H
#define MPC_NODE_H

#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include "json_loader.h"

class MPCNode {
private:
    ros::NodeHandle nh_;
    casadi::Function tc_mpc_;
    casadi::Function tc_ocp_;
    casadi::Function tc_pred_;
    JsonLoader loader_;

    void mpcLoop();

public:
    explicit MPCNode(const std::string& json_path);
    void run();
};

#endif  // MPC_NODE_H