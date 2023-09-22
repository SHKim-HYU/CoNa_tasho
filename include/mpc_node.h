#ifndef MPC_NODE_H
#define MPC_NODE_H

#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include "json_loader.h"
#include <iostream>
#include <chrono>
using namespace std::chrono;

#ifndef CASADI_RESOURCE_PATH
#define CASADI_RESOURCE_PATH ""
#endif

class MPCNode {
private:
    ros::NodeHandle nh_;
    casadi::Function tc_mpc_;
    casadi::Function tc_ocp_;
    casadi::Function tc_pred_;
    JsonLoader loader_;

    std::string json_path;

    void mpcLoop();

public:
    explicit MPCNode(const std::string& json_path);
    void run();
};

#endif  // MPC_NODE_H