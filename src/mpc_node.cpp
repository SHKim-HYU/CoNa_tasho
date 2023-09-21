#include "mpc_node.h"

MPCNode::MPCNode(const std::string& json_path) : loader_(json_path) {
    std::string mpc_path = loader_.getValue("mpc_file").asString();
    std::string ocp_path = loader_.getValue("ocp_file").asString();
    std::string pred_path = loader_.getValue("pred_file").asString();

    tc_mpc_ = casadi::Function::load(mpc_path);
    tc_ocp_ = casadi::Function::load(ocp_path);
    tc_pred_ = casadi::Function::load(pred_path);

    ROS_INFO("Loaded functions from provided paths.");
}

void MPCNode::mpcLoop() {
    // Load state, control, and parameter information from tc.json
    Json::Value x0 = loader_.getValue("x0");

    // Set inputs and evaluate the mpc function
    casadi::DM mpc_i0 = casadi::DM::zeros(814);  
    std::vector<casadi::DM> mpc_arg = {mpc_i0};
    std::vector<casadi::DM> mpc_res = tc_mpc_(mpc_arg);

    // Print the results
    casadi::DM mpc_o0 = mpc_res[0];
    ROS_INFO("MPC Result: %s", mpc_o0.toString().c_str());
}

void MPCNode::run() {
    ros::Rate loop_rate(100);  // 100Hz

    while (ros::ok()) {
        mpcLoop();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_tasho");
    MPCNode node("../lib/casadi/tc.json");
    node.run();

    return 0;
}