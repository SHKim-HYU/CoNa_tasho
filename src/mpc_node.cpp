#include "mpc_node.h"

MPCNode::MPCNode(const std::string& json_file) : loader_(json_file) {

    std::string mpc_path = loader_.getValue("mpc_file").asString();
    mpc_path = CASADI_RESOURCE_PATH + mpc_path;
    ROS_INFO("mpc_path: %s", mpc_path);
    tc_mpc_ = casadi::Function::load(mpc_path);

    std::string ocp_path = loader_.getValue("ocp_file").asString();
    ocp_path = CASADI_RESOURCE_PATH+ocp_path;
    ROS_INFO("ocp_path: %s", ocp_path);
    tc_ocp_ = casadi::Function::load(ocp_path);

    std::string pred_path = loader_.getValue("pred_file").asString();
    pred_path = CASADI_RESOURCE_PATH+pred_path; 
    ROS_INFO("pred_path: %s", pred_path);
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
    ROS_INFO("MPC Result: %s", mpc_o0);
}

void MPCNode::run() {
    ros::Rate loop_rate(5);  // 100Hz

    while (ros::ok()) {
        system_clock::time_point StartTime = system_clock::now();
        mpcLoop();
        loop_rate.sleep();
        system_clock::time_point EndTime = system_clock::now();
        microseconds micro = duration_cast<microseconds>(EndTime - StartTime);
        std::cout << "MPC function time: " << micro.count() << "us" << std::endl;

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_tasho");

    std::string json_file_path = CASADI_RESOURCE_PATH;
    json_file_path += "tc.json";
    MPCNode node(json_file_path);
    node.run();

    return 0;
}