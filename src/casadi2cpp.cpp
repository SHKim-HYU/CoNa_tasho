/*! 
 *  @file casadi2codegen.cpp
 *  @brief Example code for casadi function
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Sep. 21. 2023
 *  @Comm
 */

// g++ casadi2cpp.cpp `pkg-config --cflags --libs casadi` -o casadi2cpp


#include <casadi/casadi.hpp>
#include <iostream>
#include <chrono>
using namespace std::chrono;

#ifndef CASADI_RESOURCE_PATH
#define CASADI_RESOURCE_PATH ""
#endif


int main() {

    // //// 1. OCP: Initial guess
    // // CasADi Function Load.
    // std::string ocp_file_path = CASADI_RESOURCE_PATH;
    // ocp_file_path += "tc_ocp.casadi";
    // casadi::Function tc_ocp = casadi::Function::load(ocp_file_path);
    // std::cout << "tc_ocp: " << tc_ocp << std::endl;

    // // Input Setting..
    // casadi::DM ocp_i0 = casadi::DM::zeros(814);
    // std::cout << "MPC Input Size: " << ocp_i0.size() << std::endl;

    // // Evaluate CasADi Function
    // std::vector<casadi::DM> ocp_arg = {ocp_i0};
    // std::vector<casadi::DM> ocp_res = tc_ocp(ocp_arg);

    // // results.
    // casadi::DM ocp_o0 = ocp_res[0];
    // std::cout << "OCP Output Size: " << ocp_o0.size() << std::endl;

    // // Print res
    // std::cout << "OCP Result: " << ocp_o0 << std::endl;


    //// 2. MPC: MPC function in online loop
    // CasADi Function Load.
    std::string mpc_file_path = CASADI_RESOURCE_PATH;
    mpc_file_path += "tc_mpc.casadi";
    casadi::Function tc_mpc = casadi::Function::load(mpc_file_path);
    std::cout << "tc_mpc: " << tc_mpc << std::endl;

    // Input Setting..
    casadi::DM mpc_i0 = casadi::DM::zeros(814);
    std::cout << "MPC Input Size: " << mpc_i0.size() << std::endl;

    // Evaluate CasADi Function
    std::vector<casadi::DM> mpc_arg = {mpc_i0};
    system_clock::time_point StartTime = system_clock::now();
    std::vector<casadi::DM> mpc_res = tc_mpc(mpc_arg);
    system_clock::time_point EndTime = system_clock::now();
    milliseconds mill = duration_cast<milliseconds>(EndTime - StartTime);
    std::cout << "MPC function time: " << mill.count() << "ms" << std::endl;

    // results.
    casadi::DM mpc_o0 = mpc_res[0];
    std::cout << "MPC Output Size: " << mpc_o0.size() << std::endl;

    // Print res
    std::cout << "MPC Result: " << mpc_o0 << std::endl;


    // //// 3. Pred: Simulate the system by one step and shift the states and control vectors to warmstart the MPC
    // // CasADi Function Load.
    // std::string pred_file_path = CASADI_RESOURCE_PATH;
    // pred_file_path += "tc_pred.casadi";
    // casadi::Function tc_pred = casadi::Function::load(pred_file_path);
    // std::cout << "tc_pred: " << tc_pred << std::endl;

    // // Input data
    // std::vector<double> x0(9, 0.0); // States, Size of arg, initial value
    // std::vector<double> u(2, 0.0); // Control inputs, Size of arg, initial value
    // double T = 0.0;   // 
    // double t0 = 0.02; // initial_time
    // std::vector<double> p(15, 0.0); // Size of arg, initial value
    // double z0 = 0.0;

    // // Prepare input as DM objects
    // casadi::DM dm_x0 = casadi::DM(x0);
    // casadi::DM dm_u = casadi::DM(u);
    // casadi::DM dm_T = casadi::DM(T);
    // casadi::DM dm_t0 = casadi::DM(t0);
    // casadi::DM dm_p = casadi::DM(p);
    // casadi::DM dm_z0 = casadi::DM(z0);

    // // Prepare the input dictionary
    // std::map<std::string, casadi::DM> arg;
    // arg["x0"] = dm_x0;
    // arg["u"] = dm_u;
    // arg["T"] = dm_T;
    // arg["t0"] = dm_t0;
    // arg["p"] = dm_p;
    // arg["z0"] = dm_z0;

    // // Call the function
    // std::map<std::string, casadi::DM> res = tc_pred(arg);

    // // Extract and use the outputs
    // casadi::DM xf = res.at("xf");
    // casadi::DM Xi = res.at("Xi");
    // casadi::DM poly_coeff = res.at("poly_coeff");

    // std::cout << "xf: " << xf << std::endl;
    // std::cout << "Xi: " << Xi << std::endl;
    // std::cout << "poly_coeff: " << poly_coeff << std::endl;

    return 0;
}