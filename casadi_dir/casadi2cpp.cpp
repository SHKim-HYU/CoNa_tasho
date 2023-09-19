/*! 
 *  @file casadi2codegen.cpp
 *  @brief Example code for casadi function
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Sep. 18. 2023
 *  @Comm
 */

// g++ casadi2cpp.cpp `pkg-config --cflags --libs casadi` -o casadi2cpp


#include <casadi/casadi.hpp>
#include <iostream>

int main() {

    //// 1. OCP: Initial guess
    // CasADi Function Load.
    casadi::Function tc_ocp = casadi::Function::load("tc_ocp.casadi");

    // Input Setting..
    casadi::DM ocp_i0 = casadi::DM::zeros(814);

    // Evaluate CasADi Function
    std::vector<casadi::DM> ocp_arg = {ocp_i0};
    std::vector<casadi::DM> ocp_res = tc_ocp(ocp_arg);

    // results.
    casadi::DM ocp_o0 = ocp_res[0];

    // Print res
    std::cout << "OCP Result: " << ocp_o0 << std::endl;


    //// 2. MPC: MPC function in online loop
    // CasADi Function Load.
    casadi::Function tc_mpc = casadi::Function::load("tc_mpc.casadi");

    // Input Setting..
    casadi::DM mpc_i0 = casadi::DM::zeros(814);

    // Evaluate CasADi Function
    std::vector<casadi::DM> mpc_arg = {mpc_i0};
    std::vector<casadi::DM> mpc_res = tc_ocp(mpc_arg);

    // results.
    casadi::DM mpc_o0 = mpc_res[0];

    // Print res
    std::cout << "MPC Result: " << mpc_o0 << std::endl;


    //// 3. Pred: Simulate the system by one step and shift the states and control vectors to warmstart the MPC
    // CasADi Function Load.
    casadi::Function tc_pred = casadi::Function::load("tc_pred.casadi");

    // Input data
    std::vector<double> x0(9, 0.0); // States, Size of arg, initial value
    std::vector<double> u(2, 0.0); // Control inputs, Size of arg, initial value
    double T = 0.0;   // 
    double t0 = 0.02; // initial_time
    std::vector<double> p(15, 0.0); // Size of arg, initial value

    // Prepare input as DM objects
    casadi::DM dm_x0 = casadi::DM(x0);
    casadi::DM dm_u = casadi::DM(u);
    casadi::DM dm_T = casadi::DM(T);
    casadi::DM dm_t0 = casadi::DM(t0);
    casadi::DM dm_p = casadi::DM(p);

    // Prepare the input dictionary
    std::map<std::string, casadi::DM> arg;
    arg["x0"] = dm_x0;
    arg["u"] = dm_u;
    arg["T"] = dm_T;
    arg["t0"] = dm_t0;
    arg["p"] = dm_p;

    // Call the function
    std::map<std::string, casadi::DM> res = tc_pred(arg);

    // Extract and use the outputs
    casadi::DM xf = res.at("xf");
    casadi::DM Xi = res.at("Xi");
    casadi::DM poly_coeff = res.at("poly_coeff");

    std::cout << "xf: " << xf << std::endl;
    std::cout << "Xi: " << Xi << std::endl;
    std::cout << "poly_coeff: " << poly_coeff << std::endl;

    return 0;
}