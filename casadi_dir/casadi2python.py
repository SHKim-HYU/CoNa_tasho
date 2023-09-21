#!/usr/bin/env python3
"""
casadi function to python example
===============================================

##################
# casadi2python.py
# Sunhong Kim
# tjsghd101@naver.com
# Sep. 21. 2023
##################
"""

import casadi as cs


i0=[0.0]*814 # Serialized inputs: please refer to .json file

### 1. OCP: Initial guess(first loop)
tc_ocp = cs.Function.load('tc_ocp.casadi')
ocp_res = tc_ocp(i0)
print("OCP Result: ", ocp_res)

### 2. MPC: MPC function in online loop
tc_mpc=cs.Function.load('tc_mpc.casadi')
mpc_res = tc_mpc(i0)
print("MPC Result: ", mpc_res)


### 3. Pred: Simulate the system by one step and shift the states and control vectors to warmstart the MPC
x0=[0.0]*9; # States input
u=[0.0]*2;  # Control input
T=0.0; 		# 
t0=0.25; 	# initial time
p=[0.0]*15  # initial parameters
z0=[0.0]
tc_pred=cs.Function.load('tc_pred.casadi')

# xf: , Xi: , poly_coeff: Coefficient matrix from RK4 to reconstruct 4th order polynomial (k1,k2,k3,k4),
# qf: , zf: , Zi: , poly_coeff_z: 
xf, Xi, poly_coeff, qf, zf, Zi, poly_coeff_z = tc_pred(x0,u,T,t0,p,z0)
print("Pred Result: ", xf, Xi, poly_coeff, qf, zf, Zi, poly_coeff_z)