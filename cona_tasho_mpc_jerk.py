#!/usr/bin/env python3
"""
Motion planning model predictive control scheme
===============================================

##################
# cona_tasho_mpc.py
# Sunhong Kim
# tjsghd101@naver.com
# 18. Jul. 2023
##################
"""

import sys
from tasho import task_prototype_rockit as tp
from tasho import input_resolution
from tasho import environment as env
from tasho.templates.WMR import WMR
from tasho.MPC import MPC 
from tasho import default_mpc_options
from tasho import WorldSimulator
import pybullet as p
#from tasho import robot as rob
from robotshyu import Robot as rob
from numpy import pi, cos, sin, tan, square
from casadi import vertcat, horzcat, sumsqr, arctan2
from casadi import pi, cos, sin
from rockit import MultipleShooting, Ocp
from waypoints_functions import find_closest_point, find_closest_obstacle, index_last_point_fun, get_current_waypoints

import casadi as cs
import matplotlib.pyplot as plt
import tasho
import copy
import os
import numpy as np


from multiprocessing import Process, Manager
import rospy
import tf
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry


#############################################
################## Options ##################
#############################################

frame_enable = True
HSL = False
time_optimal = False
obstacle_avoidance = True


# Select prediction horizon and sample time for the MPC execution
horizon_samples = 50
t_mpc = 0.1 #in seconds

#############################################################################
################## Manager for global data multiprocessing ##################
#############################################################################

manager = Manager()
_q = manager.dict()
_qd = manager.dict()

_global_flag = manager.dict()
_task_flag = manager.dict()

_q['x']=0.0; _q['y']=0.0; _q['th']=0.0; _q['v']=0.0; _q['w']=0.0;
_qd['x']=0.0; _qd['y']=0.0; _qd['th']=0.0; _qd['v']=0.0; _qd['w']=0.0; _qd['dv']=0.0; _qd['dw']=0.0; 

_qd['xd_itp']=[[0.0]*horizon_samples]
_qd['yd_itp']=[[0.0]*horizon_samples]
_qd['thd_itp']=[[0.0]*horizon_samples]
_qd['xd_itp']=[[0.0]*horizon_samples]
_qd['yd_itp']=[[0.0]*horizon_samples]

_global_flag['MPC_Fail'] = False; _global_flag['OCP_Solved'] = False; _global_flag['Interpolated'] = False; _global_flag['OCP_Solved_mobile'] = False; _global_flag['Interpolated_mobile'] = False;
_global_flag['isHoming'] = False; _global_flag['initial_data'] = False; _global_flag['trajflag'] = [0]*6;
_global_flag['isArrived'] = False; _global_flag['base_flag'] = False; _global_flag['PVNet_received'] = False; _global_flag['buf_mobile']=3;

_task_flag['Task_Transition'] = False;
_task_flag['Task_Robot'] = 0



def publisher_run():
    pass


def mpc_run():
    
    ################################################
    # Define robot and initial joint angles
    ################################################
    # Import the robot object from the robot's repository (includes functions for FD, ID, FK, joint limits, etc)
    robot = rob.Robot("mir250_ppr", analytical_derivatives=False)

    # Define initial conditions of the robot
    q0_val = [0.0, 0.0, 0.0]
    dq0_val = [0.0, 0.0]
    ddq0_val = [0.0, 0.0]

    # Update robot's parameters if needed
    max_task_vel_ub = cs.DM([1.2, pi/6])
    max_task_vel_lb = cs.DM([0, -pi/6])
    max_task_acc = cs.DM([3, 3*pi])
    max_task_jerk = cs.DM([200,200*pi])
    robot.set_task_velocity_limits(lb=max_task_vel_lb, ub=max_task_vel_ub)
    robot.set_task_acceleration_limits(lb=-max_task_acc, ub=max_task_acc)
    robot.set_task_jerk_limits(lb=-max_task_jerk, ub=max_task_jerk)

    ################################################
    # Task spacification - Approximation to object
    ################################################

    if time_optimal:
        tc = tp.task_context(horizon_steps = horizon_samples, time_init_guess=horizon_samples*t_mpc)
    else:
        tc = tp.task_context(time= horizon_samples*t_mpc, horizon_steps = horizon_samples)

    x_0, y_0, th_0, v_0, w_0, dv_0, dw_0, ddv_0, ddw_0, x0_0, y0_0, th0_0, v0_0, w0_0, dv0_0, dw0_0 = WMR(robot,tc, options={'nonholonomic':True, 'jerk':True})

    # Minimal time
    tc.minimize_time(10, 0)

    # Define physical path parameter
    waypoints = tc.create_parameter('waypoints', (3,1), stage=0, grid='control')
    waypoint_last = tc.create_parameter('waypoint_last', (3,1), stage=0)
    p = cs.vertcat(x_0,y_0,th_0)

    if obstacle_avoidance == True:
        # Round Obstacle
        switch = tc.create_parameter('switch',(1,1), stage=0)
        obs_p = tc.create_parameter('obs_p', (2,1), stage=0)
        # obs_p = tc.create_parameter('obs_p', (2,1), stage=0, grid='control')
        obs_r = 1
        obs_con = {'inequality':True, 'hard':False, 'expression':np.sqrt(cs.sumsqr(p[0:2]-obs_p)), 'lower_limits':obs_r, 'norm':'L1', 'gain':switch*1e0}
        tc.add_task_constraint({"path_constraints":[obs_con]}, stage = 0)

    # Regularization
    tc.add_regularization(expression = v_0, weight = 1e-1, stage = 0)
    tc.add_regularization(expression = w_0, weight = 1e-1, stage = 0)

    tc.add_regularization(expression = dv_0, weight = 1e-1, stage = 0)
    tc.add_regularization(expression = dw_0, weight = 1e-1, stage = 0)

    tc.add_regularization(expression = ddv_0, weight = 1e-2, stage = 0)
    tc.add_regularization(expression = ddw_0, weight = 1e-2, stage = 0)

    # Path_constraint
    path_pos1 = {'hard':False, 'expression':x_0, 'reference':waypoints[0], 'gain':2e1, 'norm':'L2'}
    path_pos2 = {'hard':False, 'expression':y_0, 'reference':waypoints[1], 'gain':2e1, 'norm':'L2'}
    path_pos3 = {'hard':False, 'expression':th_0, 'reference':waypoints[2], 'gain':2e1, 'norm':'L2'}
    tc.add_task_constraint({"path_constraints":[path_pos1, path_pos2, path_pos3]}, stage = 0)

    final_pos = {'hard':False, 'expression':p, 'reference':waypoint_last, 'gain':2e1, 'norm':'L2'}
    tc.add_task_constraint({"final_constraints":[final_pos]}, stage = 0)

    ################################################
    # Set parameter values
    ################################################
    # Initial value update
    tc.set_value(x0_0, q0_val[0], stage=0)
    tc.set_value(y0_0, q0_val[1], stage=0)
    tc.set_value(th0_0, q0_val[2], stage=0)
    tc.set_value(v0_0, dq0_val[0], stage=0)
    tc.set_value(w0_0, dq0_val[1], stage=0)
    tc.set_value(dv0_0, ddq0_val[0], stage=0)
    tc.set_value(dw0_0, ddq0_val[1], stage=0)

    # Initial value for control inputs
    tc.set_initial(ddv_0, 0, stage=0)
    tc.set_initial(ddw_0, 0, stage=0)

    # Define reference path
    pathpoints = 300
    ref_path = {}
    ref_path['x'] = 1.7*np.sin(np.linspace(0,4*np.pi, pathpoints+1))
    ref_path['y'] = np.linspace(0,2, pathpoints+1)**2*2.5
    theta_path = [cs.arctan2(ref_path['y'][k+1]-ref_path['y'][k], ref_path['x'][k+1]-ref_path['x'][k]) for k in range(pathpoints)] 
    ref_path['theta'] = theta_path + [theta_path[-1]]

    if obstacle_avoidance==True:
        ref_obs = {}
        ref_obs['x'] = np.array([0.2, 0.0])
        ref_obs['y'] = np.array([2.5, 6])

    # ref_path['x'] = np.linspace(0,5, pathpoints+1)
    # ref_path['y'] = np.linspace(0,0, pathpoints+1)
    # ref_path['theta'] = np.linspace(0,0, pathpoints+1)

    wp = cs.horzcat(ref_path['x'], ref_path['y'], ref_path['theta']).T

    # First waypoint is current position
    index_closest_point = 0

    # Create a list of N waypoints
    current_waypoints = get_current_waypoints(index_closest_point, wp, horizon_samples, dist=5)

    # Set initial value for waypoint parameters
    tc.set_value(waypoints,current_waypoints[:,:-1], stage=0)
    tc.set_value(waypoint_last,current_waypoints[:,-1], stage=0)

    if obstacle_avoidance==True:
        # Set initial switch value for obstacle avoidance
        dist_obs, closest_obs = find_closest_obstacle(q0_val[:2], ref_obs)
        tc.set_value(obs_p,[ref_obs['x'][closest_obs],ref_obs['y'][closest_obs]], stage=0)
        if dist_obs>=2:
            tc.set_value(switch,0,stage=0)
        else:
            tc.set_value(switch,1,stage=0)

    # Add an output port for task velocities as well
    tc.tc_dict["out_ports"].append({"name":"port_out_x0", "var":"x0", "desc": "output port for the task position x"})
    tc.tc_dict["out_ports"].append({"name":"port_out_y0", "var":"y0", "desc": "output port for the task position x"})
    tc.tc_dict["out_ports"].append({"name":"port_out_th0", "var":"th0", "desc": "output port for the task position th"})
    tc.tc_dict["out_ports"].append({"name":"port_out_v0", "var":"v0", "desc": "output port for the task linear velocities"})
    tc.tc_dict["out_ports"].append({"name":"port_out_w0", "var":"w0", "desc": "output port for the task angular velocities"})
    tc.tc_dict["out_ports"].append({"name":"port_out_dv0", "var":"dv0", "desc": "output port for the task linear acceleration"})
    tc.tc_dict["out_ports"].append({"name":"port_out_dw0", "var":"dw0", "desc": "output port for the task angular acceleration"})

    # Add a monitor for termination criteria
    tc.add_monitor(
        {
            "name":"termination_criteria", 
            "expression":np.sqrt(cs.sumsqr(p-wp[:,-1])) - 1e-2, 
            "reference": 0.0, 
            "lower": True, 
            "initial": True,
        }
    )

    # # Add a monitor for termination criteria
    # tc.add_monitor(
    #     {
    #         "name": "termination_criteria",
    #         "expression": cs.sqrt(cs.sumsqr(v_0)) - 0.001,
    #         "reference": 0.0,  # doesn't matter
    #         "greater": True,  # doesn't matter
    #         "initial": True,
    #     }
    # )

    ################################################
    # Set solver and discretization options
    ################################################
    sol_options = {"ipopt": {"print_level": 0}}
    tc.set_ocp_solver('ipopt', sol_options)
    mpc_options = default_mpc_options.get_default_mpc_options()
    # tc.set_ocp_solver(mpc_options['ipopt_lbfgs_hsl']['solver_name'], mpc_options['ipopt_lbfgs_hsl']['options'])
    # tc.set_ocp_solver(mpc_options['ipopt']['solver_name'], mpc_options['ipopt']['options'])
    tc.mpc_solver = tc.ocp_solver
    tc.mpc_options = tc.ocp_options

    disc_options = {
        "discretization method": "multiple shooting",
        "horizon size": horizon_samples,
        "order": 1,
        "integration": "rk",
    }
    tc.set_discretization_settings(disc_options)



    ################################################
    # Solve the OCP wrt a parameter value (for the first time)
    ################################################
    # Solve the optimization problem
    sol = tc.solve_ocp()

    # Generate the controller component
    dir_casadi_func = "casadi_dir"
    os.makedirs("./"+dir_casadi_func, exist_ok=True)
    cg_opts={"ocp_cg_opts":{"save":True, "codegen":False, "jit":False}, "mpc":True, "mpc_cg_opts":{"save":True, "codegen":False, "jit":False}}
    vars_db = tc.generate_MPC_component("./"+dir_casadi_func+"/", cg_opts)

    MPC_component = MPC("mir250_path_following", "./"+dir_casadi_func+"/"+ tc.name + ".json")

    start = time.time()
    # Begin the visualization by applying the initial control signal
    t_sol, x_0_sol     = tc.sol_sample(x_0, grid="control",     stage = 0)
    t_sol, y_0_sol     = tc.sol_sample(y_0, grid="control",     stage = 0)
    t_sol, th_0_sol = tc.sol_sample(th_0, grid="control", stage = 0)
    t_sol, v_0_sol = tc.sol_sample(v_0, grid="control", stage = 0)
    t_sol, w_0_sol     = tc.sol_sample(w_0, grid="control",     stage = 0)
    t_sol, dv_0_sol = tc.sol_sample(dv_0, grid="control", stage = 0)
    t_sol, dw_0_sol     = tc.sol_sample(dw_0, grid="control",     stage = 0)

    ################################################
    # MPC Simulation
    ################################################

    # Create world simulator based on pybullet
    obj = WorldSimulator.WorldSimulator(bullet_gui=True)
    obj.visualization_realtime=True
    # Add robot to the world environment
    position = [0.0, 0.0, 0.0]
    orientation = [0.0, 0.0, 0.0, 1.0]

    robotID = obj.add_robot(position, orientation,robot_pkg=robot)

    if frame_enable:
        frameIDs = [0]*horizon_samples
        for i in range(horizon_samples):
            frameIDs[i] = obj.add_robot(position,orientation,robot_pkg=robot,frame=True)

    # Set environment
    environment = env.Environment()
    package_path = tasho.__path__[0]
    if obstacle_avoidance==True:
        # [ToDo] Describe obstacles
        obs1 = env.Cube(length = 0.7, position = [ref_obs['x'][0], ref_obs['y'][0], 0.35], orientation = [0.0, 0.0, 0.0, 1.0], urdf = package_path+"/models/objects/cube.urdf")
        environment.add_object(obs1, "obs1")
        obs2 = env.Cube(length = 0.7, position = [ref_obs['x'][1], ref_obs['y'][1], 0.35], orientation = [0.0, 0.0, 0.0, 1.0], urdf = package_path+"/models/objects/cube.urdf")
        environment.add_object(obs2, "obs2")

    for i in range(pathpoints):
        orientation=tf.transformations.quaternion_from_euler(0,0,ref_path['theta'][i])
        path = env.Frame(length = 20, position = [ref_path['x'][i], ref_path['y'][i], 0.0], orientation = orientation, urdf = package_path+"/models/objects/frame.urdf", fixed=True)
        environment.add_object(path, "path"+str(i))

    environment.set_in_world_simulator(obj)
        # cubeID = environment.get_object_ID("cube")
        # cube2ID = environment.get_object_ID("cube2")

    # Determine number of samples that the simulation should be executed
    no_samples = int(t_mpc / obj.physics_ts)
    if no_samples != t_mpc / obj.physics_ts:
        print("[ERROR] MPC sampling time not integer multiple of physics sampling time")

    # Correspondence between joint numbers in bullet and OCP
    joint_indices = [0, 1, 2]

    start = time.time()
     # Begin the visualization by applying the initial control signal
    t_sol, x_0_sol     = tc.sol_sample(x_0, grid="control",     stage = 0)
    t_sol, y_0_sol     = tc.sol_sample(y_0, grid="control",     stage = 0)
    t_sol, th_0_sol = tc.sol_sample(th_0, grid="control", stage = 0)
    t_sol, v_0_sol = tc.sol_sample(v_0, grid="control", stage = 0)
    t_sol, w_0_sol     = tc.sol_sample(w_0, grid="control",     stage = 0)
    t_sol, dv_0_sol = tc.sol_sample(dv_0, grid="control", stage = 0)
    t_sol, dw_0_sol     = tc.sol_sample(dw_0, grid="control",     stage = 0)

    obj.resetJointState(robotID,joint_indices,q0_val)
    if frame_enable==True:
        obj.resetMultiJointState(frameIDs, joint_indices, [q0_val])

    # Twist
    twist_0 = [v_0_sol[0]*np.cos(th_0_sol[0]), v_0_sol[0]*np.sin(th_0_sol[0]), w_0_sol[0]]

    obj.setController(
        robotID, "velocity", joint_indices, targetVelocities=twist_0
    )

    q_log = []
    q_dot_log = []
    predicted_pos_log = []

    x_pred = [0]*horizon_samples
    y_pred = [0]*horizon_samples
    th_pred = [0]*horizon_samples
    q_pred = [0]*horizon_samples

    cnt=0
    glob_time=time.time()
    glob_time_buf=0
    init_time_buf=0
    loop_time = 0
    while True:
        init_time=time.time()
        print("----------- MPC execution -----------")
        if cnt!=0:
            print("loop time: %f [ms]"%(1000*(loop_time)))
        start = time.time()
        q_now = obj.readJointPositions(robotID, joint_indices)
        dq_now = obj.readJointVelocities(robotID, joint_indices)

        # initialize values
        MPC_component.input_ports["port_inp_x00"]["val"] = q_now[0]
        MPC_component.input_ports["port_inp_y00"]["val"] = q_now[1]
        MPC_component.input_ports["port_inp_th00"]["val"] = q_now[2]
        MPC_component.input_ports["port_inp_v00"]["val"] = np.sqrt(dq_now[0]**2+dq_now[1]**2)
        MPC_component.input_ports["port_inp_w00"]["val"] = dq_now[2]

        # Find closest point on the reference path compared witch current position
        index_closest_point = find_closest_point(q_now[:2], ref_path, index_closest_point)

        # Create a list of N waypoints
        current_waypoints = get_current_waypoints(index_closest_point, wp, horizon_samples, dist=5)

        if obstacle_avoidance==True:
            # Set initial switch value for obstacle avoidance
            dist_obs, closest_obs = find_closest_obstacle(q_now[:2], ref_obs)
            MPC_component.input_ports["port_inp_obs_p"]["val"] = [ref_obs['x'][closest_obs],ref_obs['y'][closest_obs]]
            if dist_obs>=2:
                print("dist_obs: ", dist_obs, "switch: ",0, "idx_obs: ",closest_obs)
                MPC_component.input_ports["port_inp_switch"]["val"] = 0
            else:
                print("dist_obs: ", dist_obs, "switch: ",1, "idx_obs: ",closest_obs)
                MPC_component.input_ports["port_inp_switch"]["val"] = 1

        MPC_component.input_ports["port_inp_waypoints"]["val"] = cs.vec(current_waypoints[:,:-1]) # Input must be 'list'
        MPC_component.input_ports["port_inp_waypoint_last"]["val"] = cs.vec(current_waypoints[:,-1]) # Input must be 'list'

        if cnt == 0:
            MPC_component.configMPC()

        MPC_component.runMPC()

        if frame_enable==True:
            sol = MPC_component.res_vals
            for i in range(horizon_samples):
                x_pred[i]=sol[i+1].full()
                y_pred[i]=sol[horizon_samples+i+1].full()
                th_pred[i]=sol[2*horizon_samples+i+1].full()
                q_pred[i] = [x_pred[i], y_pred[i], th_pred[i]]
            obj.resetMultiJointState(frameIDs, joint_indices, q_pred)

        q_log.append(q_now)
        q_dot_log.append(dq_now)

        # Set control signal to the simulated robot
        xd_control_sig = MPC_component.output_ports["port_out_x0"]["val"].full()
        yd_control_sig = MPC_component.output_ports["port_out_y0"]["val"].full()
        thd_control_sig = MPC_component.output_ports["port_out_th0"]["val"].full()
        vd_control_sig = MPC_component.output_ports["port_out_v0"]["val"].full()
        wd_control_sig = MPC_component.output_ports["port_out_w0"]["val"].full()
        dvd_control_sig = (MPC_component.output_ports["port_out_dv0"]["val"] * t_mpc).full()
        dwd_control_sig = (MPC_component.output_ports["port_out_dw0"]["val"] * t_mpc).full()
        twist_d = [(vd_control_sig+dvd_control_sig)*np.cos(thd_control_sig), (vd_control_sig+dvd_control_sig)*np.sin(thd_control_sig),wd_control_sig+dwd_control_sig]
        # print(twist_d)
        obj.setController(
            robotID, "velocity", joint_indices, targetVelocities=twist_d
        )
        print("v: ", vd_control_sig, ", w: ", wd_control_sig)
        # Simulate
        obj.run_simulation(no_samples)
        end=time.time()
        
        # Termination criteria
        # if "termination_criteria_true" in MPC_component.event_output_port:
        #     break

        cnt+=1
        print("comp time = %f[ms]"%(1000*(time.time()-start)))
        while time.time()-init_time<t_mpc:
                time.sleep(1/100000000)
                glob_time_buf=time.time()
                init_time_buf=init_time

        loop_time = time.time()-init_time
        init_time_buf=init_time

    obj.end_simulation()



#############################################################################################################################

if __name__=='__main__':
    mpc_task = Process(target=mpc_run, args=())
    publisher_task = Process(target=publisher_run, args=())
    try:
        mpc_task.start()
        publisher_task.start()

        mpc_task.join()
        publisher_task.join()

    except KeyboardInterrupt:
        mpc_task.terminate()
        publisher_task.terminate()
