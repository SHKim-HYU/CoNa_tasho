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

from scipy.interpolate import interp1d,BSpline
from scipy import interpolate

from multiprocessing import Process, Manager
import rospy
import tf
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry


#############################################
################## Options ##################
#############################################

gui_enable = True
env_enable = True
frame_enable = False
HSL = False
time_optimal = False
obstacle_avoidance = False
command_activate = False



# Select prediction horizon and sample time for the MPC execution
horizon_samples = 25
t_mpc = 0.2 #in seconds
duration = t_mpc*horizon_samples

#############################################################################
################## Manager for global data multiprocessing ##################
#############################################################################

manager = Manager()
_q = manager.dict()
_qd = manager.dict()

_global_flag = manager.dict()
_task_flag = manager.dict()

_q['x']=0.0; _q['y']=0.0; _q['th']=0.0; _q['v']=0.0; _q['w']=0.0;
_q['x0']=0.0; _q['y0']=0.0; _q['th0']=0.0; _q['t']=0.0;
_qd['x']=0.0; _qd['y']=0.0; _qd['th']=0.0; _qd['v']=0.0; _qd['w']=0.0; _qd['dv']=0.0; _qd['dw']=0.0; _qd['ddv']=0.0; _qd['ddw']=0.0; _qd['sv']=0.0; _qd['sw']=0.0;

_qd['xd_itp']=[0.0]*(horizon_samples+1)
_qd['yd_itp']=[0.0]*(horizon_samples+1)
_qd['thd_itp']=[0.0]*(horizon_samples+1)
_qd['vd_itp']=[0.0]*(horizon_samples+1)
_qd['wd_itp']=[0.0]*(horizon_samples+1)
_qd['dvd_itp']=[0.0]*(horizon_samples+1)
_qd['dwd_itp']=[0.0]*(horizon_samples+1)

_global_flag['MPC_Fail'] = False; _global_flag['OCP_Solved'] = False; _global_flag['Interpolated'] = False;
_global_flag['initial_data'] = False; _global_flag['isArrived'] = False; _global_flag['base_flag'] = False; 
_global_flag['buf_mobile']=3;

_task_flag['Task_Transition'] = False;
_task_flag['Task_Robot'] = 0


def base_pose_CB(data):
    # if _global_flag['initial_data']==True:
    #     # compensate odom offset
    #     quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    #     _q['th'] = tf.transformations.euler_from_quaternion(quaternion)[2] - _q['th0']
    #     _q['x'] = (data.pose.pose.position.x-_q['x0'])*cs.cos(_q['th0']) + (data.pose.pose.position.y-_q['y0'])*cs.sin(_q['th0'])
    #     _q['y'] = -(data.pose.pose.position.x-_q['x0'])*cs.sin(_q['th0']) + (data.pose.pose.position.y-_q['y0'])*cs.cos(_q['th0'])
        
    # # init for odom position
    # else:
    #     quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    #     _q['th0'] = tf.transformations.euler_from_quaternion(quaternion)[2]
    #     _q['x0'] = data.pose.pose.position.x
    #     _q['y0'] = data.pose.pose.position.y
    #     _global_flag['initial_data']=True
    pass
    
def base_twist_CB(data):
    _q['v'] = data.twist.twist.linear.x
    _q['w'] = data.twist.twist.angular.z

    if _global_flag['initial_data']==True:
        # compensate odom offset
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        quat_err = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(tf.transformations.quaternion_from_euler(0,0,_q['th0'])),quaternion)
        _q['th'] = tf.transformations.euler_from_quaternion(quat_err)[2]
        _q['x'] = (data.pose.pose.position.x-_q['x0'])*cs.cos(_q['th0']) + (data.pose.pose.position.y-_q['y0'])*cs.sin(_q['th0'])
        _q['y'] = -(data.pose.pose.position.x-_q['x0'])*cs.sin(_q['th0']) + (data.pose.pose.position.y-_q['y0'])*cs.cos(_q['th0'])
        
    # init for odom position
    else:
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        _q['th0'] = tf.transformations.euler_from_quaternion(quaternion)[2]
        _q['x0'] = data.pose.pose.position.x
        _q['y0'] = data.pose.pose.position.y
        _global_flag['initial_data']=True


def cmd_run():

    rospy.init_node('cona_mpc', anonymous=True)

    if command_activate == True:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
    else:
        pub = rospy.Publisher('/cmd_virtual', Twist, queue_size=50)

    mpc_pub = rospy.Publisher('/mpc_res', Float64MultiArray, queue_size=10)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, base_pose_CB)
    rospy.Subscriber("/odom", Odometry, base_twist_CB)

    mpc_res = Float64MultiArray()
    base_msg = Twist()
    base_frq = 50
    
    rate = rospy.Rate(base_frq)

    xd_itp_new = []; yd_itp_new = []; thd_itp_new=[];
    vd_itp_new = []; wd_itp_new = []; 
    dvd_itp_new = []; dwd_itp_new = []; 

    u_prev = [0.0]*2

    mpc_res.data = [0.0]*9
    #Kp_mob = [1, 1]
    #Ki_mob = [Kp_mob[0]**2/4, Kp_mob[0]**2/4, Kp_mob[0]**2/4]
    
    Kp_mob = [0.1,0.1]
    Ki_mob = [0.5,0.5,0.5]

    while not rospy.is_shutdown():

        init_time=time.time()

        if _global_flag['OCP_Solved'] == True:
            
            t_itp = np.linspace(0,duration, num=horizon_samples+1, endpoint=True)
            t_itp_new = np.linspace(0,duration, num=int(base_frq*t_mpc)*(horizon_samples)+1, endpoint=True)
            
            x_f = interp1d(t_itp, _qd['xd_itp'], kind='linear')
            y_f = interp1d(t_itp, _qd['yd_itp'], kind='linear')
            th_f = interp1d(t_itp, _qd['thd_itp'], kind='linear')

            v_f = interp1d(t_itp, _qd['vd_itp'], kind='linear')
            w_f = interp1d(t_itp, _qd['wd_itp'], kind='linear')
            
            dv_f = interp1d(t_itp, _qd['dvd_itp'], kind='linear')
            dw_f = interp1d(t_itp, _qd['dwd_itp'], kind='linear')

            xd_itp_new = list(x_f(t_itp_new))
            yd_itp_new = list(y_f(t_itp_new))
            thd_itp_new = list(th_f(t_itp_new))

            vd_itp_new = list(v_f(t_itp_new))
            wd_itp_new = list(w_f(t_itp_new))

            dvd_itp_new = list(dv_f(t_itp_new))
            dwd_itp_new = list(dw_f(t_itp_new))

            # xd_itp_new = _qd['xd_itp']
            # yd_itp_new = _qd['yd_itp']
            # thd_itp_new = _qd['thd_itp']
            # vd_itp_new = _qd['vd_itp']
            # wd_itp_new = _qd['wd_itp']
            mpc_res.data = [_q['t'],_qd['x'], _qd['y'], _qd['th'],
                        _qd['v'], _qd['w'],
                        _q['x'],_q['y'],_q['th'],
                        _qd['dv'], _qd['dw']]
                        #_qd['ddv'], _qd['ddw'],
                        #_qd['sv'], _qd['sw']]

            # mpc_res.data = [_q['t']]
            mpc_pub.publish(mpc_res)
            _global_flag['OCP_Solved'] = False

        # When infeasible occuls, to make Vel & Acc zero
        if len(vd_itp_new) == 0:
            base_msg.linear.x = 0
            base_msg.linear.y = 0
            base_msg.linear.z = 0

            base_msg.angular.x = 0
            base_msg.angular.y = 0
            base_msg.angular.z = 0
        else:
            x=Ki_mob[0]*(xd_itp_new.pop(0)-_q['x'])
            y=Ki_mob[1]*(yd_itp_new.pop(0)-_q['y'])
            quat_d=tf.transformations.quaternion_from_euler(0,0,thd_itp_new.pop(0))
            quat = tf.transformations.quaternion_from_euler(0,0,_q['th'])
            quat_err = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(quat),quat_d)
            th=Ki_mob[2]*(tf.transformations.euler_from_quaternion(quat_err)[2])
            
            J=np.array([[0,1],[cs.cos(_q['th']),0],[cs.sin(_q['th']),0]])
            v=np.linalg.pinv(J)@[th,x,y] # v=[v,w]' 
            
            vd = vd_itp_new.pop(0)
            dvd = dvd_itp_new.pop(0)
            #base_msg.linear.x = u_prev[0] + (dvd + v[0] + Kp_mob[0]*(vd-_q['v']))/base_frq
            #base_msg.linear.x = vd + v[0] + Kp_mob[0]*(vd-_q['v'])
            base_msg.linear.x = v[0] + vd
            #base_msg.linear.x = vd
            base_msg.linear.y = 0
            base_msg.linear.z = 0

            wd = wd_itp_new.pop(0)
            dwd = dwd_itp_new.pop(0)
            base_msg.angular.x = 0
            base_msg.angular.y = 0
            #base_msg.angular.z = u_prev[1] + (dwd + v[1] + Kp_mob[1]*(wd-_q['w']))/base_frq
            #base_msg.angular.z = wd + v[1] + Kp_mob[1]*(wd-_q['w'])
            base_msg.angular.z = v[1] + wd
            #base_msg.angular.z = wd
            # print(base_msg)

        u_prev=[base_msg.linear.x, base_msg.angular.z]
        pub.publish(base_msg)

        rate.sleep()



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
    jq0_val = [0.0, 0.0]

    # Update robot's parameters if needed
    max_task_vel_ub = cs.DM([1.2, pi/6])
    max_task_vel_lb = cs.DM([0, -pi/6])
    max_task_acc = cs.DM([10, 10*pi])
    max_task_jerk = cs.DM([80,80*pi])
    max_task_snap = cs.DM([1000,1000*pi])
    robot.set_task_velocity_limits(lb=max_task_vel_lb, ub=max_task_vel_ub)
    robot.set_task_acceleration_limits(lb=-max_task_acc, ub=max_task_acc)
    robot.set_task_jerk_limits(lb=-max_task_jerk, ub=max_task_jerk)
    robot.set_task_snap_limits(lb=-max_task_snap, ub=max_task_snap)

    ################################################
    # Task spacification - Approximation to object
    ################################################

    if time_optimal:
        tc = tp.task_context(horizon_steps = horizon_samples, time_init_guess=horizon_samples*t_mpc)
    else:
        tc = tp.task_context(time= horizon_samples*t_mpc, horizon_steps = horizon_samples)

    x_0, y_0, th_0, v_0, w_0, dv_0, dw_0, ddv_0, ddw_0, sv_0, sw_0, x0_0, y0_0, th0_0, v0_0, w0_0, dv0_0, dw0_0, ddv0_0, ddw0_0 = WMR(robot,tc, options={'nonholonomic':True, 'snap':True})

    # Minimal time
    # tc.minimize_time(10, 0)

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
    tc.add_regularization(expression = v_0, weight = 5e0, reference=0.3, stage = 0)
    tc.add_regularization(expression = w_0, weight = 5e0, reference=0.5, stage = 0)

    tc.add_regularization(expression = dv_0, weight = 4e0, stage = 0)
    tc.add_regularization(expression = dw_0, weight = 4e0, stage = 0)

    tc.add_regularization(expression = ddv_0, weight = 1e0, stage = 0)
    tc.add_regularization(expression = ddw_0, weight = 1e0, stage = 0)

    tc.add_regularization(expression = sv_0, weight = 1e-1, stage = 0)
    tc.add_regularization(expression = sw_0, weight = 1e-1, stage = 0)

    # Path_constraint
    path_pos1 = {'hard':False, 'expression':x_0, 'reference':waypoints[0], 'gain':4e2, 'norm':'L2'}
    path_pos2 = {'hard':False, 'expression':y_0, 'reference':waypoints[1], 'gain':4e2, 'norm':'L2'}
    path_pos3 = {'hard':False, 'expression':th_0, 'reference':waypoints[2], 'gain':4e2, 'norm':'L2'}
    tc.add_task_constraint({"path_constraints":[path_pos1, path_pos2, path_pos3]}, stage = 0)

    # fina
    final_pos = {'hard':False, 'expression':p, 'reference':waypoint_last, 'gain':4e2, 'norm':'L2'}
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
    tc.set_value(ddv0_0, jq0_val[0], stage=0)
    tc.set_value(ddw0_0, jq0_val[1], stage=0)

    # Initial value for control inputs
    tc.set_initial(sv_0, 0, stage=0)
    tc.set_initial(sw_0, 0, stage=0)

    # # Define reference path
    # pathpoints = 300
    # ref_path = {}
    # ref_path['x'] = 1.5*np.sin(np.linspace(0,4*np.pi, pathpoints+1))
    # ref_path['y'] = np.linspace(0,2, pathpoints+1)**2*2.5
    # theta_path = [cs.arctan2(ref_path['y'][k+1]-ref_path['y'][k], ref_path['x'][k+1]-ref_path['x'][k]) for k in range(pathpoints)] 
    # ref_path['theta'] = theta_path + [theta_path[-1]]

    # # Corridor
    # pathpoints = 100
    # ref_path = {}
    # ref_path['x'] = 0.5*np.sin(np.linspace(0,4*np.pi, pathpoints+1))
    # ref_path['y'] = np.linspace(0,2, pathpoints+1)**2*2.5
    # theta_path = [cs.arctan2(ref_path['y'][k+1]-ref_path['y'][k], ref_path['x'][k+1]-ref_path['x'][k]) for k in range(pathpoints)] 
    # ref_path['theta'] = theta_path + [theta_path[-1]]

    # # Straight line
    # pathpoints = 20
    # ref_path = {}
    # ref_path['x'] = np.linspace(0,1, pathpoints+1)
    # ref_path['y'] = np.linspace(0,0, pathpoints+1)
    # ref_path['theta'] = np.linspace(0,0, pathpoints+1)

    # # Box1
    # viapoints = 30
    # dist_box = 2
    # pathpoints = (viapoints+1)*3
    # ref_path = {}
    # a=np.linspace(0,dist_box, viapoints+1); b=np.linspace(dist_box,0, viapoints+1);
    # e_0=np.linspace(0,0,viapoints+1);e_b=np.linspace(dist_box,dist_box,viapoints+1);
    # a_r=np.linspace(pi/2,pi/2,viapoints+1);b_r=np.linspace(pi,pi,viapoints+1);
    # ref_path['x'] = np.concatenate((a,np.concatenate((e_b,b))))
    # ref_path['y'] = np.concatenate((e_0,np.concatenate((a,e_b))))
    # ref_path['theta'] = np.concatenate((e_0,np.concatenate((a_r,b_r))))

    # Box2
    viapoints = 15
    dist_box = 2
    pathpoints = (viapoints+1)*6
    ref_path = {}
    a=np.linspace(0,dist_box/2, viapoints+1); b=np.linspace(dist_box/2,dist_box, viapoints+1);
    c=np.linspace(dist_box,dist_box/2, viapoints+1); d=np.linspace(dist_box/2,0, viapoints+1);
    e_0=np.linspace(0,0,viapoints+1);e_m=np.linspace(dist_box/2,dist_box/2,viapoints+1);e_b=np.linspace(dist_box,dist_box,viapoints+1);
    a_r=np.linspace(pi/2,pi/2,viapoints+1);b_r=np.linspace(pi,pi,viapoints+1);

    am=np.concatenate((a,e_m)); ea=np.concatenate((e_0,a));
    bm=np.concatenate((b,e_b)); mb=np.concatenate((e_m,b)); 
    cd=np.concatenate((c,d)); bb=np.concatenate((e_b,e_b)); 
    ear=np.concatenate((e_0,a_r)); br=np.concatenate((b_r,b_r))


    ref_path['x'] = np.concatenate((am,np.concatenate((bm,cd))))
    ref_path['y'] = np.concatenate((ea,np.concatenate((mb,bb))))
    ref_path['theta'] = np.concatenate((ear,np.concatenate((ear,br))))
    
    if obstacle_avoidance==True:
        ref_obs = {}
        ref_obs['x'] = np.array([0.2, 0.0])
        ref_obs['y'] = np.array([2.5, 6])

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
        if dist_obs>=5:
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
    tc.tc_dict["out_ports"].append({"name":"port_out_ddv0", "var":"ddv0", "desc": "output port for the task linear acceleration"})
    tc.tc_dict["out_ports"].append({"name":"port_out_ddw0", "var":"ddw0", "desc": "output port for the task angular acceleration"})

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
    sol_options = {"ipopt": {"linear_solver": "ma27","print_level": 0}}
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
    dir_casadi_func = "casadi"
    os.makedirs("../lib/"+dir_casadi_func, exist_ok=True)
    cg_opts={"ocp_cg_opts":{"save":True, "codegen":False, "jit":False}, "mpc":True, "mpc_cg_opts":{"save":True, "codegen":False, "jit":False}}
    vars_db = tc.generate_MPC_component("../lib/"+dir_casadi_func+"/", cg_opts)
    print("../lib/"+dir_casadi_func+"/"+ tc.name + ".json")
    MPC_component = MPC("mir250_path_following", "../lib/"+dir_casadi_func+"/"+ tc.name + ".json")

    ################################################
    # MPC Simulation
    ################################################

    # Create world simulator based on pybullet
    obj = WorldSimulator.WorldSimulator(bullet_gui=gui_enable)
    obj.visualization_realtime=False
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
    t_sol, ddv_0_sol = tc.sol_sample(ddv_0, grid="control", stage = 0)
    t_sol, ddw_0_sol     = tc.sol_sample(ddw_0, grid="control",     stage = 0)
    t_sol, sv_0_sol = tc.sol_sample(sv_0, grid="control", stage = 0)
    t_sol, sw_0_sol     = tc.sol_sample(sw_0, grid="control",     stage = 0)

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

    x_pred = [0]*(horizon_samples+1)
    y_pred = [0]*(horizon_samples+1)
    th_pred = [0]*(horizon_samples+1)

    v_pred = [0]*(horizon_samples+1)
    w_pred = [0]*(horizon_samples+1)

    dv_pred = [0]*(horizon_samples+1)
    dw_pred = [0]*(horizon_samples+1)

    ddv_pred = [0]*(horizon_samples+1)
    ddw_pred = [0]*(horizon_samples+1)

    sv_pred = [0]*(horizon_samples)
    sw_pred = [0]*(horizon_samples)

    q_pred = [0]*(horizon_samples+1)


    cnt=0
    glob_time=time.time()
    glob_time_buf=0
    init_time_buf=0
    loop_time = 0
    dvd_control_sig = 0; dwd_control_sig = 0
    while True:
        init_time=time.time()
        print("----------- MPC execution -----------")
        if cnt==0:
            # initial guess control input
            _qd['xd_itp']=x_0_sol
            _qd['yd_itp']=y_0_sol
            _qd['thd_itp']=th_0_sol
            _qd['vd_itp']=v_0_sol
            _qd['wd_itp']=w_0_sol
            _qd['dvd_itp']=dv_0_sol
            _qd['dwd_itp']=dw_0_sol

            _global_flag['OCP_Solved'] = True
            
            _qd['x']=x_0_sol[1]; _qd['y']=y_0_sol[1]; _qd['th']=th_0_sol[1];
            _qd['v']=v_0_sol[1]; _qd['w']=w_0_sol[1]; 
            _qd['dv']=dv_0_sol[1]; _qd['dw']=dw_0_sol[1];
            _qd['ddv']=ddv_0_sol[1]; _qd['ddw']=ddw_0_sol[1];
            _qd['sv']=sv_0_sol[1]; _qd['sw']=sw_0_sol[1];

        else:
            print("loop time: %f [ms]"%(1000*(loop_time)))
            start = time.time()
            # q_now = obj.readJointPositions(robotID, joint_indices)
            # dq_now = obj.readJointVelocities(robotID, joint_indices)

            # initialize values
            # MPC_component.input_ports["port_inp_x00"]["val"] = q_now[0]
            # MPC_component.input_ports["port_inp_y00"]["val"] = q_now[1]
            # MPC_component.input_ports["port_inp_th00"]["val"] = q_now[2]
            # MPC_component.input_ports["port_inp_v00"]["val"] = np.sqrt(dq_now[0]**2+dq_now[1]**2)
            # MPC_component.input_ports["port_inp_w00"]["val"] = dq_now[2]
            # MPC_component.input_ports["port_inp_dv00"]["val"] = dvd_control_sig
            # MPC_component.input_ports["port_inp_dw00"]["val"] = dwd_control_sig

            MPC_component.input_ports["port_inp_x00"]["val"] = _qd['x']
            MPC_component.input_ports["port_inp_y00"]["val"] = _qd['y']
            MPC_component.input_ports["port_inp_th00"]["val"] = _qd['th']
            MPC_component.input_ports["port_inp_v00"]["val"] = _qd['v']
            MPC_component.input_ports["port_inp_w00"]["val"] = _qd['w']
            MPC_component.input_ports["port_inp_dv00"]["val"] = _qd['dv']
            MPC_component.input_ports["port_inp_dw00"]["val"] = _qd['dw']
            MPC_component.input_ports["port_inp_ddv00"]["val"] = _qd['ddv']
            MPC_component.input_ports["port_inp_ddw00"]["val"] = _qd['ddw']

            # Find closest point on the reference path compared witch current position
            # index_closest_point = find_closest_point(q_now[:2], ref_path, index_closest_point)
            index_closest_point = find_closest_point([_qd['x'],_qd['y']], ref_path, index_closest_point)

            # Create a list of N waypoints
            current_waypoints = get_current_waypoints(index_closest_point, wp, horizon_samples, dist=5)

            if obstacle_avoidance==True:
                # Set initial switch value for obstacle avoidance
                # dist_obs, closest_obs = find_closest_obstacle(q_now[:2], ref_obs)
                dist_obs, closest_obs = find_closest_obstacle([_qd['x'],_qd['y']], ref_obs)
                MPC_component.input_ports["port_inp_obs_p"]["val"] = [ref_obs['x'][closest_obs],ref_obs['y'][closest_obs]]
                if dist_obs>=5:
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

            sol = MPC_component.res_vals
            for i in range(horizon_samples+1):
                # 1st eliment: current value, 2nd~horizon: predicted value
                x_pred[i]=sol[i].full()[0][0]
                y_pred[i]=sol[(horizon_samples+1)+i].full()[0][0]
                th_pred[i]=sol[2*(horizon_samples+1)+i].full()[0][0]

                v_pred[i]=sol[3*(horizon_samples+1)+i].full()[0][0]
                w_pred[i]=sol[4*(horizon_samples+1)+i].full()[0][0]

                dv_pred[i]=sol[5*(horizon_samples+1)+i].full()[0][0]
                dw_pred[i]=sol[6*(horizon_samples+1)+i].full()[0][0]

                ddv_pred[i]=sol[7*(horizon_samples+1)+i].full()[0][0]
                ddw_pred[i]=sol[8*(horizon_samples+1)+i].full()[0][0]
                
                if i != horizon_samples:
                    sv_pred[i]=sol[9*(horizon_samples+1)+i].full()[0][0]
                    sw_pred[i]=sol[10*(horizon_samples+1)+i].full()[0][0]

                q_pred[i] = [x_pred[i], y_pred[i], th_pred[i]]

            if frame_enable==True:
                obj.resetMultiJointState(frameIDs, joint_indices, q_pred)

            _qd['xd_itp']=x_pred
            _qd['yd_itp']=y_pred
            _qd['thd_itp']=th_pred
            _qd['vd_itp']=v_pred
            _qd['wd_itp']=w_pred
            _qd['dvd_itp']=dv_pred
            _qd['dwd_itp']=dw_pred
            _global_flag['OCP_Solved'] = True

            # Set control signal to the simulated robot
            xd_control_sig = MPC_component.output_ports["port_out_x0"]["val"].full()
            yd_control_sig = MPC_component.output_ports["port_out_y0"]["val"].full()
            thd_control_sig = MPC_component.output_ports["port_out_th0"]["val"].full()
            vd_control_sig = MPC_component.output_ports["port_out_v0"]["val"].full()
            wd_control_sig = MPC_component.output_ports["port_out_w0"]["val"].full()
            dvd_control_sig = (MPC_component.output_ports["port_out_dv0"]["val"]).full()
            dwd_control_sig = (MPC_component.output_ports["port_out_dw0"]["val"]).full()
            ddvd_control_sig = (MPC_component.output_ports["port_out_ddv0"]["val"]).full()
            ddwd_control_sig = (MPC_component.output_ports["port_out_ddw0"]["val"]).full()
            svd_control_sig = (MPC_component.output_ports["port_out_sv0"]["val"]).full()
            swd_control_sig = (MPC_component.output_ports["port_out_sw0"]["val"]).full()

            _qd['x']=x_pred[1]; _qd['y']=y_pred[1]; _qd['th']=th_pred[1];
            _qd['v']=v_pred[1]; _qd['w']=w_pred[1]; 
            _qd['dv']=dv_pred[1]; _qd['dw']=dw_pred[1];
            _qd['ddv']=ddv_pred[1]; _qd['ddw']=ddw_pred[1];
            _qd['sv']=sv_pred[0]; _qd['sw']=sw_pred[0];
            # twist_d = [(vd_control_sig+dvd_control_sig)*np.cos(thd_control_sig), (vd_control_sig+dvd_control_sig)*np.sin(thd_control_sig),wd_control_sig+dwd_control_sig]
            # twist_d = [(vd_control_sig)*np.cos(thd_control_sig), (vd_control_sig)*np.sin(thd_control_sig),wd_control_sig]
            twist_d = [v_pred[1]*np.cos(th_pred[1]), v_pred[1]*np.sin(th_pred[1]), w_pred[1]]
            # print(twist_d)
            obj.setController(
                robotID, "velocity", joint_indices, targetVelocities=twist_d
            )
            # print("v: ", vd_control_sig, ", w: ", wd_control_sig)
            print("v: ", v_pred[1], ", w: ", w_pred[1])
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
        _q['t']=1000*loop_time
        init_time_buf=init_time

    obj.end_simulation()



#############################################################################################################################

if __name__=='__main__':
    mpc_task = Process(target=mpc_run, args=())
    cmd_task = Process(target=cmd_run, args=())
    try:
        mpc_task.start()
        cmd_task.start()

        mpc_task.join()
        cmd_task.join()

    except KeyboardInterrupt:
        mpc_task.terminate()
        cmd_task.terminate()
