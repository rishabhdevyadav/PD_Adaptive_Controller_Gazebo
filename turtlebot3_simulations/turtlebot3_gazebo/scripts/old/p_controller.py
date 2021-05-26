#!/usr/bin/env python
# coding: utf-8

import numpy as np
import socket, time

import cubic_spline_planner
old_nearest_point_index = None

import tf

from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Twist
import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from tf.transformations import *
from gazebo_msgs.msg import ModelStates



class Controller:
    # initialization method
    def __init__(self):

    	self.x_1, self.y_1, self.z_1 = 0, 0, 0
    	self.vx_1, self.vy_1, self.vz_1 = 0, 0, 0
        self.wx_1, self.wy_1, self.wz_1 = 0, 0, 0
    	self.q0_1, self.q1_1, self.q2_1, self.q3_1 = 0, 0, 0, 0

        self.x_2, self.y_2, self.z_2 = 0, 0, 0
        self.vx_2, self.vy_2, self.vz_2 = 0, 0, 0
        self.wx_2, self.wy_2, self.wz_2 = 0, 0, 0
        self.q0_2, self.q1_2, self.q2_2, self.q3_2 = 0, 0, 0, 0

        self.x_3, self.y_3, self.z_3 = 0, 0, 0
        self.vx_3, self.vy_3, self.vz_3 = 0, 0, 0
        self.wx_3, self.wy_3, self.wz_3 = 0, 0, 0
        self.q0_3, self.q1_3, self.q2_3, self.q3_3 = 0, 0, 0, 0

    	self.roll1, self.pitch1, self.yaw1 = 0,0,0 #current roll, pitch, yaw
        self.roll2, self.pitch2, self.yaw2 = 0,0,0 #current roll, pitch, yaw
        self.roll3, self.pitch3, self.yaw3 = 0,0,0 #current roll, pitch, yaw

        self.cmd_pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.cmd_pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        self.cmd_pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)

        self.twist1 = Twist()
        self.twist2 = Twist()
        self.twist3 = Twist()

        self.robotpose1, self.robotpose2, self.robotpose3 = None,None, None
        self.robottwist1,self.robottwist2,self.robottwist3  = None,None, None

        self.old_nearest_point_index = None


    def gazeboStateCb(self, msg):

        idx1 = msg.name.index('tb3_1')
        idx2 = msg.name.index('tb3_2')
        idx3 = msg.name.index('tb3_3')

        self.robotpose1 = msg.pose[idx1]
        self.robottwist1 = msg.twist[idx1]
        self.robotpose2 = msg.pose[idx2]
        self.robottwist2 = msg.twist[idx2]
        self.robotpose3 = msg.pose[idx3]
        self.robottwist3 = msg.twist[idx3]

        self.x_1 = self.robotpose1.position.x
        self.y_1 = self.robotpose1.position.y
        self.z_1 = self.robotpose1.position.z
        self.q0_1 = self.robotpose1.orientation.w
        self.q1_1 = self.robotpose1.orientation.x
        self.q2_1 = self.robotpose1.orientation.y
        self.q3_1 = self.robotpose1.orientation.z
        self.vx_1 = self.robottwist1.linear.x
        self.vy_1 = self.robottwist1.linear.y
        self.vz_1 = self.robottwist1.linear.z
        self.wx_1 = self.robottwist1.angular.x
        self.wy_1 = self.robottwist1.angular.y
        self.wz_1 = self.robottwist1.angular.z

        self.x_2 = self.robotpose2.position.x
        self.y_2 = self.robotpose2.position.y
        self.z_2 = self.robotpose2.position.z
        self.q0_2 = self.robotpose2.orientation.w
        self.q1_2 = self.robotpose2.orientation.x
        self.q2_2 = self.robotpose2.orientation.y
        self.q3_2 = self.robotpose2.orientation.z
        self.vx_2 = self.robottwist2.linear.x
        self.vy_2 = self.robottwist2.linear.y
        self.vz_2 = self.robottwist2.linear.z
        self.wx_2 = self.robottwist2.angular.x
        self.wy_2 = self.robottwist1.angular.y
        self.wz_2 = self.robottwist2.angular.z

        self.x_3 = self.robotpose3.position.x
        self.y_3 = self.robotpose3.position.y
        self.z_3 = self.robotpose3.position.z
        self.q0_3 = self.robotpose3.orientation.w
        self.q1_3 = self.robotpose3.orientation.x
        self.q2_3 = self.robotpose3.orientation.y
        self.q3_3 = self.robotpose3.orientation.z
        self.vx_3 = self.robottwist3.linear.x
        self.vy_3 = self.robottwist3.linear.y
        self.vz_3 = self.robottwist3.linear.z
        self.wx_3 = self.robottwist3.angular.x
        self.wy_3 = self.robottwist1.angular.y
        self.wz_3 = self.robottwist3.angular.z

        orientation_list1 = [self.q1_1, self.q2_1, self.q3_1, self.q0_1]
        (self.roll1, self.pitch1, self.yaw1) = euler_from_quaternion(orientation_list1)

        orientation_list2 = [self.q1_2, self.q2_2, self.q3_2, self.q0_2]
        (self.roll2, self.pitch2, self.yaw2) = euler_from_quaternion(orientation_list2)

        orientation_list3 = [self.q1_3, self.q2_3, self.q3_3, self.q0_3]
        (self.roll3, self.pitch3, self.yaw3) = euler_from_quaternion(orientation_list3)


    def cmd_velocity(self, vx1, wz1, vx2, wz2, vx3, wz3):

        # print(vx1, wz1, vx2, wz2, vx3, wz3)

        self.twist1.linear.x = vx1
        self.twist1.linear.y = 0.0
        self.twist1.linear.z = 0.0
        self.twist1.angular.x = 0.0
        self.twist1.angular.y = 0.0 
        self.twist1.angular.z = wz1

        self.twist2.linear.x = vx2
        self.twist2.linear.y = 0.0
        self.twist2.linear.z = 0.0
        self.twist2.angular.x = 0.0
        self.twist2.angular.y = 0.0 
        self.twist2.angular.z = wz2

        self.twist3.linear.x = vx3
        self.twist3.linear.y = 0.0
        self.twist3.linear.z = 0.0
        self.twist3.angular.x = 0.0
        self.twist3.angular.y = 0.0 
        self.twist3.angular.z = wz3

    def pub_vel(self, vx1, wz1, vx2, wz2, vx3, wz3):
        self.cmd_velocity(vx1, wz1, vx2, wz2, vx3, wz3)
        self.cmd_pub1.publish(self.twist1)
        self.cmd_pub2.publish(self.twist2)
        self.cmd_pub3.publish(self.twist3)


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega=omega

# def agent_state_update(state,v_in,omega_in):
#     state.x = state.x + v_in * math.cos(state.yaw) * dt
#     state.y = state.y + v_in * math.sin(state.yaw) * dt
#     state.yaw = pi_2_pi(state.yaw + omega_in*dt)
#     state.v = v_in
#     state.omega = omega_in
#     return state

def update_state_gazebo(state, x, y, yaww,  v_in, omega_in):
    state.x = x
    state.y = y
    state.yaw = yaww
    state.yaw = pi_2_pi(state.yaw)
    state.v = v_in
    state.omega = omega_in
    return state

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calc_target_index(state, cx, cy):
    global old_nearest_point_index
    if old_nearest_point_index is None:
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        old_nearest_point_index = ind
    else:
        ind = old_nearest_point_index
        distance_this_index = calc_distance(state, cx[ind], cy[ind])
        while True:
            ind = ind + 1 if (ind + 1) < len(cx) else ind
            distance_next_index = calc_distance(state, cx[ind], cy[ind])
            if distance_this_index < distance_next_index:
                break
            distance_this_index = distance_next_index
        old_nearest_point_index = ind
    L = 0.0
    k = 0.03  # look forward gain
    Lfc = 0.2  # look-ahead distance
    Lf = k * state.v + Lfc
    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind] - state.x
        dy = cy[ind] - state.y
        L = math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    return ind 

def calc_distance(state, point_x, point_y):
    dx = state.x - point_x
    dy = state.y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)

def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    return ind


def agent_angSpeed(state,cx,cy,target_ind):
    yaw_new = (np.rad2deg(pi_2_pi(state.yaw)))
    if yaw_new < 0:
        yaw_new = yaw_new + 360
    dy = -state.y + cy[target_ind] 
    dx = -state.x + cx[target_ind]

    theta = np.rad2deg(pi_2_pi(math.atan2(dy, dx)))
    if theta < 0:
        theta = theta + 360

    error = (theta - yaw_new)
    if error > 180:
        error = error - 360
    if error <- 180:
        error = error + 360

    omega = 0.005*error
    return omega

def search_index(master_ind, master_state, cx, cy, gap):
    i,d = master_ind, 0
    while (d < (gap)):   #0.5 is small threshold for smooth working
        i = i - 1
        d = np.sqrt( np.square(master_state.x - cx[i]) + np.square(master_state.y - cy[i]))
    # print(d)
    return i

def dist_error(L_state, F_state, targDist):
    dist = np.sqrt( np.square(L_state.x - F_state.x) + np.square(L_state.y - F_state.y))
    error = (targDist - dist)
    return error

def agent_linSpeed(v_d, L_error, F_error):
    v_in = v_d
    if np.absolute(L_error) > 0.05:
            v_in = v_d - (0.005*L_error)
    if np.absolute(F_error) > 0.05:
            v_in = v_d + (0.005*F_error)
    return v_in


def main():

    rospy.init_node('setpoint_node', anonymous=True)
    cnt = Controller()
    rate = rospy.Rate(10.0)
    rospy.Subscriber('/gazebo/model_states', ModelStates, cnt.gazeboStateCb)

    time.sleep(2)

    ax = [-3,-2,-1,0,  2,4,6,8,10,12,14,16]
    ay = [0, 0, 0, 0,  1,0,-1,0,0,0,0,0 ]
    
    goal = [ax[-1], ay[-1]]
    cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    gap = 1

    state1 = State(x=0, y=0, yaw=0, v=0,omega=0)
    state2 = State(x=-1, y=0, yaw=0, v=0,omega=0)
    state3 = State(x=-2, y=0, yaw=0, v=0,omega=0)

    v_ref = 0.15

    while not rospy.is_shutdown():

        target_ind1 = calc_target_index(state1, cx, cy)     

        target_ind2 = search_index(target_ind1, state1, cx, cy, gap)
        nearest_ind2 = calc_nearest_index(state2, cx, cy, cyaw)

        # time.sleep(.1)

        target_ind3 = search_index(target_ind2, state2, cx, cy, gap)
        nearest_ind3 = calc_nearest_index(state3, cx, cy, cyaw)

        # time.sleep(.1)

        err12 = dist_error(state1, state2, gap)
        err23 = dist_error(state2, state3, gap)


        v_in1 = agent_linSpeed(v_ref, 0, err12)
        omega1 = agent_angSpeed(state1,cx,cy,target_ind1)
        state1 = update_state_gazebo(state1, cnt.x_1, cnt.y_1, cnt.yaw1, v_in1, omega1)

        # time.sleep(.1)

        v_in2 = agent_linSpeed(v_ref, err12, err23)
        omega2 = agent_angSpeed(state2,cx,cy,target_ind2)
        if target_ind2 - nearest_ind2 < 3:
            v_in2 = 0
            omega2 = 0
            # print(target_ind2 - nearest_ind2)
        state2 = update_state_gazebo(state2, cnt.x_2, cnt.y_2, cnt.yaw2, v_in2, omega2)

        # time.sleep(.1)

        
        v_in3 = agent_linSpeed(v_ref, err23, 0)
        omega3 = agent_angSpeed(state3,cx,cy,target_ind3)
        if target_ind3 - nearest_ind3 < 3:
            v_in3 = 0
            omega3 = 0
            # print(target_ind3 - nearest_ind3)
        state3 = update_state_gazebo(state3, cnt.x_3, cnt.y_3, cnt.yaw3, v_in3, omega3)


        # time.sleep(.5)

        # print(v_in1)

        # print(v_in1, omega1, v_in2, omega2, v_in3, omega3)
        # print(target_ind2, nearest_ind2)
        print(target_ind1, target_ind2, target_ind3)
        # print("------Running---------")

        time.sleep(1.1)
        # rate.sleep()

        cnt.pub_vel(v_in1, omega1,
                    v_in2, omega2,
                    v_in3, omega3 )


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
