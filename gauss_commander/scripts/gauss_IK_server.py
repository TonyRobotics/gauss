#!/usr/bin/env python
"""
ROS node for Inverse Kinematic analysis of the Gauss6-500 robot arm.
"""

import tf
import rospy
from gauss_msgs.srv import *
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from numpy import array, matrix, cos, sin, pi, arccos, arctan2, sqrt, sign
from numpy.linalg import inv

# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib import pyplot as plt


def get_DH_Table():
    """
    Define DH parameters for Gauss6-500.
    alphai-1 :  angle b/w z-axes of links i-1 and i along x-axis of link i-1
    ai-1     :  distance b/w z-axes of links i-1 and i along x-axis of link i-1
    di       :  distance b/w x-axes of links i-1 and i along z-axis of link i
    thetai   :  angle b/w x-axes of links i-1 and i along z-axis of link i
    """
    # Define variables for joint angles
    theta1, theta2, theta3, theta4, theta5, theta6 = 0., 0., 0., 0., 0., 0.
    # Construct DH Table of Gauss6-500
    dh = {'alpha0':     0,  'a0':      0,  'd1': 0.2505,  'theta1':   theta1,
          'alpha1':  pi/2,  'a1':      0,  'd2':      0,  'theta2':   theta2,
          'alpha2':     0,  'a2':  0.185,  'd3':      0,  'theta3':   theta3,
          'alpha3':  pi/2,  'a3':      0,  'd4':  0.252,  'theta4':   theta4,
          'alpha4': -pi/2,  'a4':      0,  'd5':      0,  'theta5':   theta5,
          'alpha5':  pi/2,  'a5':      0,  'd6':      0,  'theta6':   theta6,
          'alpha6':     0,  'a6':      0,  'dG':  0.025,  'thetaG':        0}
    
    return dh


def get_Rx(theta):
    """Define matrix for rotation (roll) about x axis."""
    Rx = matrix([[1,          0,           0],
                 [0, cos(theta), -sin(theta)],
                 [0, sin(theta),  cos(theta)]])
    return Rx


def get_Ry(theta):
    """Define matrix for rotation (pitch) about y axis."""
    Ry = matrix([[cos(theta),  0, sin(theta)],
                 [         0,  1,          0],
                 [-sin(theta), 0, cos(theta)]])
    return Ry


def get_Rz(theta):
    """Define matrix for rotation (yaw) about z axis."""
    Rz = matrix([[cos(theta), -sin(theta), 0],
                 [sin(theta),  cos(theta), 0],
                 [         0,           0, 1]])
    return Rz


def get_TF(alpha, a, d, theta):
    """Define matrix for homogeneous transforms between adjacent links."""
    Tf = matrix([
        [           cos(theta),            -sin(theta),            0,              a],
        [sin(theta)*cos(alpha),  cos(theta)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
        [sin(theta)*sin(alpha),  cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d],
        [                    0,                      0,            0,              1]
     ])
    return Tf


def get_ee_pose(pose_msg):
    """
    Extract EE pose from received trajectory pose in an IK request message.
    NOTE: Pose is position (cartesian coords) and orientation (euler angles)
    """
    ee_x = pose_msg.position.x
    ee_y = pose_msg.position.y
    ee_z = pose_msg.position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [pose_msg.orientation.x, pose_msg.orientation.y,
         pose_msg.orientation.z, pose_msg.orientation.w]
        )
    #(roll, pitch, yaw) = (0.8744, -2.26, -1.3753)
    position = (ee_x, ee_y, ee_z)
    orientation = (roll, pitch, yaw)

    print "orientation  ~~~~~~~~~~~~~~~~~~~~~~~"
    print   pose_msg.orientation.x
    print   pose_msg.orientation.y
    print   pose_msg.orientation.z
    print   pose_msg.orientation.w

    print "r p y  ~~~~~~~~~~~~~~~~~~~~~~~"
    print  roll
    print  pitch
    print  yaw

    

    return position, orientation

def get_R_EE(ee_pose):
    """
    Compute EE Rotation matrix w.r.t base frame.
    Computed from EE orientation (roll, pitch, yaw) and describes the
    orientation of each axis of EE w.r.t the base frame
    """
    roll, pitch, yaw = ee_pose[1]
    # Perform extrinsic (fixed-axis) sequence of rotations of EE about
    # x, y, and z axes by roll, pitch, and yaw radians respectively
    R_ee = get_Rz(yaw) * get_Ry(pitch) * get_Rx(roll)
    # Align EE frames in URDF vs DH params through a sequence of
    # intrinsic (body-fixed) rotations: 180 deg yaw and -90 deg pitch
    Rerror = get_Rz(pi) * get_Ry(-pi/2)
    # Account for this frame alignment error in EE pose
    R_ee = R_ee * Rerror
    #print "==================================="
    #print "R_ee: "
    #print R_ee

    return R_ee


def get_WC(dh, R_ee, ee_pose):
    """
    Compute Wrist Center position (cartesian coords) w.r.t base frame.
    Keyword arguments:
    R_ee -- EE Rotation matrix w.r.t base frame
    ee_pose -- tuple of cartesian coords and euler angles describing EE
    Return values:
    Wc -- vector of cartesian coords of WC
    """
    ee_x, ee_y, ee_z = ee_pose[0]
    # Define EE position as a vector
    EE_P = matrix([[ee_x],
                   [ee_y],
                   [ee_z]])
    # Get Col3 vector from R_ee that describes z-axis orientation of EE
    Z_ee = R_ee[:, 2]
    # WC is a displacement from EE equal to a translation along
    # the EE z-axis of magnitude dG w.r.t base frame (Refer to DH Table)
    Wc = EE_P - dh['dG']*Z_ee
    #print "==================================="
    #print "Wc: "
    #print Wc

    return Wc


def get_joints1_2_3(dh, Wc):
    """
    Calculate joint angles 1,2,3 using geometric IK method.
    NOTE: Joints 1,2,3 control position of WC (joint 5)
    """
    # Define wrist joint limits
    theta1_lower_limit = -pi/2
    theta1_upper_limit =  pi/2
    theta2_lower_limit = -pi/2
    theta2_upper_limit =  pi/6
    theta3_lower_limit = -0.26179938
    theta3_upper_limit =  pi/2

    wcx, wcy, wcz = Wc[0], Wc[1], Wc[2]

    # theta1 is calculated by viewing joint 1 and arm from top-down
    theta1 = arctan2(wcy, wcx)
    if round(wcy, 7) == 0:
        theta1 = 0

    if theta1 < theta1_lower_limit or theta1 > theta1_upper_limit:
        print "**********************************"
        print "Invalid Pose, theta1 out of range!"
        print "**********************************"

    # theta2,3 are calculated using Cosine Law on a triangle with edges
    # at joints 1,2 and WC viewed from side and
    # forming angles A, B and C repectively
    wcz_j2 = wcz - dh['d1']                            # WC z-component from j2
    wcx_j2 = sqrt(wcx**2 + wcy**2)                     # WC x-component from j2
    #print "==================================="
    #print "wcz_j2, wcx_j2: %f, %f"%(wcz_j2, wcx_j2)

    side_a = dh['d4']                                       # line segment: j3-WC
    side_b = round(sqrt(wcx_j2**2 + wcz_j2**2), 7)          # line segment: j2-WC
    side_c = dh['a2']                                       # link length:  j2-j3
    #print "==================================="
    #print "side_a, side_b, side_c: %f, %f, %f"%(side_a, side_b, side_c)

    angleA = arccos((side_b**2 + side_c**2 - side_a**2) / (2*side_b*side_c))
    angleB = arccos((side_a**2 + side_c**2 - side_b**2) / (2*side_a*side_c))
    angleC = arccos((side_a**2 + side_b**2 - side_c**2) / (2*side_a*side_b))
    #print "==================================="
    #print "angleA, angleB, angleC: %f, %f, %f"%(angleA, angleB, angleC)

    theta2 = -pi/2 + angleA + arctan2(wcz_j2, wcx_j2)

    if theta2 < theta2_lower_limit or theta2 > theta2_upper_limit:
        print "**********************************"
        print "Invalid Pose, theta2 out of range!"
        print "**********************************"
       
    theta3 = angleB - pi/2

    if theta3 < theta3_lower_limit or theta3 > theta3_upper_limit:
        print "**********************************"
        print "Invalid Pose, theta3 out of range!"
        print "**********************************"

    return theta1, theta2, theta3


def get_joints4_5_6(dh, R_ee, theta1, theta2, theta3):
    """
    Calculate joint Euler angles 4,5,6 using analytical IK method.
    NOTE: Joints 4,5,6 constitute the wrist and control WC orientation
    """
    # Compute individual transforms between adjacent links
    # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
    T0_1 = get_TF(dh['alpha0'], dh['a0'], dh['d1'], dh['theta1'])
    T1_2 = get_TF(dh['alpha1'], dh['a1'], dh['d2'], dh['theta2'])
    T2_3 = get_TF(dh['alpha2'], dh['a2'], dh['d3'], dh['theta3'])

    # Extract rotation components of joints 1,2,3 from their
    # respective individual link Transforms
    R0_1 = T0_1[0:3, 0:3]
    R1_2 = T1_2[0:3, 0:3]
    R2_3 = T2_3[0:3, 0:3]

    # Evaluate the composite rotation matrix fromed by composing
    # these individual rotation matrices
    R0_3 = R0_1 * R1_2 * R2_3
    #print "==================================="
    #print "R0_3"
    #print R0_3

    # R3_6 is the composite rotation matrix formed from an extrinsic
    # x-y-z (roll-pitch-yaw) rotation sequence that orients WC
    R3_6 = inv(array(R0_3, dtype='float')) * R_ee  # b/c R0_6 == R_ee = R0_3*R3_6
    #print "==================================="
    #print "R3_6"
    #print R3_6

    r21 = R3_6[1, 0]  # sin(theta5)*cos(theta6)
    r22 = R3_6[1, 1]  # -sin(theta5)*sin(theta6)
    r13 = R3_6[0, 2]  # -sin(theta5)*cos(theta4)
    r23 = R3_6[1, 2]  # cos(theta5)
    r33 = R3_6[2, 2]  # sin(theta4)*sin(theta5)

    #print "==================================="
    #print sqrt(r13**2 + r33**2)

    # Define wrist joint limits
    theta4_lower_limit = -2.0944
    theta4_upper_limit =  2.0944
    theta5_lower_limit = -1.7453
    theta5_upper_limit =  1.7453

    # Compute Euler angles theta 4,5,6 from R3_6 by individually
    # isolating and explicitly solving each angle
    theta5 = arctan2(sqrt(r13**2 + r33**2), -r23)
    if round(sqrt(r13**2 + r33**2), 5) == 0.0:
        theta5 = 0.0
    if theta5 < theta5_lower_limit or theta5 > theta5_upper_limit:
        print "**********************************"
        print "Invalid Pose, theta5 out of range!"
        print "**********************************"

    theta4 = arctan2(r33, r13)
    if round(r33, 5) == 0.0:
        theta4 = 0.0

    theta6 = arctan2(-r22, r21)
    if round(-r22, 5) == 0.0:
        theta6 = 0.0

    if theta4 < theta4_lower_limit:
        theta4 += pi
    
        theta5 = (-theta5)
        if theta6 < 0:
            theta6 += pi
        elif theta6 > 0:
            theta6 -= pi
        
    if theta4 > theta4_upper_limit:
        theta4 -= pi
        theta5 = (-theta5)
        if theta6 < 0:
            theta6 += pi
        elif theta6 > 0:
            theta6 -= pi

    return theta4, theta5, theta6


def plot_EE(received_ee_points, fk_ee_points, ee_errors):
    """
    Plot and compare EE positions received in IK request with those from FK.
    NOTE: Plot window for a particular requested trajectory will pop up during
          pick-place simulation prior to the execution of that trajectory
          and will have to be closed to unpause the pick-place operation
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    data1 = array(received_ee_points)
    x1, y1, z1 = data1.T
    ax.scatter(x1, y1, z1, c='blue', s=50, marker='o')
    ax.plot(x1, y1, z1, c='blue', label='rec_ee')

    data2 = array(fk_ee_points)
    x2, y2, z2 = data2.T
    ax.scatter(x2, y2, z2, c='orange', s=50, marker='s')
    ax.plot(x2, y2, z2, c='orange', label='fk_ee')

    data3 = array(ee_errors)
    x3, y3, z3 = data3.T
    ax.scatter(x3, y3, z3, c='magenta', s=50, marker='^')
    ax.plot(x3, y3, z3, c='magenta', label='ee_error')

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    ax.xaxis.label.set_color('red')
    ax.yaxis.label.set_color('green')
    ax.zaxis.label.set_color('blue')

    #ax.view_init(30,220)
    plt.legend()
    plt.show()


def handle_calculate_IK(req):
    """Handle request from a CalculateIK type service."""
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        dh = get_DH_Table()
        # Initialize service response consisting of a list of
        # joint trajectory positions (joint angles) corresponding
        # to a given gripper pose
        joint_trajectory_list = []

        # To store coordinates for plotting (in plot_ee() function)
        received_ee_points = []
        fk_ee_points = []
        ee_errors = []

        # For each gripper pose a response of six joint angles is computed
        len_poses = len(req.poses)
        for x in xrange(0, len_poses):
            joint_trajectory_point = JointTrajectoryPoint()

            # INVERSE KINEMATICS
            ee_pose = get_ee_pose(req.poses[x])
            print "==================================="
            print "ee_pose: "
            print ee_pose

            received_ee_points.append(ee_pose[0])

            R_ee = get_R_EE(ee_pose)

            Wc = get_WC(dh, R_ee, ee_pose)

            # Calculate angles for joints 1,2,3 and update dh table
            theta1, theta2, theta3 = get_joints1_2_3(dh, Wc)
            dh['theta1'] = round(theta1, 5)
            dh['theta2'] = round((theta2+pi/2), 5)  # account for 90 deg constant offset
            dh['theta3'] = round(theta3, 5)
            # Calculate angles for joints 4,5,6 and update dh table
            theta4, theta5, theta6 = get_joints4_5_6(dh, R_ee, theta1, theta2, theta3)
            dh['theta4'] = round(theta4, 5)
            dh['theta5'] = round(theta5, 5)
            dh['theta6'] = round(theta6, 5)

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3,
                                                theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
            print "~~~~~~~~~~~~~~~~~~~joint_trajectory_list~~~~~~"
            print joint_trajectory_list

            def calculate_FK():
                
                # Test Forward Kinematics
                """
                dh['theta1'] = 0 - 0.69
                dh['theta2'] = pi/2 - 0.22
                dh['theta3'] = 0 + 0.01
                dh['theta4'] = 0 - 0.37
                dh['theta5'] = 0 + 0.25
                dh['theta6'] = 0 - 0.53
                """

                """
                dh['theta1'] = 0 - 0.69
                dh['theta2'] = pi/2 - 0.22
                dh['theta3'] = 0 + 0.01
                dh['theta4'] = 0 + 2.8
                dh['theta5'] = 0 - 0.28
                dh['theta6'] = 0 + 2.57
                """

                """Calculate Forward Kinematics for verifying joint angles."""
                # Compute individual transforms between adjacent links
                # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
                T0_1 = get_TF(dh['alpha0'], dh['a0'], dh['d1'], dh['theta1'])
                T1_2 = get_TF(dh['alpha1'], dh['a1'], dh['d2'], dh['theta2'])
                T2_3 = get_TF(dh['alpha2'], dh['a2'], dh['d3'], dh['theta3'])
                T3_4 = get_TF(dh['alpha3'], dh['a3'], dh['d4'], dh['theta4'])
                T4_5 = get_TF(dh['alpha4'], dh['a4'], dh['d5'], dh['theta5'])
                T5_6 = get_TF(dh['alpha5'], dh['a5'], dh['d6'], dh['theta6'])
                T6_ee = get_TF(dh['alpha6'], dh['a6'], dh['dG'], dh['thetaG'])

                #print "==================================="
                #print "T0_3: "
                #print T0_1*T1_2*T2_3
                print "==================================="
                print "T3_6"
                print T3_4*T4_5*T5_6

                # Create overall transform between base frame and EE by
                # composing the individual link transforms
                T0_ee = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_ee
                
                print "==================================="
                print "T0_ee: "
                print T0_ee

                fk_ee = [T0_ee[0, 3], T0_ee[1, 3], T0_ee[2, 3]]
                fk_ee_points.append([round(fk_ee[0].item(0), 8),
                                     round(fk_ee[1].item(0), 8),
                                     round(fk_ee[2].item(0), 8)])
                print "==================================="
                print "fk_ee_points: "
                print fk_ee_points

                ee_x_e = abs(fk_ee[0] - ee_pose[0][0])
                ee_y_e = abs(fk_ee[1] - ee_pose[0][1])
                ee_z_e = abs(fk_ee[2] - ee_pose[0][2])
                ee_errors.append([round(ee_x_e.item(0), 8),
                                  round(ee_y_e.item(0), 8),
                                  round(ee_z_e.item(0), 8)])
                print "==================================="
                print "ee_errors: "
                print ee_errors

            # NOTE: Uncomment following line to compute FK for plotting EE
            calculate_FK()

        rospy.loginfo("Number of joint trajectory points:" +
                      " %s" % len(joint_trajectory_list))

        # NOTE: Uncomment the following line to plot EE trajectories
        #plot_EE(received_ee_points, fk_ee_points, ee_errors)

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    """Initialize IK_server ROS node and declare calculate_ik service."""
    rospy.init_node('IK_server')
    s = rospy.Service('/gauss/calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
