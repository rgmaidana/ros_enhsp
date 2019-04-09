#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from math import pi, cos, sin
import numpy as np
import rospkg
import rospy
import subprocess
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

package_path = rospkg.RosPack().get_path('ros_enhsp')

final_pos_marker = Marker()
final_pos_marker.type = final_pos_marker.CUBE
final_pos_marker.action = final_pos_marker.ADD

def rotateCoords(coords, th):
    R = np.array([[cos(th),  sin(th)],
                  [-sin(th), cos(th)]])  # Rotation matrix
    xy = np.array(coords)
    return R.dot(xy)

def get_path(actions, theta=0.0, resolution=0.05):
    # Get path from plan
    path_msg = Path()
    path_msg.header.stamp = rospy.Time().now()
    path_msg.header.frame_id = 'map'
    
    robot_x = 0
    robot_y = 0
    performance_cnt, efficiency_cnt = 0, 0
    for i in range(len(actions)):
        # Move up
        if (actions[i] == ' move_up_low_e' or actions[i] == ' move_up_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_y += resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            q = quaternion_from_euler(0, 0, (pi*90)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]
            path_msg.poses.append(path_pose)

        # Move up left
        elif (actions[i] == ' move_up_left_low_e' or actions[i] == ' move_up_left_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_x -= resolution
            robot_y += resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            q = quaternion_from_euler(0, 0, (pi*135)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]
            path_msg.poses.append(path_pose)


        # Move left
        elif (actions[i] == ' move_left_low_e' or actions[i] == ' move_left_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_x -= resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            path_msg.poses.append(path_pose)
            q = quaternion_from_euler(0, 0, (pi*180)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]

        # Move down left
        elif (actions[i] == ' move_down_left_low_e' or actions[i] == ' move_down_left_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_x -= resolution
            robot_y -= resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            q = quaternion_from_euler(0, 0, (pi*225)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]
            path_msg.poses.append(path_pose)

        # Move down
        elif (actions[i] == ' move_down_low_e' or actions[i] == ' move_down_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_y -= resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            q = quaternion_from_euler(0, 0, (pi*270)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]
            path_msg.poses.append(path_pose)
        
        # Move down right
        elif (actions[i] == ' move_down_right_low_e' or actions[i] == ' move_down_right_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_x += resolution
            robot_y -= resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            q = quaternion_from_euler(0, 0, (pi*315)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]
            path_msg.poses.append(path_pose)

        # Move right
        elif (actions[i] == ' move_right_low_e' or actions[i] == ' move_right_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_x += resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            q = quaternion_from_euler(0, 0, (pi*0)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]
            path_msg.poses.append(path_pose)

        # Move up right
        elif (actions[i] == ' move_up_right_low_e' or actions[i] == ' move_up_right_high_e'):
            path_pose = PoseStamped()
            path_pose.header.frame_id = 'map'
            robot_x += resolution
            robot_y += resolution
            robotxy = rotateCoords([robot_x, robot_y], theta)
            path_pose.pose.position.x = robotxy[0]
            path_pose.pose.position.y = robotxy[1]
            q = quaternion_from_euler(0, 0, (pi*45)/180.0)
            path_pose.pose.orientation.x = q[0]
            path_pose.pose.orientation.y = q[1]
            path_pose.pose.orientation.z = q[2]
            path_pose.pose.orientation.w = q[3]
            path_msg.poses.append(path_pose)
        
        # Toggle performance mode
        elif (actions[i] == ' turn_on_laser'):
            performance_cnt += 1

        # Toggle efficiency mode
        elif (actions[i] == ' turn_off_laser'):
            efficiency_cnt += 1

    print "# of times toggled performance mode: %d" % performance_cnt
    print "# of times toggled efficiency mode: %d" % efficiency_cnt
    return path_msg

if __name__ == "__main__":
    rospy.init_node('path_publisher', anonymous=True)
    rate = rospy.Rate(10)

    # Publishers
    path_m_pub = rospy.Publisher('/path_metric', Path, queue_size=10)
    path_im_pub = rospy.Publisher('/path_ignore_metric', Path, queue_size=10)
    final_pos_pub = rospy.Publisher('/final_pos_marker', Marker, queue_size=10)

    # Get map angle and resolution
    th = rospy.get_param('/problem_generator/map_angle')*pi/180.0   # Param in degrees, convert to radians
    resol = rospy.get_param('/problem_generator/map_resolution')

    # Call planner subprocess ignoring metric
    proc1 = subprocess.Popen(['timeout -s KILL 25s %s/common/enhsp -o %s/common/domain.pddl -f %s/common/interesting_problems/problem3.pddl -im' 
                                % (package_path, package_path, package_path)], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc1.communicate()
    print out

    plan_start_index = out.find('Plan is valid:true', 0, len(out))
    plan_end_index = out.find('Plan-Length:', plan_start_index, len(out)) - 1
    plan_list = out[plan_start_index:plan_end_index].strip('Plan is valid:true\n').split('\n')

    # Parse plan and put actions in plan topic
    actions = []
    for act in plan_list:
        i_start = act.find(')', 0, len(act)) + 1
        i_end = act.find('Parameters:', i_start, len(act)) - 1
        actions.append(act[i_start:i_end])

    path_msg_im = get_path(actions, theta=th, resolution=resol)
    
    # Call planner subprocess considering metric
    proc1 = subprocess.Popen(['timeout -s KILL 25s %s/common/enhsp -o %s/common/domain.pddl -f %s/common/interesting_problems/problem3.pddl' 
                                % (package_path, package_path, package_path)], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc1.communicate()
    print out

    plan_start_index = out.find('Plan is valid:true', 0, len(out))
    plan_end_index = out.find('Plan-Length:', plan_start_index, len(out)) - 1
    plan_list = out[plan_start_index:plan_end_index].strip('Plan is valid:true\n').split('\n')

    # Parse plan and put actions in plan topic
    actions = []
    for act in plan_list:
        i_start = act.find(')', 0, len(act)) + 1
        i_end = act.find('Parameters:', i_start, len(act)) - 1
        actions.append(act[i_start:i_end])

    path_msg_m = get_path(actions, theta=th, resolution=resol)

    final_pos_marker.header.stamp = rospy.Time().now()
    final_pos_marker.header.frame_id = 'map'
    
    final_pos_marker.pose.position.x = path_msg_m.poses[-1].pose.position.x
    final_pos_marker.pose.position.y = path_msg_m.poses[-1].pose.position.y
    final_pos_marker.pose.position.z = path_msg_m.poses[-1].pose.position.z
    final_pos_marker.pose.orientation.x = 0.0
    final_pos_marker.pose.orientation.y = 0.0
    final_pos_marker.pose.orientation.z = 0.0
    final_pos_marker.pose.orientation.w = 1.0

    final_pos_marker.scale.x = 2.0
    final_pos_marker.scale.y = 2.0
    final_pos_marker.scale.z = 2.0

    final_pos_marker.color.r = 0.0
    final_pos_marker.color.g = 0.0
    final_pos_marker.color.b = 1.0
    final_pos_marker.color.a = 1.0

    final_pos_marker.lifetime = rospy.Duration()

    final_pos_pub.publish(final_pos_marker)

    print("Publishing paths")
    while not rospy.is_shutdown():
        path_m_pub.publish(path_msg_m)
        path_im_pub.publish(path_msg_im)
        rate.sleep()