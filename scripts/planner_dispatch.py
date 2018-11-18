#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, MapMetaData
from math import pi, pow, atan2, sqrt, sin, cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ros_enhsp.msg import Plan
import rospkg
import rospy
from std_msgs.msg import String
import subprocess
from tf.transformations import euler_from_quaternion, quaternion_from_euler

package_path = rospkg.RosPack().get_path('ros_enhsp')
pose = PoseWithCovarianceStamped()

def update_pose(data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    global pose
    pose = data
    pose.pose.pose.position.x = round(pose.pose.pose.position.x, 4)
    pose.pose.pose.position.y = round(pose.pose.pose.position.y, 4)

def euclidean_distance(goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.pose.position.x - pose.pose.pose.position.x), 2) +
                pow((goal_pose.pose.position.y - pose.pose.pose.position.y), 2))

def linear_vel(goal_pose, constant=1.2):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    return constant * euclidean_distance(goal_pose)

def steering_angle(goal_pose):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    # Must set origin of reference frame to current robot position
    x1, y1 = pose.pose.pose.position.x, pose.pose.pose.position.y
    x2, y2 = goal_pose.pose.position.x, goal_pose.pose.position.y
    ang = atan2(y2-y1,x2-x1)
    return ang

def angular_vel(goal_pose, constant=0.8):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    quaternion = (pose.pose.pose.orientation.x, 
                    pose.pose.pose.orientation.y, 
                    pose.pose.pose.orientation.z, 
                    pose.pose.pose.orientation.w )
    _, _, yaw = euler_from_quaternion(quaternion)
    vel = constant * (steering_angle(goal_pose) - yaw)
    # print 'Angular velocity at %.2f yaw: %.4f' % (yaw, vel)
    return vel

def go_to(goal_pose, publisher, rate):
    """Moves the turtle to the goal."""
    vel_msg = Twist()

    # First, turn towards your objective
    vel_msg.angular.z = angular_vel(goal_pose)
    while sqrt(vel_msg.angular.z**2) >= 0.2:
        # Publishing our vel_msg
        publisher.publish(vel_msg)

        # Keep turning
        vel_msg.angular.z = angular_vel(goal_pose)
        

    # Then go on folllowing the route
    while euclidean_distance(goal_pose) >= 1.4:

        # Move only forward
        vel_msg.linear.x = linear_vel(goal_pose)
        vel_msg.angular.z = 0

        # Publishing our vel_msg
        publisher.publish(vel_msg)

        # Publish at the desired rate.
        rate.sleep()

    # Stopping our robot after the movement is over.
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)

# def movebase_goto(goal):
#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     client.wait_for_server()
#     client.send_goal(goal)
#     wait = client.wait_for_result()
#     if not wait:
#         rospy.logerr('[Planner Dispatch] Action server not available')
#         rospy.signal_shutdown(reason='')
#     else:
#         return client.get_result()

def node():
    # Initialize node
    rospy.init_node('planner_dispatch', anonymous=True)

    # Publishers
    path_pub = rospy.Publisher('/ros_enhsp/path', Path, queue_size=10)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    status_pub = rospy.Publisher('/ros_enhsp/status', String, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, update_pose)

    # Rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Wait for map meta-data
        map = rospy.wait_for_message('/map_metadata', MapMetaData)

        # Wait for robot initial position
        robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)

        # Wait for plan
        rospy.loginfo('[Planner Dispatch] Waiting for new valid plan')
        plan = rospy.wait_for_message('/ros_enhsp/plan', Plan)
        rospy.loginfo('[Planner Dispatch] Got plan')
        rospy.loginfo('[Planner Dispatch] Executing plan')

        # Get path from plan
        path_msg = Path()
        path_msg.header.stamp = rospy.Time().now()
        path_msg.header.frame_id = 'map'
        
        robot_x = pose.pose.pose.position.x
        robot_y = pose.pose.pose.position.y
        for i in range(len(plan.actions)):
            # Move up
            if (plan.actions[i] == ' move_up_low_e' or plan.actions[i] == ' move_up_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_y += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*90)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

            # Move up left
            elif (plan.actions[i] == ' move_up_left_low_e' or plan.actions[i] == ' move_up_left_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x -= map.resolution
                robot_y += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*135)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)


            # Move left
            elif (plan.actions[i] == ' move_left_low_e' or plan.actions[i] == ' move_left_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                path_msg.poses.append(path_pose)
                q = quaternion_from_euler(0, 0, (pi*180)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]

            # Move down left
            elif (plan.actions[i] == ' move_down_left_low_e' or plan.actions[i] == ' move_down_left_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x -= map.resolution
                robot_y -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*225)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

            # Move down
            elif (plan.actions[i] == ' move_down_low_e' or plan.actions[i] == ' move_down_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_y -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*270)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)
            
            # Move down right
            elif (plan.actions[i] == ' move_down_right_low_e' or plan.actions[i] == ' move_down_right_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x += map.resolution
                robot_y -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*315)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

            # Move right
            elif (plan.actions[i] == ' move_right_low_e' or plan.actions[i] == ' move_right_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*0)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

            # Move up right
            elif (plan.actions[i] == ' move_up_right_low_e' or plan.actions[i] == ' move_up_right_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x += map.resolution
                robot_y += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*45)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

        path_pub.publish(path_msg)        

        # Act upon full plan (not just path)
        path_msg = Path()
        path_msg.header.stamp = rospy.Time().now()
        path_msg.header.frame_id = 'map'
        
        robot_x = pose.pose.pose.position.x
        robot_y = pose.pose.pose.position.y
        for i in range(len(plan.actions)):
            # Move up
            if (plan.actions[i] == ' move_up_low_e' or plan.actions[i] == ' move_up_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_y += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*90)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)

                # Move to goalction servere)
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler(0, 0, (pi*90)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)

            # Move up left
            if (plan.actions[i] == ' move_up_left_low_e' or plan.actions[i] == ' move_up_left_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x -= map.resolution
                robot_y += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*135)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)

                # Send goal to move_base action server
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler()
                # q = quaternion_from_euler(0, 0, (pi*135)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)

            # Move left
            elif (plan.actions[i] == ' move_left_low_e' or plan.actions[i] == ' move_left_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                path_msg.poses.append(path_pose)
                q = quaternion_from_euler(0, 0, (pi*180)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)
                
                # Send goal to move_base action server
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler(0, 0, (pi*180)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)

            # Move down left
            if (plan.actions[i] == ' move_down_left_low_e' or plan.actions[i] == ' move_down_left_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x -= map.resolution
                robot_y -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*225)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)

                # Send goal to move_base action server
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler(0, 0, (pi*-135)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)

            # Move down
            elif (plan.actions[i] == ' move_down_low_e' or plan.actions[i] == ' move_down_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_y -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*270)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)
                # Send goal to move_base action server
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler(0, 0, (pi*-90)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)
            
            # Move down right
            if (plan.actions[i] == ' move_down_right_low_e' or plan.actions[i] == ' move_down_right_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x += map.resolution
                robot_y -= map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*315)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)
                
                # Send goal to move_base action server
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler(0, 0, (pi*-45)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)
            
            # Move right
            if (plan.actions[i] == ' move_right_low_e' or plan.actions[i] == ' move_right_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*0)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)

                # Send goal to move_base action server
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler(0, 0, (pi*0)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)

            # Move up right
            if (plan.actions[i] == ' move_up_right_low_e' or plan.actions[i] == ' move_up_right_high_e'):
                path_pose = PoseStamped()
                path_pose.header.frame_id = 'map'
                robot_x += map.resolution
                robot_y += map.resolution
                path_pose.pose.position.x = robot_x
                path_pose.pose.position.y = robot_y
                q = quaternion_from_euler(0, 0, (pi*45)/180.0)
                path_pose.pose.orientation.x = q[0]
                path_pose.pose.orientation.y = q[1]
                path_pose.pose.orientation.z = q[2]
                path_pose.pose.orientation.w = q[3]
                path_msg.poses.append(path_pose)

                # Move to goal
                go_to(path_pose, velocity_publisher, rate)

                # Send goal to move_base action server
                # mb_goal = MoveBaseGoal()
                # mb_goal.target_pose.header.frame_id = 'map'
                # mb_goal.target_pose.header.stamp = rospy.Time().now()
                # mb_goal.target_pose.pose.position.x = robot_x
                # mb_goal.target_pose.pose.position.y = robot_y
                # q = quaternion_from_euler(0, 0, (pi*45)/180.0)
                # mb_goal.target_pose.pose.orientation.x = q[0]
                # mb_goal.target_pose.pose.orientation.y = q[1]
                # mb_goal.target_pose.pose.orientation.z = q[2]
                # mb_goal.target_pose.pose.orientation.w = q[3]
                # movebase_goto(mb_goal)
           
            # Toggle performance mode
            elif (plan.actions[i] == ' performance_mode'):
                rospy.loginfo('[Planner Dispatch] Performance mode at action %d!' % i)
                proc = subprocess.Popen(['sudo', 'nvpmodel', '-q', '0'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                out, err = proc.communicate()

            # Toggle efficiency mode
            elif (plan.actions[i] == ' efficiency_mode'):
                rospy.loginfo('[Planner Dispatch] Efficiency mode at action %d!' % i)
                proc = subprocess.Popen(['sudo', 'nvpmodel', '-q', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                out, err = proc.communicate()

        status_msg = String()
        status_msg.data = 'success'
        status_pub.publish(status_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass