#!/usr/bin/env python

# Client-like problem generation interface
# Gets map, current robot position and goal as initial states to be passed to the problem file generator

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from ros_enhsp.srv import ProblemGenerator
import rospkg
import rospy
from std_msgs.msg import String

map = OccupancyGrid()
def map_callback(msg):
    global map
    map = msg

robot_pose = PoseWithCovarianceStamped()
def pose_callback(msg):
    global robot_pose
    robot_pose = msg

#### Node function ####
def node():
    # Initialize node
    rospy.init_node('problem_interface', anonymous=True)

    # Subscribers
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

    # Publishers
    pub = rospy.Publisher('/ros_enhsp/problem', String, queue_size=10)
    
    # Node publish rate
    rate = rospy.Rate(10)

    # Wait for static_map service as a flag that the map is already available
    rospy.loginfo('[Problem Interface] Waiting for map')
    rospy.wait_for_service('static_map')
    # In any case, wait for height to be non-null
    while map.info.height == 0:
        pass
    rospy.loginfo('[Problem Interface] Got %dx%d map, %.4f m/cell resolution' % (map.info.width, map.info.height, map.info.resolution))
    
    # # Wait for robot pose
    # rospy.loginfo('[Problem Interface] Waiting for current robot pose')
    # #_, robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    # robot_pose = PoseWithCovarianceStamped()
    # robot_pose.header.frame_id = 'odom'
    # robot_pose.pose.pose.position.x = 30.0
    # robot_pose.pose.pose.position.y = 30.0
    # robot_pose.pose.pose.orientation.w = 1.0
    # rospy.loginfo('[Problem Interface] Got pose (%f, %f)' % (robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y))

    # # Wait for goal
    # rospy.loginfo('[Problem Interface] Waiting for new goal')
    # # _, goal = rospy.wait_for_message('/planner/goal', PoseWithCovarianceStamped)
    # goal = PoseStamped()
    # goal.pose.position.x = 61.0
    # goal.pose.position.y = 61.0
    # goal.pose.orientation.w = 1.0
    # rospy.loginfo('[Problem Interface] Got new goal (%f, %f)' % (goal.pose.position.x, goal.pose.position.y))
    
    # # Wait for service
    # rospy.wait_for_service('problem_generator')

    # # Call service
    # rospy.loginfo('[Problem Interface] Calling problem generator')
    # problem_generator_srv = rospy.ServiceProxy('problem_generator', problem_generator)
    # problem_str = problem_generator_srv(map, robot_pose, goal)
    # rospy.loginfo('[Problem Interface] Problem successfully generated')

    # # Write problem string to file
    # with open('problem.pddl', 'w') as problem_file:
    #     problem_file.write(problem_str.problem)

    while not rospy.is_shutdown():
        # Wait for robot pose
        rospy.loginfo('[Problem Interface] Waiting for robot pose')
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        # rospy.loginfo('[Problem Interface] Got pose (%f, %f)' % (robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y))

        # Wait for goal
        rospy.loginfo('[Problem Interface] Waiting for new goal')
        goal = rospy.wait_for_message('/ros_enhsp/goal', PoseStamped)
        rospy.loginfo('[Problem Interface] Got new goal (%f, %f)' % (goal.pose.position.x, goal.pose.position.y))

        # Wait for problem generator service
        rospy.wait_for_service('ProblemGenerator')

        # Call service
        rospy.loginfo('[Problem Interface] Calling problem generator service')
        problem_generator_srv = rospy.ServiceProxy('ProblemGenerator', ProblemGenerator)
        problem_str = problem_generator_srv(map, robot_pose, goal)
        rospy.loginfo('[Problem Interface] Problem successfully generated')

        # Publish problem message
        # rospy.loginfo('[Problem Interface] Publishing problem to %s topic' % '/ros_enhsp/problem')
        problem_msg = String()
        problem_msg.data = problem_str.problem
        pub.publish(problem_msg)

        # Writing problem file to common folder
        with open('%s/common/problem.pddl' % rospkg.RosPack().get_path('ros_enhsp'), 'w') as problem_file:
            problem_file.write(problem_str.problem)

        # Sleep
        rospy.Time().now()
        rate.sleep()

if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass