#!/usr/bin/env python

# Server-like service advertiser for problem generation

import cv2
import numpy as np
import rospy
from ros_enhsp.srv import ProblemGenerator, ProblemGeneratorResponse

#### Service handler ####
def ProblemGenerator_handler(req):
    # # Define obstacles from map
    # obstacles = []
    # i_obs = 1
    # for grid_x in range(req.map.info.width):
    #     for grid_y in range(req.map.info.height):
    #         occupancy = req.map.data[grid_y * req.map.info.width + grid_x]
    #         if occupancy >= 90:
    #             x = grid_x * req.map.info.resolution + req.map.info.origin.position.x
    #             y = grid_y * req.map.info.resolution + req.map.info.origin.position.y
    #             obstacles.append(['o%d' % i_obs, x, y])
    #             i_obs += 1
    
    # Problem header
    problem = '(define (problem turtle_problem)\n\t(:domain turtlebot)'

    # # Define robot object
    problem += '\n\n\t(:objects\n\t\t\tturtle -robot'
    
    # Define obstacles' objects
    for i in range(1,7+1):
        problem += '\n\t\t\t o%d -obstacle' % i
    problem += '\n\t)'
    # #### Initial state
    problem += '\n\n\t(:init'

    # Map data
    problem += '\n\t\t\t(= (resolution) %.1f)' % (req.map.info.resolution)
    problem += '\n\t\t\t(= (maxx) %.1f)' % (req.map.info.width  * req.map.info.resolution + req.map.info.origin.position.x)
    problem += '\n\t\t\t(= (maxy) %.1f)' % (req.map.info.height * req.map.info.resolution + req.map.info.origin.position.y)
    problem += '\n\t\t\t(= (minx) %.1f)' % (req.map.info.origin.position.x)
    problem += '\n\t\t\t(= (miny) %.1f)' % (req.map.info.origin.position.y)

    # Obstacle data
    # Left wall
    problem += '\n\n\t\t\t(= (cx o1) %.1f)' % (req.map.info.resolution * 0.5)
    problem += '\n\t\t\t(= (cy o1) %.1f)' % (req.map.info.resolution * 28.5)
    problem += '\n\t\t\t(= (rc1 o1) %.1f)' % (req.map.info.resolution * 2 + 2)
    problem += '\n\t\t\t(= (rc2 o1) %.1f)' % (req.map.info.resolution * 29.5 + 2)
    problem += '\n\t\t\t(= (re1 o1) %.1f)' % (req.map.info.resolution * 5 + 2)
    problem += '\n\t\t\t(= (re2 o1) %.1f)' % (req.map.info.resolution * 32.5 + 2)
    
    # Right wall
    problem += '\n\n\t\t\t(= (cx o2) %.1f)' % (req.map.info.resolution * 46.5)
    problem += '\n\t\t\t(= (cy o2) %.1f)' % (req.map.info.resolution * 28.5)
    problem += '\n\t\t\t(= (rc1 o2) %.1f)' % (req.map.info.resolution * 2 + 2)
    problem += '\n\t\t\t(= (rc2 o2) %.1f)' % (req.map.info.resolution * 29.5 + 2)
    problem += '\n\t\t\t(= (re1 o2) %.1f)' % (req.map.info.resolution * 5 + 2)
    problem += '\n\t\t\t(= (re2 o2) %.1f)' % (req.map.info.resolution * 32.5 + 2)
    
    # Bottom wall
    problem += '\n\n\t\t\t(= (cx o3) %.1f)' % (req.map.info.resolution * 24)
    problem += '\n\t\t\t(= (cy o3) %.1f)' % (req.map.info.resolution * 0.5)
    problem += '\n\t\t\t(= (rc1 o3) %.1f)' % (req.map.info.resolution * 24 + 2)
    problem += '\n\t\t\t(= (rc2 o3) %.1f)' % (req.map.info.resolution * 2 + 2)
    problem += '\n\t\t\t(= (re1 o3) %.1f)' % (req.map.info.resolution * 28 + 2)
    problem += '\n\t\t\t(= (re2 o3) %.1f)' % (req.map.info.resolution * 5 + 2)

    # Top wall
    problem += '\n\n\t\t\t(= (cx o4) %.1f)' % (req.map.info.resolution * 24)
    problem += '\n\t\t\t(= (cy o4) %.1f)' % (req.map.info.resolution * 58.5)
    problem += '\n\t\t\t(= (rc1 o4) %.1f)' % (req.map.info.resolution * 24 + 2)
    problem += '\n\t\t\t(= (rc2 o4) %.1f)' % (req.map.info.resolution * 2 + 2)
    problem += '\n\t\t\t(= (re1 o4) %.1f)' % (req.map.info.resolution * 28 + 2)
    problem += '\n\t\t\t(= (re2 o4) %.1f)' % (req.map.info.resolution * 5 + 2)

    # Middle wall
    # problem += '\n\n\t\t\t(= (cx o5) %.1f)' % (req.map.info.resolution * 16)
    # problem += '\n\t\t\t(= (cy o5) %.1f)' % (req.map.info.resolution * 28.5)
    # problem += '\n\t\t\t(= (rc1 o5) %.1f)' % (req.map.info.resolution * 25.5)
    # problem += '\n\t\t\t(= (rc2 o5) %.1f)' % (req.map.info.resolution * 2)
    # problem += '\n\t\t\t(= (re1 o5) %.1f)' % (req.map.info.resolution * 27.5)
    # problem += '\n\t\t\t(= (re2 o5) %.1f)' % (req.map.info.resolution * 4)

    # Box upper
    problem += '\n\n\t\t\t(= (cx o5) %.1f)' % (req.map.info.resolution * 24.5)
    problem += '\n\t\t\t(= (cy o5) %.1f)' % (req.map.info.resolution * 53)
    problem += '\n\t\t\t(= (rc1 o5) %.1f)' % (req.map.info.resolution * 6+2)
    problem += '\n\t\t\t(= (rc2 o5) %.1f)' % (req.map.info.resolution * 6.5+2)
    problem += '\n\t\t\t(= (re1 o5) %.1f)' % (req.map.info.resolution * 8+6)
    problem += '\n\t\t\t(= (re2 o5) %.1f)' % (req.map.info.resolution * 8.5+6)

    # Box center
    problem += '\n\n\t\t\t(= (cx o6) %.1f)' % (req.map.info.resolution * 24.5)
    problem += '\n\t\t\t(= (cy o6) %.1f)' % (req.map.info.resolution * 20.5)
    problem += '\n\t\t\t(= (rc1 o6) %.1f)' % (req.map.info.resolution * 5+2)
    problem += '\n\t\t\t(= (rc2 o6) %.1f)' % (req.map.info.resolution * 5+2)
    problem += '\n\t\t\t(= (re1 o6) %.1f)' % (req.map.info.resolution * 8+4)
    problem += '\n\t\t\t(= (re2 o6) %.1f)' % (req.map.info.resolution * 8+4)
    
    # Box lower right
    problem += '\n\n\t\t\t(= (cx o7) %.1f)' % (req.map.info.resolution * 40.5)
    problem += '\n\t\t\t(= (cy o7) %.1f)' % (req.map.info.resolution * 5.5)
    problem += '\n\t\t\t(= (rc1 o7) %.1f)' % (req.map.info.resolution * 5+2)
    problem += '\n\t\t\t(= (rc2 o7) %.1f)' % (req.map.info.resolution * 5+2)
    problem += '\n\t\t\t(= (re1 o7) %.1f)' % (req.map.info.resolution * 8+4)
    problem += '\n\t\t\t(= (re2 o7) %.1f)' % (req.map.info.resolution * 8+4)
    
    # # Robot position
    problem += '\n\n\t\t\t(= (x turtle) %d)' % (req.pose.pose.pose.position.x / req.map.info.resolution)
    problem += '\n\t\t\t(= (y turtle) %d)' % (req.pose.pose.pose.position.y / req.map.info.resolution)

    # # Robot energy
    problem += '\n\n\t\t\t(= (energy turtle) 0)'
    
    # # Close init
    problem += '\n\t)'

    # Goal
    problem += '\n\n\t(:goal'
    problem += '\n\t\t\t(and'
    problem += '\n\t\t\t\t(= (x turtle) %d)' % (req.goal.pose.position.x / req.map.info.resolution)
    problem += '\n\t\t\t\t(= (y turtle) %d)' % (req.goal.pose.position.y / req.map.info.resolution)
    problem += '\n\t\t\t\t(= (energy turtle) 0)'
    problem += '\n\t\t\t)'
    problem += '\n\t)'

    # Metric
    if rospy.get_param('/problem_generator/ignore_metric'):
        rospy.loginfo('[Problem Generator] Ignoring metric in problem')
        pass
    else:
        problem += '\n\t(:metric minimize (energy turtle))'

    # Close domain
    problem += '\n)'

    return ProblemGeneratorResponse(problem)


#### Node function ####
def node():    
    # Initialize node
    rospy.init_node('problem_generator', anonymous=True)

    # Advertise service
    _ = rospy.Service('ProblemGenerator', ProblemGenerator, ProblemGenerator_handler)
    # rospy.loginfo("[Problem Generator] Ready")

    # Run node until killed
    rospy.spin()

if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass