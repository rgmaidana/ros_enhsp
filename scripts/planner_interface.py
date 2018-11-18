#!/usr/bin/env python

# Planner interface
# Waits for new problem to arrive and runs planner based on domain

from ros_enhsp.msg import Plan
import rospkg
import rospy
from std_msgs.msg import String
import subprocess

package_path = rospkg.RosPack().get_path('ros_enhsp')

#### Node function ####
def node():
    # Initialize node
    rospy.init_node('planner_interface', anonymous=True)

    # Publishers
    pub = rospy.Publisher('/ros_enhsp/plan', Plan, queue_size=10)
    status_pub = rospy.Publisher('/ros_enhsp/status', String, queue_size=10)
    
    # Node publish rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Wait for new problem
        # rospy.loginfo('[Planner Interface] Waiting for new problem')
        rospy.wait_for_message('/ros_enhsp/problem', String)

        # Call planner subprocess
        # rospy.loginfo('[Planner Interface] New problem available')
        proc1 = subprocess.Popen(['timeout -s KILL 50s %s/common/enhsp -o %s/common/domain.pddl -f %s/common/problem.pddl' 
                                 % (package_path, package_path, package_path)], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = proc1.communicate()
        
        # If there is an error in planning, exit node while displaying error
        if not err == '':
            if err == 'Killed\n':
                rospy.logerr('[Planner Interface] Timeout looking for plan (probably it does not exist). Please provide another goal position.')
            else:
                rospy.logerr('[Planner Interface] Error running planner process: %s' % err)
                rospy.signal_shutdown(reason='')
                break

        # Create new plan message
        new_plan = Plan()
        new_plan.header.stamp = rospy.Time().now()

        # If problem is solvable and planner did not timeout, there are no strings with "unsolvable" in planner output
        if out.find('Unsolvable', 0, len(out)) < 0 and not err == 'Killed\n':
            # If no errors, new plan has been found
            # rospy.loginfo('[Planner Interface] New plan found!')
        
            # Success status
            new_plan.status = 0

            # Get plan data
            plan_start_index = out.find('Plan is valid:true', 0, len(out))
            plan_end_index = out.find('Plan-Length:', plan_start_index, len(out)) - 1
            plan_list = out[plan_start_index:plan_end_index].strip('Plan is valid:true\n').split('\n')
            # plan_length = len(plan_list)
            # plan_time = float([s for s in out.split('\n') if 'Planning Time:' in s][0].split(':')[1])
            # rospy.loginfo('[Planner Interface] Planning time: %.3f seconds' % (plan_time/1000.0))
            # rospy.loginfo('[Planner Interface] Plan length: %d steps' % plan_length)

            # Parse plan and put actions in plan topic
            for act in plan_list:
                i_start = act.find(')', 0, len(act)) + 1
                i_end = act.find('Parameters:', i_start, len(act)) - 1
                new_plan.actions.append(act[i_start:i_end])

        # If problem is unsolvable or planner timed out, no actions should be reported, and status should be 1 (failure)
        else:
            new_plan.status = 1
            if not err == 'Killed\n':
                rospy.logerr('[Planner Interface] Problem unsolvable')
            status_msg = String()
            status_msg.data='failed'
            status_pub.publish(status_msg)

        # Publish plan topic
        # rospy.loginfo('[Planner Interface] Publishing plan to %s topic' % '/ros_enhsp/plan')
        pub.publish(new_plan)

        # # Save plan, map resolution and robot initial position in text file
        # # As the planner dispatch needs sudo permissions in jetson to change frequency,
        # # it cannot be run as a ros node

        # with open('%s/common/problem.pddl' % package_path, 'r') as prob_file:
        #     line = ' '
        #     while not line == '':
        #         line = prob_file.readline()
        #         if '(= (resolution)' in line:
        #             map_res = float(line.split('(= (resolution)')[1].split(')')[0])
        #         elif '(= (x turtle)' in line:
        #             init_x = float(line.split('(= (x turtle)')[1].split(')')[0])
        #         elif '(= (y turtle)' in line:
        #             init_y = float(line.split('(= (y turtle)')[1].split(')')[0])
        #             break

        # with open('%s/common/plan.txt' % rospkg.RosPack().get_path('ros_enhsp'), 'w') as plan_file:
        #     plan_file.write('resolution:%.1f\n' % map_res)
        #     plan_file.write('init_x:%.1f\n' % init_x)
        #     plan_file.write('init_y:%.1f\n\n' % init_y)
        #     for action in new_plan.actions:
        #         plan_file.write('%s\n' % action.strip(' '))

        # # Run plan dispatcher with sudo permissions
        # rospy.loginfo('[Planner Interface] Running planner dispatch')
        # proc2 = subprocess.Popen(['python', '%s/scripts/planner_dispatch.py' % package_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # out, err = proc2.communicate()
        # if not err == '':
        #     rospy.logerr('[Planner Interface] Could not run planner dispatch: %s' % err)
        # else:
            # rospy.loginfo('[Planner Interface] Plan executed successfully')

        # Sleep
        rate.sleep()

if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass