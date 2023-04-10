#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from d_bot_m2m_navigation.srv import TaskCall, TaskCallResponse, TaskCallRequest

from api.task import Task
from nav.slammod import astar, heuristic

class Navigation:
    
    def __init__(self):
        self.goal = None
        self.active = False

    def add_nav_goal(self, req):
        task = Task().load(req.task)
        rospy.loginfo(task)
        try:
            self.goal = task.location
            task.success = True
            self.active = True
            rospy.loginfo("new nav goal added")
        except:
            task.error = "failed to add nav goal"
            task.success = False
            rospy.loginfo(task.error)
        task_json = task.jsonify()
        response = TaskCallResponse()
        response.task = task_json
        return response

    def start_nav_goal_service(self):
        return rospy.Service('/navigation/goal', TaskCall, self.add_nav_goal)

if __name__ == '__main__':
    rospy.init_node('navigation')
    nav = Navigation()
    nav_srv = nav.start_nav_goal_service()
    rospy.spin()