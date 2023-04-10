#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# from hdl_localization.msg import Status

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

    def get_curr_location(self, data):
        try:
            rospy.loginfo("I heard %s",data)
            location = {
                "x": data.pose.pose.position.x,
                "y": data.pose.pose.position.y,
                "z": data.pose.pose.position.z
            }
            orientation = {
                "x": data.pose.pose.orientation.x,
                "y": data.pose.pose.orientation.y,
                "z": data.pose.pose.orientation.z,
                "w": data.pose.pose.orientation.w
            }
            self.location = location
        except:
            rospy.loginfo("Failed to extract position")
        return None

    def get_curr_status(self, data):
        try:
            rospy.loginfo("I heard %s",data)
            rospy.loginfo(data.pose.position.x)
        except:
            rospy.loginfo("Failed to extract position")
        return None

    def start_nav_goal_service(self):
        return rospy.Service('/navigation/goal', TaskCall, self.add_nav_goal)

    def start_status_subscriber(self):
        #return rospy.Subscriber("/status", Status, self.get_curr_status)
        return False

    def start_odom_subscriber(self):
        return rospy.Subscriber("/odom", Odometry, self.get_curr_location)

if __name__ == '__main__':
    rospy.init_node('navigation')
    nav = Navigation()
    nav_srv = nav.start_nav_goal_service()
    status_sub = nav.start_status_subscriber()
    odom_sub = nav.start_odom_subscriber()
    rospy.spin()