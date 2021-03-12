#! /usr/bin/env python
import rospy
import time
import actionlib
import sys
import math
import json
from tf import TransformListener
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback


class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.distance_tolerance = 2
        self.odom_frame_id = 'map'
        self.base_frame_id = 'base_footprint'
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.feedback_wait = 10   # wait n times before printing
        self.feedback_count = self.feedback_wait   # print first time then set to zero

    def send_goal(self, goal):
        self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        # self.client.wait_for_result()
        distance = 10
        while(distance > self.distance_tolerance):
            # now = rospy.Time()
            self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, rospy.Time(), rospy.Duration(4.0))
            trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, rospy.Time(0))
            distance = math.sqrt(pow(goal.target_pose.pose.position.x-trans[0],2)+pow(goal.target_pose.pose.position.y-trans[1],2))
        rospy.loginfo('[Result] State: %d' %(self.client.get_state()))

    def feedback_cb(self, feedback):
        if self.feedback_count >= self.feedback_wait:
            current_pose = feedback.base_position.pose
            rospy.loginfo("Current position on map: x{}, y{}".format(current_pose.position.x, current_pose.position.y))
            self.feedback_count = 0   # reset
        else:   # do not print
            self.feedback_count += 1
    
    def cancel_goal(self):
        self.client.cancel_all_goals()
        # print('goal cancelled')

    def done_cb(self, status, result):
        rospy.loginfo("Goal reached")
        rospy.loginfo('Status is: ' + str(status))

    # def shutdown(self):
    #     self.client.cancel_all_goals()


class MoveBaseGoalCreator:
    @staticmethod
    def create_2D_goal(pose_2D):
        """2d_pose = { "x": x, "y":y, "w": w }"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = pose_2D["x"]
        goal.target_pose.pose.position.y = pose_2D["y"]
        goal.target_pose.pose.orientation.z = math.sin(pose_2D["w"]/2)
        goal.target_pose.pose.orientation.w = math.cos(pose_2D["w"]/2)
        return goal

if __name__ == "__main__":

    if len(sys.argv) < 1:
        print("usage: my_node.py arg1 arg2 **")
    else:
        print("start")

    # # initializes the action client node
    rospy.init_node('move_base_action_client')
    client = MoveBaseClient()

    goals_json = "/home/asraf/catkin_ws/src/magni_launch/poses.json"

    with open(goals_json, "r") as f:
        goals_pose = json.load(f)

    # g_pose = goals_pose[sys.argv[1]]
    # goal = MoveBaseGoalCreator.create_2D_goal(g_pose)
    # client.send_goal(goal)

    l_len = len(sys.argv)
    poses = sys.argv[1:l_len] #ignore first element
    
    goals = [MoveBaseGoalCreator.create_2D_goal(goals_pose[pose]) for pose in poses]    
    for goal in goals:
        client.send_goal(goal)
    # print("send all goals")

    if rospy.is_shutdown():
        client.cancel_goal()
    
    # rospy.on_shutdown(client.cancel_goal)

    # # Uncomment these lines to test goal preemption:
    # time.sleep(3.0)
    # #client.cancel_goal()  # would cancel the goal 3 seconds after starting


