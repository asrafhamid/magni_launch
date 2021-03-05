#!/usr/bin/env python
import rospy
import roslib 
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion, Vector3, PointStamped

# Because of transformations
import tf

import numpy as np

from transform_utils import *

from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ObsDistPub:
    def __init__(self):
        self.counter = 0
        self.image = None

        self.base_frame = '/base_link'
        self.odom_frame = '/odom'
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(4)
        self.markerArray = MarkerArray()

        self.sub = rospy.Subscriber('/raw_obstacles',Obstacles, self.call_obs)
        self.markerArray_objectlisher = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)
    
    def create_txt_marker(self, cp, index, text='na', ns="", color=(0,0,0)):
        self.marker_object = Marker()
        self.marker_object.type = Marker.TEXT_VIEW_FACING
        self.marker_object.header.frame_id = "/odom"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = ns+"_dist"
        self.marker_object.id = index
        self.marker_object.text = text
        self.marker_object.action = Marker.ADD

        cp.z = 1

        self.marker_object.pose.position.x = cp.x
        self.marker_object.pose.position.y = cp.y
        self.marker_object.pose.position.z = cp.z

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.z = 0.3

        self.marker_object.color.r = color[0]
        self.marker_object.color.g = color[1]
        self.marker_object.color.b = color[2]
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it forever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0.4)
        self.markerArray.markers.append(self.marker_object)

    def create_arrow_marker(self, tail, tip, index, ns, color):
        self.marker_object = Marker()
        self.marker_object.type = Marker.ARROW
        self.marker_object.header.frame_id = "/odom"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = ns+"_arrow"
        self.marker_object.id = index
        self.marker_object.action = Marker.ADD

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale = Vector3(0.05,0.1,0)

        self.marker_object.color.r = color[0]
        self.marker_object.color.g = color[1]
        self.marker_object.color.b = color[2]
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        tip.z = 0
        self.marker_object.points = [tail, tip]

        # If we want it forever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0.4)
        self.markerArray.markers.append(self.marker_object)

    def call_obs(self, msg):
        # retrieve base_link 
        position = self.get_odom()

        #clear markerArray
        del self.markerArray.markers[:]
        if msg.segments:
            #x_dist consists of point and distance
            seg_dist = list(map(lambda seg:self.pnt2line(position,seg.first_point, seg.last_point),msg.segments))
            self.gen_marker(seg_dist, position, "segment")

        if msg.circles:
            circle_dist = list(map(lambda circle:(circle.center,distance(circle.center,position)),msg.circles))
            self.gen_marker(circle_dist, position, "circle")

    def gen_marker(self, dist_list, robot_pt, ns):

        ns_c = [None,None]
        if ns == "circle":
            ns_c[0] = (0.8,0,0)
            ns_c[1] = (1,0.2,0.2)
        elif ns == "segment":
            ns_c[0] = (0.8,0.8,0)
            ns_c[1] = (1,1,0.2)

        for idx,circle in enumerate(dist_list):
            
            self.create_txt_marker(cp=circle[0],index=idx,text=str(circle[1]), ns=ns,color=ns_c[0])
            self.create_arrow_marker(tail=robot_pt, tip=circle[0], index=idx, ns=ns,color=ns_c[1])

    def pnt2line(self, pnt, start, end):
        line_vec = vector(start, end)
        pnt_vec = vector(start, pnt)
        line_len = length(line_vec)
        line_unitvec = unit(line_vec)
        pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
        t = dot(line_unitvec, pnt_vec_scaled)    
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        nearest = scale(line_vec, t)
        dist = distance(nearest, pnt_vec)
        nearest = add(nearest, start)
        return (nearest,dist)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
        
        return Point(*trans)
    
    def start(self):
        while not rospy.is_shutdown():
            self.markerArray_objectlisher.publish(self.markerArray)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('obj_dist_pub', anonymous=True)
    obs_pub = ObsDistPub()
    obs_pub.start()
    rospy.spin()