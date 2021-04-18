#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from turtlesim.msg import Pose
import rospy
import math
import tf

turtles = {}

def handle_turtle_pose(msg, turtlename):
    turtles[turtlename] = msg

if __name__ == '__main__':
    rospy.init_node('turtlesim_demo')

    topic = 'visualization_marker_array'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

    num_turtles = rospy.get_param('~num_turtles')

    for turtle in range(num_turtles):
        turtlename = "turtle%d" % (turtle  + 1)
        rospy.Subscriber('/%s/pose' % turtlename,
                            Pose,
                            handle_turtle_pose,
                            turtlename)

    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        i = 0
        for turtle in turtles:
            msg = turtles[turtle]
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "/%s" % turtle
            marker.type = marker.MESH_RESOURCE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            q = tf.transformations.quaternion_from_euler(math.radians(90), 0, 0)
            marker.pose.orientation.w = q[0]
            marker.pose.orientation.x = q[1]
            marker.pose.orientation.y = q[2]
            marker.pose.orientation.z = q[3]
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.mesh_resource = "package://ducksim/meshes/duck.stl"

            markerArray.markers.append(marker)

            i += 1

        # Publish the MarkerArray
        publisher.publish(markerArray)

        rospy.sleep(0.01)