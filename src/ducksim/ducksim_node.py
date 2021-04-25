import rospy
import tf_conversions
import tf2_ros
import math
import random
from geometry_msgs.msg import Twist, TransformStamped
from ducksim.msg import Pose
from ducksim.srv import SpawnDuck, SpawnDuckResponse
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

SIM_FREQUENCY = 100
SIM_TIME_STEP = 1.0 / SIM_FREQUENCY

MAX_LINEAR_VEL = 5.0
MAX_ANGULAR_VEL = 2 * math.pi

FRICTION = 0.5

AREA_WIDTH = 10
AREA_HEIGHT = 10

DUCK_WIDTH = 0.5

class SimObject:

    def __init__(self, name):
        self.name = name

        self.pub = rospy.Publisher('%s/pose' % name, Pose, queue_size=1)
        
        self.pose = Pose()
        self.vel = Twist()

        self.br = tf2_ros.TransformBroadcaster()

    def step(self, time):
        # Update position based on velocity
        self.pose.theta += self.vel.angular.z * time
        if self.pose.theta > 2 * math.pi:
            self.pose.theta -= 2 * math.pi
        elif self.pose.theta < 0:
            self.pose.theta += 2 * math.pi
        self.pose.x += self.vel.linear.y * math.cos(self.pose.theta) * time \
            + self.vel.linear.x * math.sin(self.pose.theta) * time
        self.pose.y += self.vel.linear.y * math.sin(self.pose.theta) * time \
            + self.vel.linear.x * math.cos(self.pose.theta) * time

        # Publish updated pose
        self.pub.publish(self.pose)

        # Publish a tf2 Transform to duck's cordinate frame
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = self.name
        t.transform.translation.x = self.pose.x
        t.transform.translation.y = self.pose.y
        t.transform.translation.z = 0
        orientation = tf_conversions.transformations.quaternion_from_euler(0, 0, self.pose.theta)
        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        self.br.sendTransform(t)

    def collide(self, x, y, vel, coeff):
        #prev = self.vel.linear.y
        if self.vel.linear.y > 0:
            self.vel.linear.y = (self.vel.linear.y + vel) * coeff
        else:
            self.vel.linear.y = -(abs(self.vel.linear.y) + vel) * coeff

        theta1 = self.pose.theta
        if theta1 > math.pi:
            theta1 -= math.pi

        theta4 = math.atan2(y - self.pose.y, x - self.pose.x)
        self.pose.theta = 2 * theta4 - 3 * theta1
        self.step(SIM_TIME_STEP)

        #rospy.loginfo("1=%.1f 4=%.1f 5=%.1f prev=%.1f other=%.1f new=%.1f" % (math.degrees(theta1), math.degrees(theta4), math.degrees(self.pose.theta), prev, vel, self.vel.linear.y))

class Ball(SimObject):
    def __init__(self, name):
        SimObject.__init__(self, name)

    def step(self, time):
        SimObject.step(self, time)

        # Apply friction
        if self.vel.linear.y > 0.2:
            self.vel.linear.y -= self.vel.linear.y**2 * FRICTION * time
        elif self.vel.linear.y < -0.2:
            self.vel.linear.y += self.vel.linear.y**2 * FRICTION * time
        else:
            self.vel.linear.y = 0

class Duck(SimObject):
    def __init__(self, name):
        SimObject.__init__(self, name)

        self.sub = rospy.Subscriber('%s/cmd_vel' % name, Twist, self.set_vel, queue_size=1)

    def set_vel(self, twist):
        if twist.linear.y > MAX_LINEAR_VEL:
            twist.linear.y = MAX_LINEAR_VEL
        elif twist.linear.y < -MAX_LINEAR_VEL:
            twist.linear.y = -MAX_LINEAR_VEL
        
        if twist.angular.z > MAX_ANGULAR_VEL:
            twist.angular.z = MAX_ANGULAR_VEL
        elif twist.angular.z < -MAX_ANGULAR_VEL:
            twist.angular.z = -MAX_ANGULAR_VEL

        self.vel.linear.x = 0
        self.vel.linear.y = twist.linear.y
        self.vel.linear.z = 0

        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = twist.angular.z

class DuckSimNode:

    def __init__(self):
        rospy.init_node('ducksim', anonymous=True)

        self.objs = []

        self.spawnDuckSrv = rospy.Service('spawn_duck', SpawnDuck, self.spawn_duck)
        self.markerPub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)

        self.num_ducks = rospy.get_param('~num_ducks', default=0)
        for i in range(self.num_ducks):
            duck = Duck("duck%d" % i)
            duck.pose.x = random.random() * AREA_WIDTH
            duck.pose.y = random.random() * AREA_HEIGHT
            duck.pose.theta = random.random() * 2 * math.pi
            duck.vel.linear.y = random.random() * MAX_LINEAR_VEL
            self.objs.append(duck)

        self.num_balls = rospy.get_param('~num_balls', default=0)
        for i in range(self.num_balls):
            ball = Ball("ball%d" % i)
            ball.pose.x = random.random() * AREA_WIDTH
            ball.pose.y = random.random() * AREA_HEIGHT
            ball.pose.theta = 0
            ball.vel.linear.y = 0
            self.objs.append(ball)

        rate = rospy.Rate(SIM_FREQUENCY)
        while not rospy.is_shutdown():
            for obj in self.objs:
                obj.step(SIM_TIME_STEP)
            self.publish_markers()

            for obj in self.objs:
                x, y = obj.pose.x, obj.pose.y
                if x > AREA_WIDTH:
                    obj.collide(x + 1, y, 0, 1.0)
                    obj.pose.x = AREA_WIDTH
                elif x < 0:
                    obj.collide(x - 1, y, 0, 1.0)
                    obj.pose.x = 0
                if y > AREA_HEIGHT:
                    obj.collide(x, y + 1, 0, 1.0)
                    obj.pose.y = AREA_HEIGHT
                elif y < 0:
                    obj.collide(x, y - 1, 0, 1.0)
                    obj.pose.y = 0

            for i in range(len(self.objs)):
                for j in range(i+1, len(self.objs)):
                    obj1, obj2 = self.objs[i], self.objs[j]
                    x1, y1 = obj1.pose.x, obj1.pose.y
                    x2, y2 = obj2.pose.x, obj2.pose.y
                    
                    if math.sqrt((x1-x2)**2 + (y1-y2)**2) < DUCK_WIDTH:
                        obj1.collide(x2, y2, abs(obj2.vel.linear.y), 0.5)
                        obj2.collide(x1, y1, abs(obj1.vel.linear.y), 0.5)
                        #rospy.loginfo("Collide %s %s" % (obj1.name, obj2.name))
            rate.sleep()

    def spawn_duck(self, req):
        name = "duck%d" % self.num_ducks
        self.num_ducks += 1
        rospy.loginfo("Spawning duck: %s" % name)

        self.objs.append(Duck(name))

        return SpawnDuckResponse(name)

    def publish_markers(self):
        markerArray = MarkerArray()
        for i, obj in enumerate(self.objs):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "/%s" % obj.name
            marker.type = marker.MESH_RESOURCE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            if isinstance(obj, Duck):
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.mesh_resource = "package://ducksim/meshes/duck.stl"
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.mesh_resource = "package://ducksim/meshes/lettuce.stl"

            q = tf_conversions.transformations.quaternion_from_euler(math.radians(90), 0, 0)
            marker.pose.orientation.w = q[0]
            marker.pose.orientation.x = q[1]
            marker.pose.orientation.y = q[2]
            marker.pose.orientation.z = q[3]
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0

            markerArray.markers.append(marker)

        # Publish the MarkerArray
        self.markerPub.publish(markerArray)

if __name__ == '__main__':
    DuckSimNode()