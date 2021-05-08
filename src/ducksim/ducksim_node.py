import rospy
import tf_conversions
import tf2_ros
import math
import random
from geometry_msgs.msg import Twist, TransformStamped, Point
from ducksim.msg import Pose, WorldStatus
from ducksim.srv import SpawnDuck, SpawnDuckResponse
from ducksim.srv import MoveObject, MoveObjectRequest, MoveObjectResponse
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

SIM_FREQUENCY = 50
SIM_TIME_STEP = 1.0 / SIM_FREQUENCY

MAX_LINEAR_VEL = 2.0
MAX_ANGULAR_VEL = 2 * math.pi

FRICTION = 1.0

AREA_WIDTH = 10
AREA_HEIGHT = 10

DUCK_WIDTH = 0.3

CARRY_DISTANCE = 0.5

class SimObject:

    def __init__(self, name):
        self.name = name

        self.pub = rospy.Publisher('%s/pose' % name, Pose, queue_size=1)
        
        self.pose = Pose()
        self.vel = Twist()

        self.parent = None # This object is in inventory of parent

        self.br = tf2_ros.TransformBroadcaster()

        self.marker = Marker()
        self.marker.id = random.randint(0, 2**31)
        self.marker.header.frame_id = "/%s" % self.name
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.action = Marker.ADD
        self.marker.color.a = 1.0
        self.marker.frame_locked = True

        q = tf_conversions.transformations.quaternion_from_euler(math.radians(90), 0, 0)
        self.marker.pose.orientation.w = q[0]
        self.marker.pose.orientation.x = q[1]
        self.marker.pose.orientation.y = q[2]
        self.marker.pose.orientation.z = q[3]
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0

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
        prev = self.vel.linear.y
        if self.vel.linear.y > 0:
            self.vel.linear.y = (self.vel.linear.y + vel) * coeff
        else:
            self.vel.linear.y = -(abs(self.vel.linear.y) + vel) * coeff

        theta1 = self.pose.theta
        if theta1 > math.pi:
            theta1 -= math.pi

        theta4 = math.atan2(y - self.pose.y, x - self.pose.x)
        if prev == 0:
            self.pose.theta = theta4 - math.pi
        else:
           self.pose.theta = 2 * theta4 - 3 * theta1
        self.step(SIM_TIME_STEP)

        #rospy.loginfo("1=%.1f 4=%.1f 5=%.1f prev=%.1f other=%.1f new=%.1f" % (math.degrees(theta1), math.degrees(theta4), math.degrees(self.pose.theta), prev, vel, self.vel.linear.y))

class FrictionObject(SimObject):
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

class Ball(FrictionObject):
    def __init__(self, name):
        SimObject.__init__(self, name)

        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.scale.x = 0.01
        self.marker.scale.y = 0.01
        self.marker.scale.z = 0.01
        self.marker.mesh_resource = "package://ducksim/meshes/lettuce.stl"

class TrashCan(FrictionObject):
    def __init__(self, name):
        SimObject.__init__(self, name)

        self.marker.color.r = 0.008
        self.marker.color.g = 0.412
        self.marker.color.b = 0.157
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.mesh_resource = "package://ducksim/meshes/trash_can.dae"

class Duck(SimObject):
    def __init__(self, name):
        SimObject.__init__(self, name)

        self.sub = rospy.Subscriber('%s/cmd_vel' % name, Twist, self.set_vel, queue_size=1)

        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.scale.x = 0.005
        self.marker.scale.y = 0.005
        self.marker.scale.z = 0.005
        self.marker.mesh_resource = "package://ducksim/meshes/duck.stl"

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
        self.moveObjSrv = rospy.Service('move_obj', MoveObject, self.move_obj)
        self.markerPub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=SIM_FREQUENCY)
        self.statusPub = rospy.Publisher('world_status', WorldStatus, queue_size=SIM_FREQUENCY)

        self.num_ducks = 0
        for i in range(rospy.get_param('~num_ducks', default=0)):
            self.spawn_duck(None)

        self.num_balls = rospy.get_param('~num_balls', default=0)
        for i in range(self.num_balls):
            ball = Ball("ball%d" % i)
            ball.pose.x = random.random() * AREA_WIDTH
            ball.pose.y = random.random() * AREA_HEIGHT
            ball.pose.theta = 0
            ball.vel.linear.y = 0
            self.objs.append(ball)

        self.num_trash_cans = rospy.get_param('~num_trash_cans', default=1)
        for i in range(self.num_trash_cans):
            trash_can = TrashCan("goal%d" % i)
            trash_can.pose.x = random.random() * AREA_WIDTH
            trash_can.pose.y = random.random() * AREA_HEIGHT
            self.objs.append(trash_can)

        # Create a marker for the edge of the area
        self.edge_marker = Marker()
        self.edge_marker.id = random.randint(0, 2**31)
        self.edge_marker.header.frame_id = "/world"
        self.edge_marker.type = Marker.LINE_STRIP
        self.edge_marker.action = Marker.ADD
        self.edge_marker.scale.x = 0.2
        self.edge_marker.color.a = 1.0
        self.edge_marker.color.r = 0.0
        self.edge_marker.color.g = 0.0
        self.edge_marker.color.b = 1.0
        self.edge_marker.points.append(Point(0, 0, 0))
        self.edge_marker.points.append(Point(AREA_WIDTH, 0, 0))
        self.edge_marker.points.append(Point(AREA_WIDTH, AREA_HEIGHT, 0))
        self.edge_marker.points.append(Point(0, AREA_HEIGHT, 0))
        self.edge_marker.points.append(Point(0, 0, 0))

        count = 0
        rate = rospy.Rate(SIM_FREQUENCY)
        while not rospy.is_shutdown():
            for obj in self.objs:
                obj.step(SIM_TIME_STEP)
            
            # Publish markers less frequently
            if count == 20:
                self.publish_markers()
                self.publish_world_status()
                count = 0
            else:
                count += 1

            # Bounce objects off of edges of area
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

            # Bounce objects off of each other
            to_remove = []
            for i in range(len(self.objs)):
                for j in range(i+1, len(self.objs)):
                    obj1, obj2 = self.objs[i], self.objs[j]

                    # Don't collide objects currently being carried
                    if obj1.parent is not None or obj2.parent is not None:
                        continue

                    if isinstance(obj1, FrictionObject):
                        x1, y1 = obj1.pose.x, obj1.pose.y
                        x2, y2 = obj2.pose.x, obj2.pose.y
                        
                        # Collide objects if within DUCK_WIDTH
                        if math.sqrt((x1-x2)**2 + (y1-y2)**2) < DUCK_WIDTH:
                            # Mark balls that collide with trash_cans to be removed
                            if isinstance(obj1, TrashCan) and isinstance(obj2, Ball):
                                to_remove.append(obj2)
                                continue
                            elif isinstance(obj1, Ball) and isinstance(obj2, TrashCan):
                                to_remove.append(obj1)
                                continue

                            obj1.collide(x2, y2, abs(obj2.vel.linear.y), 0.5)
                            if isinstance(obj2, FrictionObject):
                                obj2.collide(x1, y1, abs(obj1.vel.linear.y), 0.5)
                        #rospy.loginfo("Collide %s %s" % (obj1.name, obj2.name))

            # Delete objects that collide with trash_cans
            for obj in to_remove:
                rospy.loginfo("Removing %s for colliding with trash_can" % obj.name)
                self.remove_marker(obj)

            # Set position of carried objects
            for obj in self.objs:
                if obj.parent is not None:
                    obj.pose.theta = obj.parent.pose.theta
                    obj.pose.x = obj.parent.pose.x + CARRY_DISTANCE * math.cos(obj.parent.pose.theta)
                    obj.pose.y = obj.parent.pose.y + CARRY_DISTANCE * math.sin(obj.parent.pose.theta)
            rate.sleep()

    def spawn_duck(self, req):
        name = "duck%d" % self.num_ducks
        self.num_ducks += 1
        rospy.loginfo("Spawning duck: %s" % name)

        duck = Duck(name)
        duck.pose.x = random.random() * AREA_WIDTH
        duck.pose.y = random.random() * AREA_HEIGHT
        duck.pose.theta = random.random() * 2 * math.pi
        duck.vel.linear.y = random.random() * MAX_LINEAR_VEL
        self.objs.append(duck)

        markerArray = MarkerArray()
        markerArray.markers.append(duck.marker)
        self.markerPub.publish(markerArray)

        self.publish_world_status()

        return SpawnDuckResponse(name)

    def move_obj(self, req):
        ducks = [duck for duck in self.objs if isinstance(duck, Duck) and duck.name == req.duck]
        if len(ducks) == 0:
            rospy.logerr("Unrecognized duck in move_obj: %s" % req.duck)
            return MoveObjectResponse(MoveObjectResponse.FAILED)
        duck = ducks[0]

        objs = [obj for obj in self.objs if obj.name == req.obj]
        if len(objs) == 0:
            rospy.logerr("Unrecognized obj in move_obj: %s" % req.obj)
            return MoveObjectResponse(MoveObjectResponse.FAILED)
        obj = objs[0]

        if req.action == MoveObjectRequest.PICKUP:
            if obj.parent is None:
                obj.parent = duck
                rospy.loginfo("%s picked up %s" % (duck.name, obj.name))
                return MoveObjectResponse(MoveObjectResponse.SUCCESS)

            rospy.logerr("%s tried to pickup %s but it is already being held" % (duck.name, obj.name)) 

        elif req.action == MoveObjectRequest.DROP:
            if obj.parent is duck:
                obj.parent = None
                rospy.loginfo("%s dropped %s" % (duck.name, obj.name))
                return MoveObjectResponse(MoveObjectResponse.SUCCESS)

            rospy.logerr("%s tried to drop an object it's not holding" % duck.name) 

        else:
            rospy.logerr("Unrecognized MoveObject action %d from %s" % (req.action, req.duck))

        return MoveObjectResponse(MoveObjectResponse.FAILED)

    def remove_marker(self, obj):
        try:
            # Remove marker for object
            obj.marker.type = Marker.DELETE
            markerArray = MarkerArray()
            markerArray.markers.append(obj.marker)
            self.markerPub.publish(markerArray)

            self.objs.remove(obj)

            self.publish_world_status()
        except:
            rospy.logerr("Error removing object!")

    def publish_markers(self):
        markerArray = MarkerArray()
        for obj in self.objs:
            markerArray.markers.append(obj.marker)

        # Publish marker for edge of area
        markerArray.markers.append(self.edge_marker)

        # Publish the MarkerArray
        self.markerPub.publish(markerArray)

    def publish_world_status(self):
        status = WorldStatus()

        for obj in self.objs:
            if isinstance(obj, Duck):
                status.ducks.append(obj.name)
            elif isinstance(obj, Ball):
                status.objects.append(obj.name)
            elif isinstance(obj, TrashCan):
                status.goals.append(obj.name)

        self.statusPub.publish(status)

if __name__ == '__main__':
    DuckSimNode()