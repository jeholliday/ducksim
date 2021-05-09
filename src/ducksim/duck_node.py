import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from ducksim.msg import Pose
from ducksim.srv import SpawnDuck, AssignTask, AssignTaskResponse, CompleteTask
from ducksim.srv import MoveObject, MoveObjectRequest, MoveObjectResponse

class DuckNode:

    def __init__(self):
        rospy.init_node('duck', anonymous=True)

        rospy.loginfo("Waiting for spawn_duck service...")
        rospy.wait_for_service('spawn_duck')
        try:
            spawn_duck = rospy.ServiceProxy('spawn_duck', SpawnDuck)
            resp = spawn_duck()
            self.name = resp.name
            rospy.loginfo("I am now %s" % self.name)
        except rospy.ServiceException as e:
            rospy.loginfo("spawn_duck call failed: %s"%e)
            exit(1)

        self.pose = None
        self.pose_sub = rospy.Subscriber('%s/pose' % self.name, Pose, self.updated_pose, queue_size=1)
        
        self.goal = None
        self.goal_sub = None

        self.pub = rospy.Publisher('%s/cmd_vel' % self.name, Twist, queue_size=1)

        self.cancel_task_srv = rospy.Service('%s/cancel_task' % self.name, Empty, self.cancel_task)

        self.holding = None

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.goal is None:
                self.get_task()
            rate.sleep()

    def updated_pose(self, pose):
        self.pose = pose

    def updated_goal(self, pose):
        pass

    def get_task(self):
        rospy.loginfo("Waiting for assign_task service...")
        rospy.wait_for_service('assign_task')
        try:
            assign_task = rospy.ServiceProxy('assign_task', AssignTask)
            self.goal = assign_task(self.name)
            if self.goal.type == AssignTaskResponse.NO_MORE_TASKS:
                rospy.loginfo("No more tasks. Stopping...")
                exit()
            elif self.goal.type == AssignTaskResponse.GO_TO_POINT:
                rospy.loginfo("Moving to (%.2f, %.2f)" % (self.goal.x, self.goal.y))
            elif self.goal.type == AssignTaskResponse.COLLECT_BALL:
                rospy.loginfo("Collecting: %s" % self.goal.name)
                self.goal_sub = rospy.Subscriber('%s/pose' % self.goal.name, Pose, self.updated_pose, queue_size=1)
                self.pickup_object(self.goal.name)
                self.holding = self.goal.name
            elif self.goal.type == AssignTaskResponse.DROP_BALL:
                rospy.loginfo("Dropping ball at: %s" % self.goal.name)
                self.goal_sub = rospy.Subscriber('%s/pose' % self.goal.name, Pose, self.updated_pose, queue_size=1)
                self.drop_object(self.holding)
        except rospy.ServiceException as e:
            rospy.loginfo("assign_task call failed: %s"%e)

    def cancel_task(self, req):
        pass

    def complete_task(self):
        if self.goal_sub is not None:
            self.goal_sub.unregister()
            self.goal_sub = None
        self.goal = None

        rospy.loginfo("Waiting for complete_task service...")
        rospy.wait_for_service('complete_task')
        try:
            rospy.ServiceProxy('complete_task', CompleteTask)(self.name)
            rospy.loginfo("Completed task...")
        except rospy.ServiceException as e:
            rospy.loginfo("complete_task call failed: %s"%e)

    def pickup_object(self, obj):
        rospy.loginfo("Waiting for move_obj service...")
        rospy.wait_for_service('move_obj')
        try:
            move_obj = rospy.ServiceProxy('move_obj', MoveObject)
            res = move_obj(self.name, obj, MoveObjectRequest.PICKUP)
            if res.result == MoveObjectResponse.SUCCESS:
                rospy.loginfo("Picked up: %s" % obj)
                return True
            else:
                rospy.logerr("Failed to pick up: %s" % obj)
                return False
        except rospy.ServiceException as e:
            rospy.loginfo("move_obj call failed: %s"%e)

    def drop_object(self, obj):
        rospy.loginfo("Waiting for move_obj service...")
        rospy.wait_for_service('move_obj')
        try:
            move_obj = rospy.ServiceProxy('move_obj', MoveObject)
            res = move_obj(self.name, obj, MoveObjectRequest.DROP)
            if res.result == MoveObjectResponse.SUCCESS:
                rospy.loginfo("Dropped object: %s" % obj)
                return True
            else:
                rospy.logerr("Failed to drop: %s" % obj)
                return False
        except rospy.ServiceException as e:
            rospy.loginfo("move_obj call failed: %s"%e)