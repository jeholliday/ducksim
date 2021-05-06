import rospy
from geometry_msgs.msg import Twist
from ducksim.msg import Pose
from ducksim.srv import AssignTask, AssignTaskResponse, CompleteTask, CompleteTaskResponse


class HerderNode:

    def __init__(self):
        rospy.init_node('herder', anonymous=True)
        rospy.loginfo("Started herder")

        self.assign_task_srv = rospy.Service('assign_task', AssignTask, self.assign_task)
        self.complete_task_srv = rospy.Service('complete_task', CompleteTask, self.complete_task)

        rospy.spin()

    def assign_task(self, req):
        res = AssignTaskResponse()
        res.type = AssignTaskResponse.NO_MORE_TASKS
        return res

    def complete_task(self, req):
        rospy.loginfo("%s completed its task" % req.name)
        return CompleteTaskResponse()