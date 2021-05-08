import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from ducksim.msg import Pose, WorldStatus
from ducksim.srv import AssignTask, AssignTaskResponse, CompleteTask, CompleteTaskResponse


class HerderNode:

    def __init__(self):
        rospy.init_node('herder', anonymous=True)
        rospy.loginfo("Started herder")

        self.objects = {}         # known objects
        self.to_be_collected = {} # objects to be collected
        self.collecting_obj  = {} # ducks collecting an object
        self.depositing_obj  = {} # ducks depositing an object
        self.goals = {}

        self.world_status_sub = rospy.Subscriber('world_status', WorldStatus, self.world_status, queue_size=1)

        self.assign_task_srv = rospy.Service('assign_task', AssignTask, self.assign_task)
        self.complete_task_srv = rospy.Service('complete_task', CompleteTask, self.complete_task)

        rospy.spin()

    def world_status(self, status):
        #rospy.loginfo("%d ducks, %d objects, %d goals" % (len(status.ducks), len(status.objects), len(status.goals)))
        for obj in status.objects:
            if obj not in self.objects:
                # Remember objects
                self.objects[obj] = None
                self.to_be_collected[obj] = None

        for goal in status.goals:
            if goal not in self.goals:
                self.goals[goal] = None
            
        to_remove = []
        for obj in self.objects:
            if obj not in status.objects:
                # Object was deposited at a goal
                rospy.loginfo("Detected deposition of: %s" % obj)
                to_remove.append(obj)
                if obj in self.to_be_collected:
                    self.to_be_collected.pop(obj)

                # Check if object was being collected by a duck
                duck = None
                for d in self.collecting_obj:
                    if self.collecting_obj[d] == obj:
                       duck = d 
                if duck != None:
                    # Object was randomly deposited while being collected
                    try:
                        cancel_task = rospy.ServiceProxy('%s/cancel_task' % duck.name, Empty)
                        cancel_task()
                        rospy.loginfo("Cancelled task for %s to collect %s" % (duck, obj))
                        self.collecting_obj.pop(duck)
                    except rospy.ServiceException as e:
                        rospy.loginfo("%s/cancel_task call failed: %s" % (duck, e))
        for obj in to_remove:
            self.objects.pop(obj)

    def assign_task(self, req):
        res = AssignTaskResponse()
        if req.name in self.collecting_obj:
            res.type = AssignTaskResponse.DROP_BALL
            res.name = next(iter(self.goals))
            self.collecting_obj.pop(req.name)
            self.depositing_obj[req.name] = res.name
            rospy.loginfo("Assigned %s to take object to %s" % (req.name, res.name))
        elif len(self.to_be_collected) == 0:
            res.type = AssignTaskResponse.NO_MORE_TASKS
            rospy.loginfo("No more tasks for %s" % req.name)
        else:
            res.type = AssignTaskResponse.COLLECT_BALL
            res.name = next(iter(self.to_be_collected))
            self.collecting_obj[req.name] = res.name
            self.to_be_collected.pop(res.name)
            rospy.loginfo("Assigned %s to collect %s" % (req.name, res.name))
        return res

    def complete_task(self, req):
        rospy.loginfo("%s completed its task" % req.name)
        if req.name in self.depositing_obj:
            self.depositing_obj.pop(req.name)
        return CompleteTaskResponse()