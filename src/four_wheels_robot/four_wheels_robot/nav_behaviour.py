import math

import py_trees
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

BIN_POSES = {
    "plastic_pickup": (4.58027, -0.3831, 0.0),    
    "organic_pickup": (3.6183, -0.51862, 1.57),
    "metal_pickup":   (2.5426, -0.51862, 3.14),
    "plastic_droppoint": (-3.8396, 4.8104, 0.0),    
    "organic_droppoint": (-1.2289, 5.038, 1.57),
    "metal_droppoint":   (-1.367, 3.2114, 1.57),
    "entry_gate" : (-2.3882, -1.5931, 0.780),
    "gate": (-3.4296, -2.4559, 0),
    "ramp": (-3.4144, 0.4788,0)
}


class NavigateToBin(py_trees.behaviour.Behaviour):
    """Sends a NavigateToPose goal and stays RUNNING until Nav2 finishes."""

    def __init__(self, name, node, x, y, yaw):
        super().__init__(name)
        self.node = node
        self.x, self.y, self.yaw = x, y, yaw
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")

        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key="object_detected", access=py_trees.common.Access.WRITE)

        self._reset()

    def _reset(self):
        self.send_future = None
        self.result_future = None
        self.goal_handle = None

    def initialise(self):
        """Runs once each time this behaviour starts: send the goal."""
        self._reset()
        if not self.client.server_is_ready():
            self.feedback_message = "navigate_to_pose server not ready"
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y
        goal.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.send_future = self.client.send_goal_async(goal)
        self.feedback_message = f"goal sent to ({self.x}, {self.y})"

    def update(self):
        if self.send_future is None:
            return py_trees.common.Status.FAILURE

        # Step 1: wait for the server to accept or reject the goal
        if self.goal_handle is None:
            if not self.send_future.done():
                return py_trees.common.Status.RUNNING
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                self.feedback_message = "goal rejected"
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()

        # Step 2: wait for the robot to arrive
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        status = self.result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            #self.bb.object_detected = "none"          # consume the detection
            self.feedback_message = "arrived"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"navigation failed (status {status})"
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """If we are interrupted while running, cancel the Nav2 goal."""
        if (
            self.status == py_trees.common.Status.RUNNING
            and self.goal_handle is not None
            and self.result_future is not None
            and not self.result_future.done()
        ):
            self.goal_handle.cancel_goal_async()