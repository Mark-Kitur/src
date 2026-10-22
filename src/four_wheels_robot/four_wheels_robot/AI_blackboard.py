
# bt_main.py
import time
from threading import Thread

import py_trees
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .nav_behaviour import NavigateToBin, BIN_POSES

S = py_trees.common.Status

OBJECT_CLASSES = ["plastic", "organic", "metal"]
GATE_CLASSES = ["closed_gate", "open_gate"]


class DetectAt(py_trees.behaviour.Behaviour):
    """Listens to /class_detected ONLY while running.
    Writes the first accepted class to blackboard `key`.
    On timeout: writes `default` and succeeds if a default is given, else fails."""

    def __init__(self, name, node, key, accepted, timeout=None, default=None):
        super().__init__(name)
        self.node = node
        self.key = key
        self.accepted = accepted
        self.timeout = timeout
        self.default = default
        self.active = False
        self.result = None
        self.t0 = None

        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.bb.set(key, "none")                      # default so reads never KeyError

        node.create_subscription(String, "/class_detected", self.callback, 10)

    def callback(self, msg: String):
        if self.active and msg.data in self.accepted:
            self.result = msg.data

    def initialise(self):
        self.result = None                            # discard anything old
        self.t0 = self.node.get_clock().now()
        self.active = True

    def update(self):
        if self.result is not None:
            self.bb.set(self.key, self.result)
            self.feedback_message = f"{self.key} = {self.result}"
            return S.SUCCESS

        if self.timeout is not None:
            elapsed = (self.node.get_clock().now() - self.t0).nanoseconds * 1e-9
            if elapsed > self.timeout:
                if self.default is not None:
                    self.bb.set(self.key, self.default)
                    self.feedback_message = f"timeout, using default {self.default}"
                    return S.SUCCESS
                self.feedback_message = "no detection (timeout)"
                return S.FAILURE
        return S.RUNNING

    def terminate(self, new_status):
        self.active = False                           # stop listening


class IsObject(py_trees.behaviour.Behaviour):
    """Condition: SUCCESS if blackboard `key` equals `expected`."""

    def __init__(self, name, expected, key="object_detected"):
        super().__init__(name)
        self.expected = expected
        self.key = key
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key=key, access=py_trees.common.Access.READ)

    def update(self):
        return S.SUCCESS if self.bb.get(self.key) == self.expected else S.FAILURE


def nav(node, name, key):
    x, y, yaw = BIN_POSES[key]
    return NavigateToBin(f"GoTo_{name}", node, x, y, yaw)


def make_mission(node, obj):
    # gate if open, otherwise ramp
    via_gate = py_trees.composites.Sequence(name="ViaGate", memory=True)
    via_gate.add_children([
        IsObject("GateIsOpen", "open_gate", key="gate_state"),
        nav(node, "gate", "gate"),
    ])
    gate_or_ramp = py_trees.composites.Selector(name="GateOrRamp", memory=True)
    gate_or_ramp.add_children([via_gate, nav(node, "ramp", "ramp")])

    mission = py_trees.composites.Sequence(name=f"Mission_{obj}", memory=True)
    mission.add_children([
        IsObject(f"Is_{obj}", obj),
        nav(node, f"{obj}_pickup", f"{obj}_pickup"),
        nav(node, "entry_gate", "entry_gate"),
        DetectAt("DetectGateState", node, "gate_state", GATE_CLASSES,
                 timeout=10.0, default="closed_gate"),   # unsure -> ramp
        gate_or_ramp,
        nav(node, f"{obj}_droppoint", f"{obj}_droppoint"),   # last step: mission ends here
    ])
    return mission


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        "bt_node",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )

    detect_start = DetectAt("DetectAtStart", node, "object_detected", OBJECT_CLASSES)

    sorter = py_trees.composites.Selector(name="Sorter", memory=True)
    sorter.add_children([make_mission(node, o) for o in OBJECT_CLASSES])

    root = py_trees.composites.Sequence(name="Root", memory=True)
    root.add_children([detect_start, sorter])
    root.setup_with_descendants()

    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    try:
        while rclpy.ok():
            root.tick_once()
            print(py_trees.display.unicode_tree(root, show_status=True))

            if root.status == S.SUCCESS:
                print("MISSION COMPLETE")
                break
            if root.status == S.FAILURE:
                print("MISSION FAILED")
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        root.stop(S.INVALID)          # runs terminate(): cancels any live Nav2 goal
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
