import time
import json
import os
from threading import Thread

import py_trees
from py_trees.timers import Timer
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .nav_behaviour import NavigateToBin, BIN_POSES

S = py_trees.common.Status

OBJECT_CLASSES = ["plastic", "organic", "metal"]
GATE_CLASSES = ["closed_gate", "open_gate"]

JSON_PATH = os.path.expanduser("~/ros2_ws/mission.json")


# ─── Detection / condition behaviours ─────────────────────────────────

class DetectAt(py_trees.behaviour.Behaviour):
    """Waits for a /class_detected message in `accepted`. On timeout uses `default`."""

    def __init__(self, name, node, key, accepted, timeout=None, default=None, marks=None):
        super().__init__(name)
        self.node = node
        self.key = key
        self.accepted = accepted
        self.timeout = timeout
        self.default = default
        self.marks = marks
        self.active = False
        self.result = None
        self.t0 = None

        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key=key, access=py_trees.common.Access.WRITE)
        self.bb.set(key, "none")

        node.create_subscription(String, "/class_detected", self.callback, 10)

    def callback(self, msg: String):
        if self.active and msg.data in self.accepted:
            self.result = msg.data

    def initialise(self):
        self.result = None
        self.t0 = time.monotonic()          # real time, not sim time
        self.active = True

    def update(self):
        if self.result is not None:
            self.bb.set(self.key, self.result)
            return S.SUCCESS

        if self.timeout is not None and (time.monotonic() - self.t0) > self.timeout:
            if self.default is not None:
                self.bb.set(self.key, self.default)
                if self.marks is not None:
                    self.marks[f"{self.key}_fallback"] = True   # logged in JSON
                return S.SUCCESS
            return S.FAILURE
        return S.RUNNING

    def terminate(self, new_status):
        self.active = False


class IsObject(py_trees.behaviour.Behaviour):
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


def delay(name, secs):
    return Timer(name=name, duration=secs)


# ─── Timing ───────────────────────────────────────────────────────────

class Mark(py_trees.behaviour.Behaviour):
    """Stores a real-time checkpoint (or a fixed value) in the shared `marks` dict."""

    def __init__(self, name, marks, label, value=None):
        super().__init__(name)
        self.marks = marks
        self.label = label
        self.value = value

    def update(self):
        self.marks[self.label] = self.value if self.value is not None else time.monotonic()
        return S.SUCCESS


def build_entry(marks, obj, status):
    def leg(a, b):
        if a in marks and b in marks:
            return round(marks[b] - marks[a], 3)
        return None

    legs = {
        "to_cam_sec":      leg("start", "at_cam"),
        "detect_object_sec": leg("at_cam", "detected"),
        "to_pickup_sec":   leg("pickup_depart", "at_pickup"),
        "detect_gate_sec": leg("gate_detect_start", "gate_detected"),
        "via_route_sec":   leg("route_start", "route_done"),
        "to_drop_sec":     leg("drop_start", "end"),
    }
    total = leg("start", "end")

    entry = {
        "status": status,
        "object": obj,
        "route": marks.get("route"),
        "object_fallback": marks.get("object_detected_fallback", False),
        "gate_fallback": marks.get("gate_state_fallback", False),
        "total_sec": total,
        **legs,
    }

    if all(v is not None for v in legs.values()) and total is not None:
        entry["fixed_waits_sec"] = round(total - sum(legs.values()), 3)
    else:
        times = {k: v for k, v in marks.items() if isinstance(v, float) and k != "end"}
        entry["failed_after"] = max(times, key=times.get) if times else None

    entry["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
    return entry


def save_entry(json_path, entry):
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    data = {"missions": []}
    if os.path.exists(json_path):
        with open(json_path, "r") as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                pass
    data["missions"].append(entry)
    with open(json_path, "w") as f:
        json.dump(data, f, indent=4)


class MissionTimerStop(py_trees.behaviour.Behaviour):
    """Final checkpoint: computes all legs and appends the record to the JSON file."""

    def __init__(self, name, marks, json_path=JSON_PATH):
        super().__init__(name)
        self.marks = marks
        self.json_path = json_path
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key="object_detected", access=py_trees.common.Access.READ)

    def update(self):
        self.marks["end"] = time.monotonic()
        obj = self.bb.get("object_detected") or "unknown"
        entry = build_entry(self.marks, obj, "success")
        try:
            save_entry(self.json_path, entry)
        except Exception as e:
            print(f"[MissionTimerStop] JSON write error: {e}")

        print(f"\n{'=' * 50}")
        print(f"  MISSION DONE | {obj} | total {entry['total_sec']}s")
        for k, v in entry.items():
            print(f"    {k}: {v}")
        print(f"  Saved to {self.json_path}")
        print(f"{'=' * 50}\n")
        return S.SUCCESS


# ─── Mission builder ──────────────────────────────────────────────────

def make_mission(node, obj, marks):
    """pickup -> wait -> detect gate -> gate or ramp -> drop point."""

    via_gate = py_trees.composites.Sequence(name=f"ViaGate_{obj}", memory=True)
    via_gate.add_children([
        IsObject(f"GateIsOpen_{obj}", "open_gate", key="gate_state"),
        nav(node, f"gate_{obj}", "gate"),
        Mark(f"Route_gate_{obj}", marks, "route", "gate"),
    ])

    via_ramp = py_trees.composites.Sequence(name=f"ViaRamp_{obj}", memory=True)
    via_ramp.add_children([
        nav(node, f"ramp_{obj}", "ramp"),
        Mark(f"Route_ramp_{obj}", marks, "route", "ramp"),
    ])

    gate_or_ramp = py_trees.composites.Selector(name=f"GateOrRamp_{obj}", memory=True)
    gate_or_ramp.add_children([via_gate, via_ramp])

    mission = py_trees.composites.Sequence(name=f"Mission_{obj}", memory=True)
    mission.add_children([
        IsObject(f"Is_{obj}", obj),
        delay(f"small_wait_{obj}", 3),
        Mark(f"M_pickup_depart_{obj}", marks, "pickup_depart"),
        nav(node, f"{obj}_pickup", f"{obj}_pickup"),
        Mark(f"M_at_pickup_{obj}", marks, "at_pickup"),
        delay(f"Wait_pickup_{obj}", 3),
        Mark(f"M_gate_detect_start_{obj}", marks, "gate_detect_start"),
        DetectAt(f"DetectGateState_{obj}", node, "gate_state", GATE_CLASSES,
                 timeout=10.0, default="closed_gate", marks=marks),   # unsure -> ramp
        Mark(f"M_gate_detected_{obj}", marks, "gate_detected"),
        delay(f"Process_gate_state_{obj}", 2),
        Mark(f"M_route_start_{obj}", marks, "route_start"),
        gate_or_ramp,
        Mark(f"M_route_done_{obj}", marks, "route_done"),
        delay(f"final_stretch_{obj}", 2),
        Mark(f"M_drop_start_{obj}", marks, "drop_start"),
        nav(node, f"{obj}_droppoint", f"{obj}_droppoint"),
    ])
    return mission


# ─── Main ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        "bt_node",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )

    marks = {}   # shared checkpoints for this run

    go_cam = nav(node, "cam_point", "cam_point")
    detect = DetectAt("DetectObject", node, "object_detected", OBJECT_CLASSES,
                      timeout=30.0, default="plastic", marks=marks)

    sorter = py_trees.composites.Selector(name="Sorter", memory=True)
    sorter.add_children([make_mission(node, o, marks) for o in OBJECT_CLASSES])

    one_cycle = py_trees.composites.Sequence(name="OneCycle", memory=True)
    one_cycle.add_children([
        Mark("CycleStart", marks, "start"),          # clock starts at the start point
        go_cam,
        Mark("M_at_cam", marks, "at_cam"),
        detect,
        Mark("M_detected", marks, "detected"),
        sorter,
        MissionTimerStop("CycleStop", marks, JSON_PATH),   # clock stops + JSON write
    ])

    root = one_cycle
    root.setup_with_descendants()
    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # read-only client so a failed run can still be logged with its object
    main_bb = py_trees.blackboard.Client(name="main")
    main_bb.register_key(key="object_detected", access=py_trees.common.Access.READ)

    tick = 0
    try:
        while rclpy.ok():
            root.tick_once()
            if tick % 10 == 0:                       # print tree about once per second
                print(py_trees.display.unicode_tree(root, show_status=True))
            tick += 1

            if root.status == S.SUCCESS:
                print("MISSION COMPLETE")
                break

            if root.status == S.FAILURE:
                print("MISSION FAILED")
                marks["end"] = time.monotonic()
                try:
                    obj = main_bb.get("object_detected")
                except KeyError:
                    obj = "unknown"
                try:
                    save_entry(JSON_PATH, build_entry(marks, obj, "failed"))
                except Exception as e:
                    print(f"[main] JSON write error: {e}")
                break

            time.sleep(0.1)                          # 10 Hz tick
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        root.stop(S.INVALID)
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
