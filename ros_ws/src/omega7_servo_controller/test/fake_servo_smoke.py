"""ROS integration check against fake hardware. Never connects to a robot."""

import time

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node("omega7_servo_fake_smoke")
    states = []
    faults = []

    def on_state(msg):
        if "fr3_joint2" in msg.name:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            states.append((stamp, msg.position[msg.name.index("fr3_joint2")]))

    node.create_subscription(JointState, "/joint_states", on_state, 100)
    node.create_subscription(Bool, "/omega7_servo_controller/fault_latched",
                             lambda msg: faults.append(msg.data), 10)
    enable = node.create_publisher(Bool, "/omega7_servo_controller/enable", 10)
    target_pub = node.create_publisher(
        JointState, "/omega7_servo_controller/target_joint_states", 10)
    manager = node.create_client(ListControllers, "/controller_manager/list_controllers")
    assert manager.wait_for_service(timeout_sec=8.0)
    controllers = {}
    active_deadline = time.monotonic() + 12.0
    while time.monotonic() < active_deadline:
        future = manager.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if future.done() and future.exception() is None:
            controllers = {item.name: item.state for item in future.result().controller}
            if controllers.get("omega7_servo_controller") == "active":
                break
        time.sleep(0.1)
    assert controllers.get("omega7_servo_controller") == "active", controllers
    discovery_deadline = time.monotonic() + 3.0
    while ((enable.get_subscription_count() == 0 or target_pub.get_subscription_count() == 0)
           and time.monotonic() < discovery_deadline):
        rclpy.spin_once(node, timeout_sec=0.05)
    assert enable.get_subscription_count() > 0 and target_pub.get_subscription_count() > 0
    deadline = time.monotonic() + 5.0
    while len(states) < 10 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert states, "No fake joint states"
    start = states[-1][1]
    names = [f"fr3_joint{i}" for i in range(1, 8)]
    positions = {name: 0.0 for name in names}

    def capture_full(msg):
        if all(name in msg.name for name in names):
            positions.update({name: msg.position[msg.name.index(name)] for name in names})

    full_sub = node.create_subscription(JointState, "/joint_states", capture_full, 10)
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    goal = JointState()
    goal.name = names
    goal.position = [positions[name] for name in names]
    goal.position[1] = start + 0.01
    enable.publish(Bool(data=True))
    begin = time.monotonic()
    next_publish = begin + 0.1
    while time.monotonic() - begin < 1.5:
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.monotonic() >= next_publish:
            enable.publish(Bool(data=True))
            target_pub.publish(goal)
            next_publish += 0.1
    assert states[-1][1] > start + 0.001, (start, states[-1])
    assert states[-1][1] < start + 0.012, (start, states[-1])
    speeds = [abs((b[1] - a[1]) / (b[0] - a[0]))
              for a, b in zip(states, states[1:]) if b[0] > a[0] + 0.001]
    assert max(speeds) < 0.08, max(speeds)
    goal.position[1] = start + 0.035
    begin = time.monotonic()
    next_publish = begin
    while time.monotonic() - begin < 0.3:
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.monotonic() >= next_publish:
            enable.publish(Bool(data=True))
            target_pub.publish(goal)
            next_publish += 0.1
    silent_at = time.monotonic()
    while time.monotonic() - silent_at < 2.0:
        rclpy.spin_once(node, timeout_sec=0.01)
    assert states[-1][1] < start + 0.050, "stale target exceeded bounded braking distance"
    late = [q for stamp, q in states if stamp >= states[-1][0] - 0.2]
    assert late and max(late) - min(late) < 0.001, (
        "servo did not settle after target timeout", min(late), max(late), late[-1])
    enable.publish(Bool(data=False))
    goal.position[2] = -3.0
    target_pub.publish(goal)
    fault_deadline = time.monotonic() + 1.0
    while not any(faults) and time.monotonic() < fault_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert any(faults), "invalid target did not latch a servo fault"
    print(f"fake servo active; q2 moved {states[-1][1] - start:.5f} rad; "
          f"max observed speed {max(speeds):.5f} rad/s")
    node.destroy_subscription(full_sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
