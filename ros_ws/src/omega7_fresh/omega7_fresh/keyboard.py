"""Own the terminal and launch process; send explicit desired arm state."""

import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def main():
    if not sys.stdin.isatty():
        raise SystemExit("Run from an interactive terminal; keyboard input is unavailable")
    launch_args = sys.argv[1:]
    if not launch_args:
        raise SystemExit(
            "Usage: ros2 run omega7_fresh keyboard "
            "omega7_fresh omega7_servo_teleop.launch.py [args]")
    rclpy.init()
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    node = Node("omega7_keyboard")
    client = node.create_client(SetBool, "/omega7_fresh/set_motion")
    state = {"enabled": False, "pending": None}

    def on_status(msg):
        if state["pending"] is None:
            state["enabled"] = msg.data

    node.create_subscription(Bool, "/omega7_fresh/motion_enabled", on_status, 10)
    old_terminal = termios.tcgetattr(sys.stdin.fileno())
    child = None
    try:
        child = subprocess.Popen(["ros2", "launch", *launch_args], stdin=subprocess.DEVNULL,
                                 start_new_session=True)
        tty.setcbreak(sys.stdin.fileno())
        print("SPACE: enable/disable arm | Ctrl-C: stop", flush=True)
        while child.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            future = state["pending"]
            if future is not None and future.done():
                try:
                    result = future.result()
                    if result.success:
                        state["enabled"] = state["requested"]
                    print(f"\n{result.message}", flush=True)
                except Exception as exc:
                    print(f"\nMotion service error: {exc}", flush=True)
                state["pending"] = None
            if select.select([sys.stdin], [], [], 0)[0]:
                key = os.read(sys.stdin.fileno(), 1)
                if key == b" " and state["pending"] is None:
                    if not client.service_is_ready():
                        print("\nMotion service unavailable; arm remains disabled", flush=True)
                        continue
                    state["requested"] = not state["enabled"]
                    request = SetBool.Request()
                    request.data = state["requested"]
                    state["pending"] = client.call_async(request)
        return child.returncode
    except KeyboardInterrupt:
        return 130
    finally:
        if rclpy.ok() and client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            future = client.call_async(request)
            deadline = time.monotonic() + 0.5
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_terminal)
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
