#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutonomousTrolleySupervisor(Node):
    def __init__(self):
        super().__init__("autonomous_trolley_supervisor")

        self.declare_parameter("control_command_topic", "/autonomous_trolley_command")
        self.declare_parameter("mission_command_topic", "/trolley_command")
        self.declare_parameter("mission_status_topic", "/dropzone_mission_status")
        self.declare_parameter("supervisor_status_topic", "/autonomous_trolley_status")
        self.declare_parameter("goal_topic", "/goal_pose_raw")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("extra_cmd_vel_topic", "/cmd_vel_joy")
        self.declare_parameter(
            "nav_cancel_service", "/navigate_to_pose/_action/cancel_goal"
        )

        self.declare_parameter("nav_xy_tolerance", 0.12)
        self.declare_parameter("nav_yaw_tolerance_deg", 12.0)
        self.declare_parameter("nav_goal_hold_sec", 1.0)
        self.declare_parameter("nav_goal_timeout_sec", 180.0)
        self.declare_parameter("goal_message_timeout_sec", 10.0)
        self.declare_parameter("side_align_timeout_sec", 75.0)
        self.declare_parameter("drive_under_timeout_sec", 60.0)
        self.declare_parameter("fine_align_timeout_sec", 75.0)
        self.declare_parameter("drive_out_timeout_sec", 30.0)

        self.declare_parameter("detection_wait_sec", 3.0)
        self.declare_parameter("detection_retry_sec", 2.0)
        self.declare_parameter("detection_timeout_sec", 25.0)
        self.declare_parameter("max_fine_align_attempts", 3)

        self.control_command_topic = str(
            self.get_parameter("control_command_topic").value
        )
        self.mission_command_topic = str(
            self.get_parameter("mission_command_topic").value
        )
        self.supervisor_status_topic = str(
            self.get_parameter("supervisor_status_topic").value
        )

        self.nav_xy_tolerance = float(
            self.get_parameter("nav_xy_tolerance").value
        )
        self.nav_yaw_tolerance = math.radians(
            float(self.get_parameter("nav_yaw_tolerance_deg").value)
        )
        self.nav_goal_hold_sec = float(
            self.get_parameter("nav_goal_hold_sec").value
        )
        self.max_fine_align_attempts = int(
            self.get_parameter("max_fine_align_attempts").value
        )

        self.active = False
        self.state = "idle"
        self.state_started_at = time.monotonic()
        self.latest_goal: Optional[PoseStamped] = None
        self.latest_goal_at: Optional[float] = None
        self.current_goal: Optional[PoseStamped] = None
        self.latest_pose: Optional[PoseWithCovarianceStamped] = None
        self.goal_inside_since: Optional[float] = None
        self.pending_command: Optional[str] = None
        self.pending_command_at: Optional[float] = None
        self.detection_started_at: Optional[float] = None
        self.fine_align_attempt = 0
        self.last_nav_log_at = 0.0

        self.mission_command_pub = self.create_publisher(
            String, self.mission_command_topic, 10
        )
        self.status_pub = self.create_publisher(
            String, self.supervisor_status_topic, 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.extra_cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("extra_cmd_vel_topic").value), 10
        )

        self.create_subscription(
            String,
            self.control_command_topic,
            self.control_command_callback,
            10,
        )
        if self.control_command_topic != self.mission_command_topic:
            self.create_subscription(
                String,
                self.mission_command_topic,
                self.mission_command_callback,
                10,
            )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_status_topic").value),
            self.mission_status_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.goal_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )

        self.cancel_nav_client = self.create_client(
            CancelGoal, str(self.get_parameter("nav_cancel_service").value)
        )
        self.create_timer(0.10, self.control_loop)

        self.publish_status(
            "ready. Send trolley_ready_autonomous on "
            f"{self.control_command_topic}; manual trolley_ready remains unchanged."
        )

    @staticmethod
    def normalize_command(text: str) -> str:
        return text.strip().lower().replace(" ", "_").replace("-", "_")

    def control_command_callback(self, msg: String):
        command = self.normalize_command(msg.data)

        if command in ("stop", "cancel"):
            self.stop_autonomous("stopped by operator", cancel_navigation=True)
            self.publish_mission_command("stop")
            return

        if command in (
            "trolley_ready_autonomous",
            "trolleyreadyautonomous",
            "autonomous_trolley_ready",
        ):
            self.start_autonomous()
            return

        if command in ("autonomous_status", "status"):
            self.publish_status(f"state={self.state} active={self.active}")
            return

        if command in ("docking_successful", "docking_success", "docking_ok"):
            self.confirm_docking_success()
            return

        if command in ("docking_failed", "docking_not_successful"):
            self.confirm_docking_failure()
            return

        if command in (
            "undocking_successful",
            "undocking_success",
            "undocking_ok",
        ):
            self.confirm_undocking_success()
            return

        if command in ("undocking_failed", "undocking_not_successful"):
            self.confirm_undocking_failure()
            return

        if command in ("retry_fine_align", "restart_fine_align"):
            if self.active and self.state == "waiting_fine_retry_confirmation":
                self.fine_align_attempt = 0
                self.start_fine_alignment()
            else:
                self.publish_status(
                    f"ignored {command}; current state={self.state}."
                )
            return

        self.get_logger().warn(
            f'Unknown autonomous command "{command}" on '
            f"{self.control_command_topic}."
        )

    def mission_command_callback(self, msg: String):
        command = self.normalize_command(msg.data)
        if command in ("stop", "cancel"):
            self.stop_autonomous("stopped by operator", cancel_navigation=True)
            return

        # Accept autonomous controls on /trolley_command too, although the
        # dedicated control topic avoids unknown-command warnings in DropzoneMission.
        if command in (
            "trolley_ready_autonomous",
            "trolleyreadyautonomous",
            "autonomous_trolley_ready",
            "docking_successful",
            "docking_success",
            "docking_ok",
            "docking_failed",
            "docking_not_successful",
            "undocking_successful",
            "undocking_success",
            "undocking_ok",
            "undocking_failed",
            "undocking_not_successful",
            "retry_fine_align",
            "restart_fine_align",
            "autonomous_status",
        ):
            self.control_command_callback(msg)

    def start_autonomous(self):
        if self.active:
            self.stop_autonomous("restarting autonomous sequence", cancel_navigation=True)

        self.active = True
        self.fine_align_attempt = 0
        self.detection_started_at = None
        self.pending_command = None
        self.pending_command_at = None
        self.current_goal = None
        self.goal_inside_since = None
        self.set_state("waiting_approach_goal")
        self.publish_status(
            "autonomous sequence started. Sending trolley_ready and waiting for "
            "the approach goal to be reached."
        )
        self.publish_mission_command("trolley_ready")

    def mission_status_callback(self, msg: String):
        if not self.active:
            return

        status = msg.data.strip()
        status_lower = status.lower()

        if "warning: empty_trolley_detected" in status_lower:
            self.pending_command = None
            self.pending_command_at = None
            self.publish_status(
                "WARNING: empty trolley detected. Docking and delivery were skipped; "
                "waiting for the charging goal to be reached."
            )
            self.wait_for_navigation_goal("charging")
            return

        if status_lower.startswith("side_align_complete"):
            self.detection_started_at = time.monotonic()
            self.set_state("waiting_detection")
            self.schedule_command(
                "check_trolley_type",
                float(self.get_parameter("detection_wait_sec").value),
            )
            self.publish_status(
                "side alignment complete. Waiting briefly for live trolley detection."
            )
            return

        if status_lower.startswith("trolley_type_unknown_live_yolo"):
            if self.state not in ("checking_trolley_type", "waiting_detection"):
                return
            detection_elapsed = (
                time.monotonic() - self.detection_started_at
                if self.detection_started_at is not None
                else 0.0
            )
            detection_timeout = float(
                self.get_parameter("detection_timeout_sec").value
            )
            if detection_elapsed >= detection_timeout:
                self.stop_autonomous(
                    "trolley detection timed out; inspect YOLO and restart",
                    cancel_navigation=False,
                )
                return
            self.set_state("waiting_detection")
            self.schedule_command(
                "check_trolley_type",
                float(self.get_parameter("detection_retry_sec").value),
            )
            self.publish_status(
                "no live trolley type yet; waiting and retrying detection."
            )
            return

        if status_lower.startswith("trolley_type_checked"):
            self.pending_command = None
            self.pending_command_at = None
            self.set_state("driving_under")
            self.publish_status(
                f"{status} Starting drive_under automatically."
            )
            self.publish_mission_command("drive_under")
            return

        if status_lower.startswith("under_trolley"):
            self.fine_align_attempt = 0
            self.start_fine_alignment()
            return

        if status_lower.startswith("fine_align_complete"):
            self.set_state("waiting_docking_confirmation")
            self.publish_status(
                f"{status} Confirm the physical docking with docking_successful, "
                "or send docking_failed to run fine alignment again."
            )
            return

        if status_lower.startswith("fine_alignment_stopped"):
            if self.state != "fine_aligning":
                return
            if self.fine_align_attempt < self.max_fine_align_attempts:
                self.publish_status(
                    f"fine alignment attempt {self.fine_align_attempt} stopped; "
                    "retrying automatically."
                )
                self.schedule_command("fine_align", 1.0)
                self.set_state("waiting_fine_retry")
            else:
                self.set_state("waiting_fine_retry_confirmation")
                self.publish_status(
                    f"fine alignment failed {self.fine_align_attempt} times. "
                    "Send retry_fine_align after inspection, or stop."
                )
            return

        if status_lower.startswith("dropoff_goal_sent"):
            self.wait_for_navigation_goal("dropoff")
            return

        if status_lower.startswith("drive_out_done"):
            self.set_state("waiting_charging_goal")
            self.publish_status(
                "drive_out completed. Sending the robot back to charging."
            )
            self.publish_mission_command("charging")
            return

        if status_lower.startswith(
            ("side_alignment_stopped", "drive_under_stopped")
        ):
            self.stop_autonomous(
                f"mission stage failed: {status}", cancel_navigation=False
            )

    def start_fine_alignment(self):
        self.fine_align_attempt += 1
        self.set_state("fine_aligning")
        self.publish_status(
            f"starting fine alignment attempt {self.fine_align_attempt}/"
            f"{self.max_fine_align_attempts}."
        )
        self.publish_mission_command("fine_align")

    def confirm_docking_success(self):
        if not self.active or self.state != "waiting_docking_confirmation":
            self.publish_status(
                f"ignored docking_successful; current state={self.state}."
            )
            return
        self.set_state("waiting_dropoff_goal")
        self.publish_status(
            "docking confirmed. Sending the remembered laundry/trash delivery goal."
        )
        self.publish_mission_command("deliver_trolley")

    def confirm_docking_failure(self):
        if not self.active or self.state != "waiting_docking_confirmation":
            self.publish_status(
                f"ignored docking_failed; current state={self.state}."
            )
            return
        self.fine_align_attempt = 0
        self.publish_status(
            "docking marked unsuccessful. Running fine alignment again."
        )
        self.start_fine_alignment()

    def confirm_undocking_success(self):
        if not self.active or self.state != "waiting_undocking_confirmation":
            self.publish_status(
                f"ignored undocking_successful; current state={self.state}."
            )
            return
        self.set_state("driving_out")
        self.publish_status(
            "dropoff/undocking confirmed. Starting drive_out, then charging."
        )
        self.publish_mission_command("drive_out")

    def confirm_undocking_failure(self):
        if not self.active or self.state != "waiting_undocking_confirmation":
            self.publish_status(
                f"ignored undocking_failed; current state={self.state}."
            )
            return
        self.publish_status(
            "undocking marked unsuccessful. Robot remains stopped at the dropoff; "
            "inspect it, then send undocking_successful or stop."
        )

    def goal_callback(self, msg: PoseStamped):
        self.latest_goal = msg
        self.latest_goal_at = time.monotonic()

        contexts = {
            "waiting_approach_goal": "approach",
            "waiting_dropoff_goal": "dropoff",
            "waiting_charging_goal": "charging",
        }
        context = contexts.get(self.state)
        if self.active and context is not None:
            self.begin_navigation(context, msg)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = msg

    def wait_for_navigation_goal(self, context: str):
        waiting_state = f"waiting_{context}_goal"
        self.set_state(waiting_state)

        goal_age = (
            time.monotonic() - self.latest_goal_at
            if self.latest_goal_at is not None
            else math.inf
        )
        if self.latest_goal is not None and goal_age <= 2.0:
            self.begin_navigation(context, self.latest_goal)
        else:
            self.publish_status(f"waiting to receive the {context} Nav2 goal.")

    def begin_navigation(self, context: str, goal: PoseStamped):
        self.current_goal = goal
        self.goal_inside_since = None
        self.set_state(f"navigating_{context}")
        self.publish_status(
            f"navigating to {context}: x={goal.pose.position.x:+.3f}, "
            f"y={goal.pose.position.y:+.3f}, "
            f"yaw={math.degrees(quaternion_to_yaw(goal.pose.orientation)):+.1f}deg."
        )

    def control_loop(self):
        now = time.monotonic()

        if self.pending_command is not None and self.pending_command_at is not None:
            if now >= self.pending_command_at:
                command = self.pending_command
                self.pending_command = None
                self.pending_command_at = None
                if command == "check_trolley_type":
                    self.set_state("checking_trolley_type")
                    self.publish_mission_command(command)
                elif command == "fine_align":
                    self.start_fine_alignment()
                else:
                    self.publish_mission_command(command)

        if not self.active:
            return

        if self.state.startswith("navigating_"):
            self.check_navigation_goal(now)

        timeout = self.timeout_for_state()
        if timeout is not None and now - self.state_started_at > timeout:
            self.stop_autonomous(
                f"timeout while in state={self.state}", cancel_navigation=True
            )

    def check_navigation_goal(self, now: float):
        if self.current_goal is None or self.latest_pose is None:
            return

        goal_pose = self.current_goal.pose
        robot_pose = self.latest_pose.pose.pose
        dx = goal_pose.position.x - robot_pose.position.x
        dy = goal_pose.position.y - robot_pose.position.y
        distance = math.hypot(dx, dy)
        goal_yaw = quaternion_to_yaw(goal_pose.orientation)
        robot_yaw = quaternion_to_yaw(robot_pose.orientation)
        yaw_error = normalize_angle(goal_yaw - robot_yaw)

        if now - self.last_nav_log_at >= 2.0:
            self.last_nav_log_at = now
            self.get_logger().info(
                f"{self.state}: distance={distance:.3f}m "
                f"yaw_error={math.degrees(yaw_error):+.1f}deg"
            )

        inside = (
            distance <= self.nav_xy_tolerance
            and abs(yaw_error) <= self.nav_yaw_tolerance
        )
        if not inside:
            self.goal_inside_since = None
            return

        if self.goal_inside_since is None:
            self.goal_inside_since = now
            return
        if now - self.goal_inside_since < self.nav_goal_hold_sec:
            return

        context = self.state.removeprefix("navigating_")
        self.current_goal = None
        self.goal_inside_since = None

        if context == "approach":
            self.set_state("side_aligning")
            self.publish_status(
                "approach pose reached. Starting side alignment automatically."
            )
            self.publish_mission_command("side_align")
        elif context == "dropoff":
            self.set_state("waiting_undocking_confirmation")
            self.publish_status(
                "delivery goal reached. Inspect the trolley/dropoff, then confirm with "
                "undocking_successful to start drive_out. Send undocking_failed or stop "
                "if it is not safe."
            )
        elif context == "charging":
            self.active = False
            self.set_state("idle")
            self.publish_status(
                "charging position reached. Sequence complete; waiting for either "
                "manual trolley_ready or trolley_ready_autonomous."
            )

    def timeout_for_state(self) -> Optional[float]:
        if self.state in (
            "waiting_approach_goal",
            "waiting_dropoff_goal",
            "waiting_charging_goal",
        ):
            return float(self.get_parameter("goal_message_timeout_sec").value)
        if self.state.startswith("navigating_"):
            return float(self.get_parameter("nav_goal_timeout_sec").value)
        if self.state == "side_aligning":
            return float(self.get_parameter("side_align_timeout_sec").value)
        if self.state == "driving_under":
            return float(self.get_parameter("drive_under_timeout_sec").value)
        if self.state in ("fine_aligning", "waiting_fine_retry"):
            return float(self.get_parameter("fine_align_timeout_sec").value)
        if self.state == "driving_out":
            return float(self.get_parameter("drive_out_timeout_sec").value)
        return None

    def schedule_command(self, command: str, delay_sec: float):
        self.pending_command = command
        self.pending_command_at = time.monotonic() + max(0.0, delay_sec)

    def set_state(self, state: str):
        self.state = state
        self.state_started_at = time.monotonic()

    def publish_mission_command(self, command: str):
        msg = String()
        msg.data = command
        self.mission_command_pub.publish(msg)
        self.get_logger().info(
            f'Published mission command "{command}" on {self.mission_command_topic}.'
        )

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"Autonomous status: {text}")

    def stop_autonomous(self, reason: str, cancel_navigation: bool):
        was_running = self.active or self.state != "idle"
        self.active = False
        self.pending_command = None
        self.pending_command_at = None
        self.current_goal = None
        self.goal_inside_since = None
        self.detection_started_at = None
        self.set_state("idle")
        self.publish_zero_motion()
        if cancel_navigation:
            self.cancel_all_navigation_goals()
        if was_running:
            self.publish_status(f"autonomous sequence stopped: {reason}.")

    def publish_zero_motion(self):
        zero = Twist()
        self.cmd_vel_pub.publish(zero)
        self.extra_cmd_vel_pub.publish(zero)

    def cancel_all_navigation_goals(self):
        if not self.cancel_nav_client.service_is_ready():
            self.get_logger().warn(
                "Nav2 cancel service is not ready; zero Twist was published, but "
                "verify that the Nav2 goal is cancelled."
            )
            return

        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        self.cancel_nav_client.call_async(request)
        self.get_logger().warn("Requested cancellation of all NavigateToPose goals.")


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousTrolleySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_zero_motion()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
