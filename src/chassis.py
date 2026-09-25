"""Simple position control for a RoboMaster EP chassis."""

import math
import time

from src.PID import PIDController


def angle_error(target, current):
    """Shortest signed angle from current to target, in degrees."""
    return (target - current + 180) % 360 - 180


class ChassisController:
    def __init__(self, robot, logger, settings):
        self.chassis = robot.chassis
        self.logger = logger
        self.settings = settings
        pid = settings["pid"]
        self.pid_x = PIDController(**pid["x"], max_output=settings["max_speed_m_s"])
        self.pid_y = PIDController(**pid["y"], max_output=settings["max_speed_m_s"])
        self.pid_yaw = PIDController(**pid["yaw"], max_output=settings["max_turn_deg_s"])
        self.heading_reference = None

    def reset_heading(self):
        """Use the current heading as the reference on the next move_to call."""
        self.heading_reference = None

    def stop(self):
        """Send zero speed to all three axes."""
        self.chassis.drive_speed(x=0, y=0, z=0)

    def get_pose(self):
        """Return (x metres, y metres, yaw degrees), or None if data is stale."""
        age = self.settings["sample_timeout_s"]
        position = self.logger.get_latest("position", max_age_s=age)
        attitude = self.logger.get_latest("attitude", max_age_s=age)
        if position is None or attitude is None:
            return None
        return position[0], position[1], attitude[0]

    def move_to(self, x, y, yaw=None, timeout_s=None, abort_event=None):
        """Drive toward an absolute (x, y) waypoint; optionally face yaw.

        x/y use the chassis position frame established at subscription.
        With yaw=None, hold the first move's heading across later waypoints.
        Returns the final pose. Raises TimeoutError when data or progress stops.
        """
        timeout = timeout_s if timeout_s is not None else self.settings["timeout_s"]
        if timeout <= 0:
            raise ValueError("timeout_s must be positive")
        for pid in (self.pid_x, self.pid_y, self.pid_yaw):
            pid.reset()

        period = self.settings["control_period_s"]
        deadline = time.monotonic() + timeout
        previous = time.monotonic()
        target_yaw = yaw
        if yaw is not None:
            self.heading_reference = yaw
        try:
            while time.monotonic() < deadline:
                if abort_event is not None and abort_event.is_set():
                    raise RuntimeError("motion aborted because exploration telemetry failed")
                pose = self.get_pose()
                if pose is None:
                    raise TimeoutError("position or attitude data is missing/stale")
                current_x, current_y, current_yaw = pose
                if target_yaw is None and self.settings.get("hold_heading", True):
                    if self.heading_reference is None:
                        self.heading_reference = current_yaw
                    target_yaw = self.heading_reference
                error_x = x - current_x
                error_y = y - current_y
                heading_error = 0 if target_yaw is None else angle_error(target_yaw, current_yaw)

                arrived = math.hypot(error_x, error_y) <= self.settings["position_tolerance_m"]
                facing = target_yaw is None or abs(heading_error) <= self.settings["angle_tolerance_deg"]
                if arrived and facing:
                    return pose

                now = time.monotonic()
                dt = max(now - previous, 0.001)
                previous = now
                # Compute speeds in the fixed position frame, then rotate into
                # the robot's forward/right frame used by drive_speed().
                vx_world = self.pid_x.compute(error_x, dt)
                vy_world = self.pid_y.compute(error_y, dt)
                heading_rad = math.radians(current_yaw)
                vx_robot = vx_world * math.cos(heading_rad) + vy_world * math.sin(heading_rad)
                vy_robot = -vx_world * math.sin(heading_rad) + vy_world * math.cos(heading_rad)
                speed = math.hypot(vx_robot, vy_robot)
                limit = self.settings["max_speed_m_s"]
                if speed > limit:
                    vx_robot *= limit / speed
                    vy_robot *= limit / speed
                turn = 0 if facing else self.pid_yaw.compute(heading_error, dt)
                self.chassis.drive_speed(x=vx_robot, y=vy_robot, z=turn)
                time.sleep(period)
            raise TimeoutError(f"waypoint ({x}, {y}, {yaw}) not reached within {timeout} s")
        finally:
            self.stop()
