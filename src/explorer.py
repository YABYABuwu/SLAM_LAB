"""Depth-first frontier traversal over a SLAM occupancy grid."""

import math
import threading
import time

from src.slam import CellWallGrid, _wrap_degrees


class DFSExplorer:
    """Traverse shared open cell borders observed with the gimbal ToF."""

    DIRECTIONS = ((1, 0), (0, -1), (-1, 0), (0, 1))

    def __init__(self, chassis, gimbal, logger, slam_map, settings):
        self.chassis = chassis
        self.gimbal = gimbal
        self.logger = logger
        self.map = slam_map
        self.settings = settings
        self.status = "ready"
        self.error = None
        self.visited = set()
        self.stack = []
        self.moves = 0
        self.base_pose = None
        self.current_cell = (0, 0)
        self.wall_grid = CellWallGrid(
            settings["step_m"], min(1000, math.ceil(math.hypot(
                settings["map"]["width_m"], settings["map"]["height_m"]
            ) / settings["step_m"]) + 1)
        )
        self.slam_worker = None
        self.lock = threading.RLock()

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "error": self.error,
                "visited": [list(node) for node in sorted(self.visited)],
                "stack": [list(node) for node in self.stack],
                "visited_points": [list(self._to_map(node)) for node in sorted(self.visited)]
                if self.base_pose is not None else [],
                "stack_points": [list(self._to_map(node)) for node in self.stack]
                if self.base_pose is not None else [],
                "moves": self.moves,
                "max_nodes": self.settings["max_nodes"],
                "cell_grid": self.wall_grid.snapshot(self.base_pose, self.current_cell),
            }

    def _set_status(self, status, error=None):
        with self.lock:
            self.status, self.error = status, error
            snapshot = self.snapshot()
        self.map.set_exploration_state(snapshot)

    def _to_map(self, node):
        step = self.settings["step_m"]
        u, v = node[0] * step, node[1] * step
        x0, y0, yaw = self.base_pose
        angle = math.radians(yaw)
        return (x0 + u * math.cos(angle) - v * math.sin(angle),
                y0 + u * math.sin(angle) + v * math.cos(angle))

    def _current_yaw(self):
        sample = self.logger.get_sample("attitude", max_age_s=self.settings["sample_timeout_s"])
        if sample is None:
            raise TimeoutError("attitude data is missing or stale during exploration")
        return float(sample[0][0])

    def _gimbal_sample(self):
        sample = self.logger.get_sample("gimbal", max_age_s=self.settings["sample_timeout_s"])
        if sample is None or len(sample[0]) < 2:
            raise TimeoutError("gimbal angle data is missing or stale during exploration")
        return float(sample[0][1]), float(sample[1])

    @staticmethod
    def _command_yaw(target_relative_yaw, current_relative_yaw):
        """Choose the nearest equivalent chassis-relative SDK yaw target."""
        candidates = (target_relative_yaw - 360.0, target_relative_yaw,
                      target_relative_yaw + 360.0)
        reachable = [angle for angle in candidates if -250.0 <= angle <= 250.0]
        if not reachable:
            raise RuntimeError("gimbal cannot reach the requested direction within its yaw limits")
        return min(reachable, key=lambda angle: abs(angle - current_relative_yaw))

    def _scan_for_direction(self, delta):
        """Point the single ToF toward a candidate cell and wait for its new scan."""
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise RuntimeError(worker_status["error"])
        world_yaw = self.base_pose[2] + math.degrees(math.atan2(delta[1], delta[0]))
        body_yaw = self._current_yaw()
        sensor = self.settings["sensor"]
        target_yaw = _wrap_degrees(
            world_yaw - body_yaw - float(sensor["yaw_offset_deg"])
        )
        current_yaw, _ = self._gimbal_sample()
        command_yaw = self._command_yaw(target_yaw, current_yaw)
        request_time = time.time()
        tolerance = self.settings["gimbal"]["angle_tolerance_deg"]
        use_recenter = (abs(target_yaw) <= tolerance and
                        abs(self.settings["gimbal"]["pitch_deg"]) <= 0.1)
        previous_status = self.status
        self._set_status("recentering" if use_recenter else "scanning")
        if use_recenter:
            command_yaw = 0.0
            action = self.gimbal.recenter(
                pitch_speed=30,
                yaw_speed=self.settings["gimbal"]["yaw_speed_deg_s"],
            )
        else:
            action = self.gimbal.moveto(
                pitch=self.settings["gimbal"]["pitch_deg"],
                yaw=command_yaw,
                pitch_speed=30,
                yaw_speed=self.settings["gimbal"]["yaw_speed_deg_s"],
            )

        expected_motion_s = abs(command_yaw - current_yaw) / self.settings["gimbal"]["yaw_speed_deg_s"]
        deadline = time.monotonic() + (
            max(self.settings["gimbal"]["move_timeout_s"], expected_motion_s + 0.5) +
            self.settings["gimbal"]["scan_timeout_s"]
        )
        aligned = False
        measured_yaw = current_yaw
        scan_yaw = self.map.latest_gimbal_yaw_deg
        action_complete = action is None
        while time.monotonic() < deadline:
            worker_status = self.slam_worker.status() if self.slam_worker is not None else None
            if worker_status is not None and worker_status["error"]:
                raise RuntimeError(worker_status["error"])
            action_state = getattr(action, "state", None)
            if action_state in ("action_failed", "action_rejected", "action_exception", "action_aborted"):
                raise RuntimeError(f"gimbal {('recenter' if use_recenter else 'moveto')} failed: {action_state}")
            measured_yaw, angle_timestamp = self._gimbal_sample()
            aligned = (angle_timestamp > request_time and
                       abs(_wrap_degrees(target_yaw - measured_yaw)) <= tolerance)

            scan_timestamp = self.map.latest_scan_timestamp
            scan_yaw = self.map.latest_gimbal_yaw_deg
            scan_range = self.map.latest_range_mm
            if (aligned and scan_timestamp is not None and
                    scan_timestamp > request_time and
                    scan_yaw is not None and
                    abs(_wrap_degrees(target_yaw - scan_yaw)) <= tolerance and
                    scan_range is not None and
                    time.time() - scan_timestamp <= self.settings["sample_timeout_s"] * 2):
                if not action_complete:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not action.wait_for_completed(timeout=remaining):
                        raise TimeoutError("gimbal SDK action did not complete after an aligned ToF scan")
                    if not action.has_succeeded:
                        raise RuntimeError(f"gimbal SDK action ended as {action.state}")
                    action_complete = True
                    continue
                self._set_status(previous_status)
                return float(scan_range), world_yaw
            time.sleep(0.03)
        raise TimeoutError(
            f"gimbal scan timed out: relative target {target_yaw:.1f}°, "
            f"SDK command {command_yaw:.1f}°, relative actual {measured_yaw:.1f}°, "
            f"last mapped scan yaw {scan_yaw if scan_yaw is not None else 'none'}°, "
            f"action state {getattr(action, 'state', 'unavailable')}"
        )

    def _can_step(self, node, destination):
        delta = (destination[0] - node[0], destination[1] - node[1])
        with self.lock:
            self.wall_grid.checks.pop((tuple(node), delta), None)
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise RuntimeError(worker_status["error"])
        pose = self.map.pose
        if pose is None:
            return False
        measured_mm, world_yaw = self._scan_for_direction(delta)
        try:
            measured_m = measured_mm / 1000.0
        except (TypeError, ValueError):
            return False
        if not math.isfinite(measured_m) or measured_m <= 0:
            return False
        body_yaw = math.radians(self._current_yaw())
        move_yaw = math.radians(world_yaw)
        sensor_config = self.settings["sensor"]
        pivot_projection = (
            (sensor_config["pivot_x_m"] * math.cos(body_yaw) -
             sensor_config["pivot_y_m"] * math.sin(body_yaw)) * math.cos(move_yaw) +
            (sensor_config["pivot_x_m"] * math.sin(body_yaw) +
             sensor_config["pivot_y_m"] * math.cos(body_yaw)) * math.sin(move_yaw)
        )
        sensor_offset = (
            pivot_projection + sensor_config["offset_from_yaw_axis_m"] *
            math.cos(math.radians(sensor_config["offset_yaw_deg"]))
        )
        pose = self.map.pose
        center = self._to_map(node)
        pose_along = ((pose[0] - center[0]) * math.cos(move_yaw) +
                      (pose[1] - center[1]) * math.sin(move_yaw))
        required = (self.settings["step_m"] - pose_along - sensor_offset +
                    self.settings["robot_radius_m"] + self.settings["clearance_margin_m"])
        with self.lock:
            self.wall_grid.observe(
                node, delta, measured_m + sensor_offset + pose_along,
                measured_mm, required, self.map.latest_scan_timestamp,
            )
            allowed = self.wall_grid.can_cross(node, delta)
        self.map.set_exploration_state(self.snapshot())
        return allowed

    def _scan_all_directions(self, node):
        """Complete a fresh scan of all four neighboring directions before moving."""
        previous_status = self.status
        for index, delta in enumerate(self.DIRECTIONS, 1):
            self._set_status(f"scanning_{index}_of_{len(self.DIRECTIONS)}")
            neighbor = (node[0] + delta[0], node[1] + delta[1])
            self._can_step(node, neighbor)
        with self.lock:
            clear = {delta: self.wall_grid.can_cross(node, delta) for delta in self.DIRECTIONS}
        self._set_status(previous_status)
        return clear

    def _move(self, destination):
        x, y = self._to_map(destination)
        previous_scan = self.map.latest_scan_timestamp or 0.0
        self.chassis.move_to(x, y, yaw=self.base_pose[2],
                             abort_event=self.slam_worker.abort_event)
        with self.lock:
            self.moves += 1
            self.current_cell = tuple(destination)
        deadline = time.monotonic() + self.settings["sample_timeout_s"] * 2
        while time.monotonic() < deadline:
            if (self.map.latest_scan_timestamp or 0.0) > previous_scan:
                return
            time.sleep(0.05)
        raise TimeoutError("no fresh SLAM scan arrived after moving to a cell")

    def run(self, slam_worker):
        """Explore open grid edges with fresh clearance, then backtrack."""
        self.slam_worker = slam_worker
        try:
            self._set_status("starting")
            slam_worker.wait_ready(timeout_s=self.settings["sample_timeout_s"] * 4)
            pose = self.map.pose
            if pose is None:
                raise RuntimeError("SLAM has no initial pose")
            with self.lock:
                self.base_pose = tuple(pose)
                self.wall_grid.cells.add((0, 0))
            root = (0, 0)
            with self.lock:
                self.stack = [root]
                self.visited = {root}
            self._set_status("exploring")
            while self.stack:
                with self.lock:
                    current = self.stack[-1]
                    node_count = len(self.visited)
                if node_count >= self.settings["max_nodes"]:
                    self._set_status("node_limit_returning")
                    while len(self.stack) > 1:
                        with self.lock:
                            child, parent = self.stack[-1], self.stack[-2]
                        clear = self._scan_all_directions(child)
                        back = (parent[0] - child[0], parent[1] - child[1])
                        if not clear[back] or not self._can_step(child, parent):
                            raise RuntimeError("DFS cannot safely return to the start cell")
                        self._set_status("returning_to_start")
                        self._move(parent)
                        with self.lock:
                            self.stack.pop()
                    self._set_status("node_limit_returned")
                    return self.snapshot()

                clear = self._scan_all_directions(current)
                next_node = None
                for delta in self.DIRECTIONS:
                    neighbor = (current[0] + delta[0], current[1] + delta[1])
                    if neighbor in self.visited:
                        continue
                    if clear[delta] and self._can_step(current, neighbor):
                        next_node = neighbor
                        break

                if next_node is not None:
                    self._set_status(f"moving_to_{next_node[0]}_{next_node[1]}")
                    self._move(next_node)
                    with self.lock:
                        self.stack.append(next_node)
                        self.visited.add(next_node)
                    self._set_status("exploring")
                    continue

                if self.moves == 0 and len(self.stack) == 1:
                    self._set_status(
                        "no_safe_direction",
                        "กริดยังไม่มีขอบทางเปิดที่ผ่านระยะเผื่อตัวหุ่นครบ",
                    )
                    return self.snapshot()

                with self.lock:
                    finished = self.stack.pop()
                    parent = self.stack[-1] if self.stack else None
                if parent is not None:
                    back = (parent[0] - finished[0], parent[1] - finished[1])
                    if not clear[back] or not self._can_step(finished, parent):
                        raise RuntimeError("DFS backtrack path is no longer clear")
                    self._set_status(f"backtracking_to_{parent[0]}_{parent[1]}")
                    self._move(parent)
                    self._set_status("exploring")

            self._set_status("completed")
            return self.snapshot()
        except Exception as error:
            self._set_status("failed", str(error))
            raise
