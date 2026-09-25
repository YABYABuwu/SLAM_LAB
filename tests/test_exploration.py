import io
import copy
import csv
import json
import math
import tempfile
import threading
import time
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace

from src.config_loader import load_config
from src.dashboard import Dashboard
from src.explorer import DFSExplorer
from src.logger import SensorLogger
from src.slam import OccupancyGridSLAM, SlamWorker


class FakeLogger:
    def __init__(self):
        self.samples = {}

    def set(self, name, values, timestamp=None):
        self.samples[name] = (tuple(values), time.time() if timestamp is None else timestamp)

    def get_sample(self, name, max_age_s=None):
        return self.samples.get(name)


class FakeSDKModule:
    def __init__(self):
        self.callbacks = {}

    def __getattr__(self, name):
        if name.startswith("sub_"):
            return lambda **options: self._subscribe(name, options)
        if name.startswith("unsub_"):
            return lambda: self.callbacks.pop(name.replace("unsub_", "sub_"), None)
        raise AttributeError(name)

    def _subscribe(self, name, options):
        self.callbacks[name] = options["callback"]
        return True


class FakeSlamWorker:
    def __init__(self):
        self.abort_event = threading.Event()

    def wait_ready(self, timeout_s=5.0):
        return None

    def status(self):
        return {"running": True, "ready": True, "error": None}


class SimulatedGimbal:
    def __init__(self, slam_map, logger, range_provider):
        self.slam_map = slam_map
        self.logger = logger
        self.range_provider = range_provider
        self.yaw = 0.0
        self.pitch = 0.0
        self.scan_time = time.time()
        self.commands = []
        self.logger.set("gimbal", (0, 0, 0, 0), self.scan_time)

    def _range(self, pose):
        return (self.range_provider(pose, self.yaw)
                if callable(self.range_provider) else self.range_provider)

    def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
        self.pitch, self.yaw = float(pitch), float(yaw)
        self.commands.append((self.pitch, self.yaw))
        self.scan_time = max(time.time(), self.scan_time) + 0.002
        self.logger.set("gimbal", (self.pitch, self.yaw, self.pitch, self.yaw),
                        self.scan_time)
        pose = tuple(self.slam_map.pose)
        self.slam_map.update(pose, self._range(pose), timestamp=self.scan_time,
                             gimbal_yaw_deg=self.yaw)


class SimulatedChassis:
    def __init__(self, slam_map, logger, gimbal, ranges):
        self.slam_map = slam_map
        self.logger = logger
        self.gimbal = gimbal
        self.ranges = ranges
        self.scan_time = time.time()
        self.commands = []

    def move_to(self, x, y, yaw=None, abort_event=None):
        if abort_event is not None and abort_event.is_set():
            raise RuntimeError("simulated motion aborted")
        pose = (x, y, yaw or 0.0)
        self.commands.append(pose)
        self.scan_time += 0.01
        self.logger.set("attitude", (pose[2], 0, 0), self.scan_time)
        readings = (self.ranges(pose, self.gimbal.yaw)
                    if callable(self.ranges) else self.ranges)
        self.slam_map.update(pose, readings, timestamp=self.scan_time,
                             gimbal_yaw_deg=self.gimbal.yaw)
        return pose


class ExplorationTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.settings = self.config["exploration"]
        self.ranges = [2000, 2000, 2000, 2000]

    def make_map(self):
        slam_map = OccupancyGridSLAM(self.settings)
        timestamp = time.time()
        slam_map.update((0, 0, 0), self.ranges, timestamp=timestamp)
        slam_map.update((0, 0, 0), self.ranges, timestamp=timestamp + 0.01)
        return slam_map

    def room_range(self, pose, gimbal_yaw, half_extent=1.6):
        x, y, yaw = pose
        yaw_rad = math.radians(yaw)
        sensor = self.settings["sensor"]
        beam = math.radians(yaw + gimbal_yaw + sensor["yaw_offset_deg"])
        sx = (x + sensor["pivot_x_m"] * math.cos(yaw_rad) -
              sensor["pivot_y_m"] * math.sin(yaw_rad) +
              sensor["offset_from_yaw_axis_m"] * math.cos(beam))
        sy = (y + sensor["pivot_x_m"] * math.sin(yaw_rad) +
              sensor["pivot_y_m"] * math.cos(yaw_rad) +
              sensor["offset_from_yaw_axis_m"] * math.sin(beam))
        dx, dy = math.cos(beam), math.sin(beam)
        intersections = []
        if dx > 1e-9:
            intersections.append((half_extent - sx) / dx)
        elif dx < -1e-9:
            intersections.append((-half_extent - sx) / dx)
        if dy > 1e-9:
            intersections.append((half_extent - sy) / dy)
        elif dy < -1e-9:
            intersections.append((-half_extent - sy) / dy)
        return round(min(distance for distance in intersections if distance > 0) * 1000)

    def make_dashboard(self, slam_map):
        logger = SimpleNamespace(stream_settings={}, dropped_rows=0)
        robot = SimpleNamespace(camera=SimpleNamespace())
        return Dashboard(robot, logger, self.config["dashboard"], slam_map=slam_map)

    def request(self, dashboard, method, path, body=b""):
        handler_type = dashboard._handler_class()
        handler = handler_type.__new__(handler_type)
        handler.path = path
        handler.wfile = io.BytesIO()
        handler.rfile = io.BytesIO(body)
        handler.headers = {"Content-Length": str(len(body))}
        handler.send_response = lambda code: None
        handler.send_header = lambda name, value: None
        handler.end_headers = lambda: None
        handler.send_error = lambda code, message=None: self.fail(
            f"dashboard returned HTTP {code}: {message}"
        )
        getattr(handler, method)()
        return handler.wfile.getvalue()

    def test_dashboard_map_json_round_trip_and_ros_export(self):
        slam_map = self.make_map()
        dashboard = self.make_dashboard(slam_map)

        api_map = json.loads(self.request(dashboard, "do_GET", "/api/map"))
        self.assertEqual(api_map["format"], "robomaster-occupancy-grid")
        self.assertTrue(api_map["has_map"])
        self.assertEqual(api_map["scan_count"], 2)
        self.assertGreater(api_map["counts"]["free_cells"], 0)
        self.assertGreater(api_map["counts"]["occupied_cells"], 0)
        self.assertTrue(api_map["trajectory"])
        self.assertEqual(api_map["sensor_model"]["type"], "single_gimbal_tof")
        self.assertAlmostEqual(api_map["sensor_model"]["offset_from_yaw_axis_m"], 0.075)
        self.assertEqual(len(api_map["data"]), api_map["width"] * api_map["height"])

        exported = json.loads(self.request(
            dashboard, "do_GET", "/api/map/export?format=json"
        ))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "map.json"
            path.write_text(json.dumps(exported), encoding="utf-8")
            restored = OccupancyGridSLAM(self.settings)
            restored.load_file(path)
            self.assertEqual(restored.to_dict()["data"], exported["data"])
            self.assertEqual(restored.to_dict()["pose"], exported["pose"])
            self.assertEqual(restored.to_dict()["trajectory"], exported["trajectory"])
            self.assertEqual(restored.to_dict()["sensor_model"], exported["sensor_model"])

        self.request(dashboard, "do_POST", "/api/map/import", json.dumps(exported).encode())
        self.assertEqual(dashboard.map_snapshot()["data"], exported["data"])

        archive_bytes = self.request(dashboard, "do_GET", "/api/map/export?format=ros")
        with zipfile.ZipFile(io.BytesIO(archive_bytes)) as archive:
            self.assertEqual(set(archive.namelist()), {"map.yaml", "map.pgm", "slam.json"})
            self.assertIn(b"resolution:", archive.read("map.yaml"))
            self.assertTrue(archive.read("map.pgm").startswith(b"P5\n"))
            self.assertEqual(json.loads(archive.read("slam.json"))["data"], exported["data"])

    def test_gimbal_tof_beam_uses_yaw_axis_offset_and_selected_channel(self):
        settings = copy.deepcopy(self.settings)
        settings["sensor"]["tof_channel"] = 2
        slam_map = OccupancyGridSLAM(settings)
        beam, = slam_map._beam_geometry((1.0, 2.0, 90.0),
                                        [111, 222, 333, 444], -90.0)
        sx, sy, ex, ey, hit = beam
        self.assertAlmostEqual(sx, 1.075)
        self.assertAlmostEqual(sy, 2.0)
        self.assertAlmostEqual(ex, 1.297)
        self.assertAlmostEqual(ey, 2.0)
        self.assertTrue(hit)
        self.assertTrue(slam_map.scan_is_valid([111, 222, 333, 444], -90))

        settings["sensor"]["offset_yaw_deg"] = 90.0
        lateral_map = OccupancyGridSLAM(settings)
        lateral_beam, = lateral_map._beam_geometry((0, 0, 0), 222, 0)
        self.assertAlmostEqual(lateral_beam[0], 0.0)
        self.assertAlmostEqual(lateral_beam[1], 0.075)
        self.assertAlmostEqual(lateral_beam[2], 0.222)
        self.assertAlmostEqual(lateral_beam[3], 0.075)

    def test_dfs_visits_nodes_and_returns_to_start(self):
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, self.ranges)
        chassis = SimulatedChassis(slam_map, logger, gimbal, self.ranges)
        worker = FakeSlamWorker()
        settings = dict(self.settings)
        settings["max_nodes"] = 4
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)

        result = explorer.run(worker)

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertEqual(len(result["visited"]), 4)
        self.assertEqual(result["stack"], [[0, 0]])
        self.assertEqual(result["moves"], 6)
        self.assertAlmostEqual(slam_map.pose[0], 0.0)
        self.assertAlmostEqual(slam_map.pose[1], 0.0)
        self.assertEqual(slam_map.exploration_state["status"], "node_limit_returned")

    def test_dfs_does_not_move_when_all_sensor_ranges_are_blocked(self):
        blocked_ranges = [300, 300, 300, 300]
        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), blocked_ranges)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, blocked_ranges)
        chassis = SimulatedChassis(slam_map, logger, gimbal, blocked_ranges)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, self.settings)

        result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["visited"], [[0, 0]])
        self.assertEqual(result["moves"], 0)
        self.assertEqual(chassis.commands, [])

    def test_dfs_explores_and_backtracks_inside_a_simulated_room(self):
        range_provider = lambda pose, yaw: self.room_range(pose, yaw, half_extent=1.6)
        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), range_provider((0, 0, 0), 0))
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, range_provider)
        chassis = SimulatedChassis(slam_map, logger, gimbal, range_provider)
        settings = dict(self.settings)
        settings["max_nodes"] = 8
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)

        result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertEqual(len(result["visited"]), 8)
        self.assertGreater(result["moves"], len(result["visited"]) - 1)
        self.assertEqual(result["stack"], [[0, 0]])
        self.assertAlmostEqual(slam_map.pose[0], 0.0)
        self.assertAlmostEqual(slam_map.pose[1], 0.0)
        self.assertGreater(slam_map.to_dict()["counts"]["occupied_cells"], 0)

    def test_slam_worker_uses_logger_streams_and_rejects_unsafe_status(self):
        timestamp = time.time()
        logger = FakeLogger()
        logger.set("position", (0, 0, 0), timestamp)
        logger.set("attitude", (0, 0, 0), timestamp)
        logger.set("tof", self.ranges, timestamp)
        logger.set("gimbal", (0, 0, 0, 0), timestamp)
        logger.set("status", (0,) * 10, timestamp)
        slam_map = OccupancyGridSLAM(self.settings)
        worker = SlamWorker(logger, slam_map, self.settings)
        worker.start()
        try:
            worker.wait_ready(timeout_s=2)
            self.assertTrue(worker.status()["ready"])
            self.assertEqual(slam_map.scan_count, 1)
            self.assertEqual(slam_map.latest_gimbal_yaw_deg, 0)
        finally:
            worker.stop()

        channel_settings = copy.deepcopy(self.settings)
        channel_settings["sensor"]["tof_channel"] = 3
        channel_logger = FakeLogger()
        channel_logger.set("position", (0, 0, 0), timestamp)
        channel_logger.set("attitude", (0, 0, 0), timestamp)
        channel_logger.set("tof", (111, 222, 333, 444), timestamp)
        channel_logger.set("gimbal", (0, 45, 0, 45), timestamp)
        channel_logger.set("status", (0,) * 10, timestamp)
        channel_map = OccupancyGridSLAM(channel_settings)
        channel_worker = SlamWorker(channel_logger, channel_map, channel_settings)
        channel_worker.start()
        try:
            channel_worker.wait_ready(timeout_s=2)
            self.assertEqual(channel_map.latest_range_mm, 333)
            self.assertEqual(channel_map.latest_gimbal_yaw_deg, 45)
        finally:
            channel_worker.stop()

        unsafe_logger = FakeLogger()
        unsafe_logger.set("position", (0, 0, 0), timestamp)
        unsafe_logger.set("attitude", (0, 0, 0), timestamp)
        unsafe_logger.set("tof", self.ranges, timestamp)
        unsafe_logger.set("gimbal", (0, 0, 0, 0), timestamp)
        unsafe_status = [0] * 10
        unsafe_status[5] = 1
        unsafe_logger.set("status", unsafe_status, timestamp)
        unsafe_worker = SlamWorker(
            unsafe_logger, OccupancyGridSLAM(self.settings), self.settings
        )
        unsafe_worker.start()
        try:
            with self.assertRaisesRegex(RuntimeError, "safety status flag 5"):
                unsafe_worker.wait_ready(timeout_s=2)
            self.assertTrue(unsafe_worker.abort_event.is_set())
        finally:
            unsafe_worker.stop()

    def test_real_logger_streams_feed_slam_and_preserve_four_tof_csv_columns(self):
        chassis_module = FakeSDKModule()
        sensor_module = FakeSDKModule()
        gimbal_module = FakeSDKModule()
        robot = SimpleNamespace(chassis=chassis_module, sensor=sensor_module,
                                gimbal=gimbal_module)
        streams = {
            "position": {"enabled": True, "save": False, "frequency_hz": 10},
            "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
            "status": {"enabled": True, "save": False, "frequency_hz": 5},
            "tof": {"enabled": True, "save": True, "frequency_hz": 5},
            "gimbal": {"enabled": True, "save": False, "frequency_hz": 5},
        }
        with tempfile.TemporaryDirectory() as temp:
            logger = SensorLogger(robot, {"directory": temp, "streams": streams})
            logger.start()
            try:
                chassis_module.callbacks["sub_position"]((0, 0, 0))
                chassis_module.callbacks["sub_attitude"]((0, 0, 0))
                chassis_module.callbacks["sub_status"]((0,) * 10)
                sensor_module.callbacks["sub_distance"](tuple(self.ranges))
                gimbal_module.callbacks["sub_angle"]((0, 0, 0, 0))
                slam_map = OccupancyGridSLAM(self.settings)
                worker = SlamWorker(logger, slam_map, self.settings)
                worker.start()
                try:
                    worker.wait_ready(timeout_s=2)
                    self.assertEqual(slam_map.scan_count, 1)
                    self.assertEqual(slam_map.latest_range_mm, self.ranges[0])
                    self.assertEqual(logger.get_latest("tof"), tuple(self.ranges))
                    history = logger.get_history_since()
                    self.assertEqual(history["streams"]["tof"][0][2], tuple(self.ranges))
                finally:
                    worker.stop()
            finally:
                logger.stop()

            with (logger.run_dir / "tof.csv").open(newline="", encoding="utf-8") as file:
                rows = list(csv.reader(file))
            self.assertEqual(rows[0], [
                "timestamp", "elapsed_s", "tof_1_mm", "tof_2_mm", "tof_3_mm", "tof_4_mm"
            ])
            self.assertEqual(len(rows[1]), 6)

    def test_map_import_rejects_invalid_schema_and_live_replacement(self):
        slam_map = self.make_map()
        valid_document = slam_map.to_dict()
        target = OccupancyGridSLAM(self.settings)

        boolean_version = copy.deepcopy(valid_document)
        boolean_version["version"] = True
        with self.assertRaisesRegex(ValueError, "unsupported map file version"):
            target.load_dict(boolean_version)

        truncated_grid = copy.deepcopy(valid_document)
        truncated_grid["data"].pop()
        with self.assertRaisesRegex(ValueError, "invalid grid metadata"):
            target.load_dict(truncated_grid)

        live_worker = SimpleNamespace(is_running=True)
        dashboard = Dashboard(
            SimpleNamespace(camera=SimpleNamespace()),
            SimpleNamespace(stream_settings={}, dropped_rows=0),
            self.config["dashboard"], slam_map=target, slam_worker=live_worker,
        )
        with self.assertRaisesRegex(ValueError, "stop exploration"):
            dashboard.import_map(valid_document)


if __name__ == "__main__":
    unittest.main()
