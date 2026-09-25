import csv
import io
import json
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

from main import _sdk_connection_type
from src.PID import PIDController
from src.chassis import ChassisController, angle_error
from src.config_loader import load_config
from src.dashboard import Dashboard
from src.logger import SensorLogger


class FakeModule:
    def __init__(self):
        self.callbacks = {}
        self.commands = []

    def sub_position(self, **options):
        self.callbacks["position"] = options["callback"]
        return True

    def unsub_position(self):
        self.callbacks.pop("position", None)

    def sub_attitude(self, **options):
        self.callbacks["attitude"] = options["callback"]
        return True

    def unsub_attitude(self):
        self.callbacks.pop("attitude", None)

    def drive_speed(self, **speed):
        self.commands.append(speed)


class TemplateTests(unittest.TestCase):
    def test_config_and_pid(self):
        config = load_config()
        self.assertFalse(config["mission"]["enabled"])
        pid = PIDController(1, 0, 0, max_output=0.3)
        self.assertEqual(pid.compute(2, 0.1), 0.3)
        self.assertEqual(pid.compute(-2, 0.1), -0.3)
        self.assertEqual(angle_error(-179, 179), 2)
        self.assertFalse(config["dashboard"]["enabled"])

    def test_sdk_connection_uses_constant_objects(self):
        constants = SimpleNamespace(
            CONNECTION_WIFI_AP=object(),
            CONNECTION_WIFI_STA=object(),
            CONNECTION_USB_RNDIS=object(),
        )
        for name, expected in (
            ("ap", constants.CONNECTION_WIFI_AP),
            ("sta", constants.CONNECTION_WIFI_STA),
            ("rndis", constants.CONNECTION_USB_RNDIS),
        ):
            self.assertIs(_sdk_connection_type(name, constants), expected)

    def test_dashboard_serves_page_and_latest_telemetry(self):
        logger = SimpleNamespace(
            stream_settings={"position": {"enabled": True}, "tof": {"enabled": False}},
            dropped_rows=2,
            get_latest=lambda name, max_age_s=None: (1, 2, 0),
            get_history_since=lambda after_id=0: {
                "cursor": 3, "streams": {"position": [(3, 1.0, (1, 2, 0))]}
            },
        )
        robot = SimpleNamespace(camera=SimpleNamespace())
        dashboard = Dashboard(robot, logger, load_config()["dashboard"])
        dashboard.mission_status = "Ready"
        handler_type = dashboard._handler_class()

        def request(path):
            handler = handler_type.__new__(handler_type)
            handler.path = path
            handler.wfile = io.BytesIO()
            handler.send_response = lambda code: None
            handler.send_header = lambda name, value: None
            handler.end_headers = lambda: None
            handler.do_GET()
            return handler.wfile.getvalue()

        page = request("/")
        self.assertIn(b"RoboMaster Dashboard", page)
        status = json.loads(request("/api/status"))
        self.assertEqual(status["streams"]["position"], [1, 2, 0])
        self.assertNotIn("tof", status["streams"])
        self.assertEqual(status["dropped_csv_rows"], 2)
        history = json.loads(request("/api/history?since=2"))
        self.assertEqual(history["cursor"], 3)
        self.assertEqual(history["streams"]["position"][0][2], [1, 2, 0])
        self.assertEqual(history["columns"]["position"], ["x_m", "y_m", "z_deg"])

    def test_dashboard_camera_keeps_only_latest_jpeg(self):
        calls = []

        def read_image(**options):
            calls.append(options)
            if len(calls) == 1:
                return object()
            raise RuntimeError("camera disconnected")

        fake_cv2 = SimpleNamespace(
            IMWRITE_JPEG_QUALITY=1,
            imencode=lambda extension, image, options: (
                True, SimpleNamespace(tobytes=lambda: b"jpeg-bytes")
            ),
        )
        logger = SimpleNamespace(stream_settings={}, dropped_rows=0)
        robot = SimpleNamespace(camera=SimpleNamespace(read_cv2_image=read_image))
        settings = load_config()["dashboard"].copy()
        settings["max_fps"] = 1000
        dashboard = Dashboard(robot, logger, settings)
        dashboard.running.set()
        with patch.dict("sys.modules", {"cv2": fake_cv2}):
            dashboard._camera_loop()
        self.assertEqual(dashboard.latest_jpeg, b"jpeg-bytes")
        self.assertEqual(dashboard.frame_number, 1)
        self.assertEqual(calls[0]["strategy"], "newest")
        self.assertEqual(dashboard.camera_error, "camera disconnected")

    def test_logger_selection_and_csv(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "streams": {
                    "position": {"enabled": True, "save": True, "frequency_hz": 10},
                    "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            module.callbacks["position"]((1, 2, 0))
            module.callbacks["attitude"]((45, 0, 0))
            self.assertEqual(logger.get_latest("position"), (1, 2, 0))
            self.assertEqual(logger.get_latest("attitude"), (45, 0, 0))
            logger.stop()
            with (logger.run_dir / "position.csv").open(newline="") as file:
                self.assertEqual(len(list(csv.reader(file))), 2)
            with (logger.run_dir / "run_summary.json").open(encoding="utf-8") as file:
                summary = json.load(file)
            self.assertEqual(summary["received_rows"]["position"], 1)
            self.assertEqual(summary["dropped_csv_rows"], 0)
            self.assertFalse((logger.run_dir / "attitude.csv").exists())
            self.assertEqual(module.callbacks, {})

    def test_full_queue_drops_csv_rows_but_keeps_latest_position(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "queue_max_rows": 2,
                "batch_size": 1,
                "flush_interval_s": 0.1,
                "streams": {
                    "position": {"enabled": True, "save": True, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            for number in range(100):
                module.callbacks["position"]((number, 0, 0))
            self.assertEqual(logger.get_latest("position"), (99, 0, 0))
            logger.stop()
            self.assertGreater(logger.dropped_rows, 0)
            with (logger.run_dir / "position.csv").open(newline="") as file:
                rows = list(csv.reader(file))
            self.assertEqual(len(rows) - 1, 100 - logger.dropped_rows)

    def test_history_is_bounded_and_copies_nested_sdk_values(self):
        robot = SimpleNamespace()
        settings = {
            "directory": "unused",
            "history_max_samples": 2,
            "streams": {"esc": {"enabled": True, "save": False, "frequency_hz": 10}},
        }
        logger = SensorLogger(robot, settings)
        speeds = [1, 2, 3, 4]
        logger._callback("esc", (speeds, [0] * 4, [0] * 4, [0] * 4))
        speeds[0] = 99
        self.assertEqual(logger.get_history_since()["streams"]["esc"][0][2][0][0], 1)
        logger._callback("esc", ([5] * 4, [0] * 4, [0] * 4, [0] * 4))
        logger._callback("esc", ([6] * 4, [0] * 4, [0] * 4, [0] * 4))
        history = logger.get_history_since(1)
        self.assertEqual(history["cursor"], 3)
        self.assertEqual(len(history["streams"]["esc"]), 2)
        self.assertEqual(history["streams"]["esc"][0][2][0][0], 5)

    def test_move_to_stops_at_target_and_on_missing_data(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "streams": {
                    "position": {"enabled": True, "save": False, "frequency_hz": 10},
                    "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            motion = load_config()["motion"]
            chassis = ChassisController(robot, logger, motion)
            with self.assertRaises(TimeoutError):
                chassis.move_to(1, 0)
            self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})
            module.callbacks["position"]((1, 0, 0))
            module.callbacks["attitude"]((0, 0, 0))
            self.assertEqual(chassis.move_to(1, 0, yaw=0), (1, 0, 0))
            logger.stop()

    def test_move_to_sends_bounded_speed_then_times_out(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "streams": {
                    "position": {"enabled": True, "save": False, "frequency_hz": 10},
                    "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            module.callbacks["position"]((0, 0, 0))
            module.callbacks["attitude"]((90, 0, 0))
            motion = load_config()["motion"]
            motion["control_period_s"] = 0.001
            chassis = ChassisController(robot, logger, motion)
            with self.assertRaises(TimeoutError):
                chassis.move_to(1, 0, yaw=None, timeout_s=0.01)
            first = module.commands[0]
            self.assertAlmostEqual(first["x"], 0, places=4)
            self.assertLess(first["y"], 0)
            self.assertEqual(first["z"], 0)
            self.assertLessEqual(abs(first["y"]), motion["max_speed_m_s"])
            self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})
            logger.stop()

    def test_move_to_corrects_heading_drift_without_new_turn_target(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["control_period_s"] = 0.001
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        poses = iter([(0, 0, 0), (0, 0, 7)])
        chassis.get_pose = lambda: next(poses, (0, 0, 7))

        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, timeout_s=0.02)

        self.assertEqual(module.commands[0]["z"], 0)
        self.assertTrue(any(command["z"] < 0 for command in module.commands[1:]))
        self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})

        module.commands.clear()
        chassis.get_pose = lambda: (0, 0, 7)
        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, timeout_s=0.01)
        self.assertLess(module.commands[0]["z"], 0)  # The original heading survives a new waypoint.

        module.commands.clear()
        chassis.reset_heading()
        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, timeout_s=0.01)
        self.assertEqual(module.commands[0]["z"], 0)


if __name__ == "__main__":
    unittest.main()
