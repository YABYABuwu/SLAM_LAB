import copy
import unittest

from src.config_loader import load_config
from src.slam import CellWallGrid, OccupancyGridSLAM


class CellWallGridTests(unittest.TestCase):
    def test_ray_opens_crossed_edges_and_wall_is_shared(self):
        grid = CellWallGrid(0.6, 20)
        self.assertFalse(grid.can_cross((0, 0), (1, 0)))
        grid.observe((0, 0), (1, 0), 1.555, 1480, 0.785, 1.0)
        self.assertTrue(grid.can_cross((0, 0), (1, 0)))
        self.assertEqual(grid.state((1, 0), (1, 0)), "open")
        self.assertEqual(grid.state((2, 0), (1, 0)), "wall")
        self.assertEqual(grid.state((3, 0), (-1, 0)), "wall")
        # A reciprocal open edge alone does not grant fresh clearance.
        self.assertFalse(grid.can_cross((1, 0), (-1, 0)))
        grid.observe((1, 0), (-1, 0), .16, 85, .785, 2.0)
        self.assertEqual(grid.state((0, 0), (1, 0)), "wall")
        self.assertFalse(grid.can_cross((0, 0), (1, 0)))

    def test_open_edge_can_still_have_insufficient_body_clearance(self):
        grid = CellWallGrid(.6, 20)
        grid.observe((0, 0), (1, 0), .16, 85, .08, 1.0)
        self.assertFalse(grid.can_cross((0, 0), (1, 0)))
        grid.observe((0, 0), (0, -1), .859, 784, .785, 1.0)
        self.assertEqual(grid.state((0, 0), (0, -1)), "open")
        self.assertFalse(grid.can_cross((0, 0), (0, -1)))
        grid.observe((0, 0), (0, -1), .860, 785, .7850000000000001, 2.0)
        self.assertTrue(grid.can_cross((0, 0), (0, -1)))

    def test_ray_budget_does_not_invent_a_wall(self):
        grid = CellWallGrid(.6, 3)
        grid.observe((0, 0), (1, 0), 65.0, 64925, .785, 1.0)
        self.assertEqual(grid.state((2, 0), (1, 0)), "open")
        self.assertEqual(grid.state((3, 0), (1, 0)), "unknown")

    def test_wall_grid_round_trip_and_invalid_shared_edge(self):
        settings = load_config()["exploration"]
        grid = CellWallGrid(.6, 20)
        grid.observe((0, 0), (0, 1), .16, 85, .785, 1.0)
        slam = OccupancyGridSLAM(settings)
        slam.update((0, 0, 0), 1480)
        slam.set_exploration_state({"cell_grid": grid.snapshot((0, 0, 0), (0, 0))})
        document = slam.to_dict()
        restored = OccupancyGridSLAM(settings)
        restored.load_dict(document)
        self.assertEqual(restored.to_dict()["exploration"], document["exploration"])
        invalid = copy.deepcopy(document)
        invalid["exploration"]["cell_grid"]["cells"][0]["sides"]["y+"]["state"] = "open"
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            restored.load_dict(invalid)
        self.assertEqual(restored.to_dict()["exploration"], document["exploration"])


if __name__ == "__main__":
    unittest.main()
