"""Small sanity checks for ml/sim.py.

    python ml/test_sim.py
"""

import unittest

from sim import (
    FRONT_RANGE_MM,
    Maze,
    MazeEnv,
    Wall,
    _outer_walls,
    make_maze,
    ray_distance,
)


class RayTests(unittest.TestCase):
    def test_hits_vertical_wall(self):
        wall = Wall(500, 0, 524, 1000)
        self.assertAlmostEqual(ray_distance(0, 50, 0, 1200, [wall]), 500, places=3)

    def test_misses_wall(self):
        wall = Wall(500, 0, 524, 100)
        self.assertAlmostEqual(ray_distance(0, 500, 0, 1200, [wall]), 1200, places=3)

    def test_distance_is_clamped_by_range(self):
        wall = Wall(900, 0, 924, 1000)
        self.assertAlmostEqual(ray_distance(0, 50, 0, 500, [wall]), 500, places=3)


class MazeTests(unittest.TestCase):
    def test_rooms_start_is_clear(self):
        maze = make_maze("rooms", 4)
        env = MazeEnv(maze=maze, seed=4, sensor_noise=False)
        self.assertFalse(env._hit_wall(env.x, env.y))
        self.assertGreater(env._read_sensors()[0], 300)

    def test_mix_includes_both_shapes(self):
        self.assertEqual(make_maze("mix", 0).name, "s_curve")
        self.assertEqual(make_maze("mix", 1).name, "rooms")

    def test_render_has_robot(self):
        env = MazeEnv(maze=make_maze("s_curve", 0), sensor_noise=False)
        self.assertIn("R", env.render_ascii())


class EnvTests(unittest.TestCase):
    def test_observation_shape(self):
        env = MazeEnv(maze=make_maze("rooms", 2), seed=2, sensor_noise=False)
        obs = env.reset()
        self.assertEqual(len(obs), 8)
        for value in obs:
            self.assertGreaterEqual(value, -1.0)
            self.assertLessEqual(value, 1.0)

    def test_forward_moves_and_stays_straight(self):
        maze = Maze(_outer_walls(1400, 900), 300, 450, 0, "test")
        env = MazeEnv(maze=maze, seed=0, sensor_noise=False)
        env.reset()
        x0, heading0 = env.x, env.heading
        _, reward, done, info = env.step((0.5, 0.5))
        self.assertFalse(done)
        self.assertGreater(env.x, x0)
        self.assertAlmostEqual(env.heading, heading0, places=6)
        self.assertGreater(reward, 0)
        self.assertGreater(info["path"], 0)

    def test_left_sensor_sees_left_wall(self):
        walls = _outer_walls(1200, 2000)
        walls.append(Wall(0, 1120, 1200, 1144))  # above the start, on the left
        maze = Maze(walls, 300, 1000, 0, "left-wall")
        env = MazeEnv(maze=maze, seed=0, sensor_noise=False)
        obs = env.reset()
        self.assertLess(obs[1], 0.3)
        self.assertGreater(obs[2], 0.9)

    def test_collision_ends_episode(self):
        maze = Maze(_outer_walls(1200, 900), 100, 450, 0, "near-wall")
        env = MazeEnv(maze=maze, seed=0, sensor_noise=False)
        env.reset()
        done = False
        for _ in range(100):
            _, _, done, info = env.step((-0.6, -0.6))
            if done:
                break
        self.assertTrue(done)
        self.assertEqual(info["done_reason"], "collision")
        self.assertTrue(info["collision"])

    def test_reset_is_deterministic(self):
        env = MazeEnv(kind="rooms", seed=11, sensor_noise=True)
        first = env.reset(seed=11)
        second = env.reset(seed=11)
        self.assertEqual(first, second)

    def test_front_never_exceeds_range(self):
        env = MazeEnv(kind="mix", seed=5, sensor_noise=True)
        env.reset(seed=5)
        for _ in range(50):
            _, _, done, info = env.step((0.5, 0.5))
            self.assertLessEqual(info["front"], FRONT_RANGE_MM)
            if done:
                break


if __name__ == "__main__":
    unittest.main(verbosity=2)
