"""Small 2D simulator for the MazeRunner drive policy.

The important bits match src/main.cpp:
  * differential drive, PWM -> mm/s, and the 25 Hz output smoothing
  * the eight inputs used by nnPolicy()
  * front ultrasonic in mm (1200 max) and left/right ToF (800 max)
  * gyro yaw rate with the same sign as the MPU6050 z axis

The robot is a circle. Walls are axis-aligned rectangles. Coordinates are mm,
heading is degrees, 0 = +x, and positive heading is counter-clockwise.

    python ml/sim.py --episodes 3
    python ml/sim.py --episodes 1 --render
    python ml/test_sim.py
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from typing import Sequence

# Robot and motor model. The firmware's speed constants were measured at
# 150 PWM, which is what the autonomous page uses.
DT = 0.04  # 25 Hz, same as neuralDrive()
TRACK_MM = 92.0
ROBOT_RADIUS_MM = 55.0
WHEEL_MM_S_PER_PWM = 250.0 / 150.0
PWM_DEADBAND = 10
MAX_PWM = 255
ACTION_TO_PWM = 200.0  # netPwm() in main.cpp

# Sensors.
FRONT_RANGE_MM = 1200.0
SIDE_RANGE_MM = 800.0
SENSOR_FORWARD_MM = 55.0
SENSOR_SIDE_MM = 50.0
DELTA_SCALE = 150.0
GYRO_SCALE = 120.0

WALL_THICKNESS = 24.0


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass(frozen=True)
class Wall:
    x0: float
    y0: float
    x1: float
    y1: float


# Walls are rectangles, so a box is just a Wall with x0 < x1 and y0 < y1.

@dataclass
class Maze:
    walls: list[Wall]
    start_x: float
    start_y: float
    start_heading: float = 0.0
    name: str = "maze"


def _rect(cx: float, cy: float, w: float, h: float) -> Wall:
    return Wall(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def _outer_walls(width: float, height: float) -> list[Wall]:
    t = WALL_THICKNESS
    return [
        Wall(0, 0, width, t),
        Wall(0, height - t, width, height),
        Wall(0, 0, t, height),
        Wall(width - t, 0, width, height),
    ]


def _overlaps(a: Wall, b: Wall, margin: float = 0.0) -> bool:
    return not (
        a.x1 + margin < b.x0
        or b.x1 + margin < a.x0
        or a.y1 + margin < b.y0
        or b.y1 + margin < a.y0
    )


def random_rooms(
    seed: int = 0,
    width: float = 2400.0,
    height: float = 1800.0,
    count: int = 10,
) -> Maze:
    """A big room with random wall pieces, good for basic wall following."""
    rng = random.Random(seed)
    walls = _outer_walls(width, height)
    sx, sy = width * 0.18, height * 0.5
    added = 0
    attempts = 0

    while added < count and attempts < count * 60:
        attempts += 1
        length = rng.uniform(280.0, 700.0)
        cx = rng.uniform(300.0, width - 300.0)
        cy = rng.uniform(250.0, height - 250.0)
        if rng.random() < 0.5:
            cand = _rect(cx, cy, length, WALL_THICKNESS)
        else:
            cand = _rect(cx, cy, WALL_THICKNESS, length)

        # Leave a clear starting area, and don't stack walls on each other.
        if math.hypot(cx - sx, cy - sy) < 480.0:
            continue
        if cand.x0 < 60 or cand.y0 < 60 or cand.x1 > width - 60 or cand.y1 > height - 60:
            continue
        if any(_overlaps(cand, w, margin=110.0) for w in walls):
            continue

        walls.append(cand)
        added += 1

    return Maze(walls, sx, sy, 0.0, "rooms")


def s_curve(seed: int = 0, width: float = 2400.0, height: float = 1800.0) -> Maze:
    """Two vertical walls with opposite gaps, so the robot has to weave."""
    rng = random.Random(seed)
    walls = _outer_walls(width, height)

    x1 = width * 0.34 + rng.uniform(-70.0, 70.0)
    gap1 = 520.0 + rng.uniform(-60.0, 60.0)
    walls.append(_rect(x1, gap1 + (height - gap1) / 2, WALL_THICKNESS, height - gap1))

    x2 = width * 0.68 + rng.uniform(-70.0, 70.0)
    gap2 = 520.0 + rng.uniform(-60.0, 60.0)
    walls.append(_rect(x2, (height - gap2) / 2, WALL_THICKNESS, height - gap2))

    return Maze(walls, width * 0.12, height * 0.5, 0.0, "s_curve")


def make_maze(kind: str, seed: int) -> Maze:
    if kind == "rooms":
        return random_rooms(seed)
    if kind == "s_curve":
        return s_curve(seed)
    if kind == "mix":
        return s_curve(seed) if seed % 2 == 0 else random_rooms(seed)
    raise ValueError(f"unknown maze kind: {kind}")


def _ray_wall(
    ox: float, oy: float, dx: float, dy: float, wall: Wall, max_range: float
) -> float | None:
    """Distance along a ray to an axis-aligned rectangle, or None."""
    tmin, tmax = 0.0, max_range
    for origin, direction, lo, hi in (
        (ox, dx, wall.x0, wall.x1),
        (oy, dy, wall.y0, wall.y1),
    ):
        if abs(direction) < 1e-12:
            if origin < lo or origin > hi:
                return None
        else:
            t1 = (lo - origin) / direction
            t2 = (hi - origin) / direction
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > tmin:
                tmin = t1
            if t2 < tmax:
                tmax = t2
            if tmin > tmax:
                return None
    return tmin


def ray_distance(
    ox: float, oy: float, angle_deg: float, max_range: float, walls: Sequence[Wall]
) -> float:
    rad = math.radians(angle_deg)
    dx, dy = math.cos(rad), math.sin(rad)
    best = max_range
    for wall in walls:
        hit = _ray_wall(ox, oy, dx, dy, wall, best)
        if hit is not None and hit < best:
            best = hit
    return best


def _point_in_wall(x: float, y: float, wall: Wall) -> bool:
    return wall.x0 <= x <= wall.x1 and wall.y0 <= y <= wall.y1


def _circle_hits_wall(x: float, y: float, r: float, wall: Wall) -> bool:
    cx = clamp(x, wall.x0, wall.x1)
    cy = clamp(y, wall.y0, wall.y1)
    return (x - cx) ** 2 + (y - cy) ** 2 <= r * r


class MazeEnv:
    """One robot in one maze. API is reset(seed) -> obs and step(action)."""

    def __init__(
        self,
        maze: Maze | None = None,
        kind: str = "mix",
        seed: int = 0,
        max_steps: int = 600,
        sensor_noise: bool = True,
        collision_ends: bool = True,
        mismatch: float = 0.0,
    ) -> None:
        self.kind = kind
        self.fixed_maze = maze
        self.maze = maze if maze is not None else make_maze(kind, seed)
        self.seed = seed
        self.rng = random.Random(seed)
        self.max_steps = max_steps
        self.sensor_noise = sensor_noise
        self.collision_ends = collision_ends
        self.mismatch = mismatch
        self.right_gain = 1.0
        self.reset()

    def reset(self, seed: int | None = None) -> tuple[float, ...]:
        if seed is not None:
            self.seed = seed
            self.rng = random.Random(seed)
            if self.fixed_maze is None:
                self.maze = make_maze(self.kind, seed)

        self.x = self.maze.start_x
        self.y = self.maze.start_y
        self.heading = self.maze.start_heading
        self.cmd_l = 0.0
        self.cmd_r = 0.0
        self.yaw_rate = 0.0
        self.prev_l = SIDE_RANGE_MM
        self.prev_r = SIDE_RANGE_MM
        self.prev_speed = 0.0
        self.steps = 0
        self.path = 0.0
        self.progress = 0.0
        self.collisions = 0
        self.min_front = FRONT_RANGE_MM
        self.total_reward = 0.0
        self.done = False
        self.done_reason = ""
        self._progress_hist: list[float] = []
        self.right_gain = 1.0
        if self.mismatch:
            self.right_gain += self.rng.uniform(-self.mismatch, self.mismatch)

        front, left, right = self._read_sensors()
        self.min_front = front
        obs = self._make_obs(front, left, right, 0.0)
        self.prev_l, self.prev_r = left, right
        return obs

    def _sensor(self, angle: float, max_range: float, fx: float, fy: float) -> float:
        h = math.radians(self.heading)
        ox = self.x + math.cos(h) * fx - math.sin(h) * fy
        oy = self.y + math.sin(h) * fx + math.cos(h) * fy
        dist = ray_distance(ox, oy, self.heading + angle, max_range, self.maze.walls)

        if not self.sensor_noise or dist >= max_range:
            return dist
        dist += self.rng.gauss(0.0, 8.0 if angle == 0 else 5.0)
        if self.rng.random() < 0.008:  # occasional missing echo
            return max_range
        return clamp(dist, 0.0, max_range)

    def _read_sensors(self) -> tuple[float, float, float]:
        front = self._sensor(0.0, FRONT_RANGE_MM, SENSOR_FORWARD_MM, 0.0)
        left = self._sensor(90.0, SIDE_RANGE_MM, 0.0, SENSOR_SIDE_MM)
        right = self._sensor(-90.0, SIDE_RANGE_MM, 0.0, -SENSOR_SIDE_MM)
        return front, left, right

    def _make_obs(
        self, front: float, left: float, right: float, prev_speed: float
    ) -> tuple[float, ...]:
        return (
            clamp(front / FRONT_RANGE_MM, 0.0, 1.0),
            clamp(left / SIDE_RANGE_MM, 0.0, 1.0),
            clamp(right / SIDE_RANGE_MM, 0.0, 1.0),
            clamp((right - left) / SIDE_RANGE_MM, -1.0, 1.0),
            clamp((left - self.prev_l) / DELTA_SCALE, -1.0, 1.0),
            clamp((right - self.prev_r) / DELTA_SCALE, -1.0, 1.0),
            clamp(self.yaw_rate / GYRO_SCALE, -1.0, 1.0),
            clamp(prev_speed, 0.0, 1.0),
        )

    @staticmethod
    def _pwm(value: float) -> int:
        pwm = int(round(value * ACTION_TO_PWM))
        if 0 < pwm < 22:
            pwm = 22
        if -22 < pwm < 0:
            pwm = -22
        return int(clamp(pwm, -MAX_PWM, MAX_PWM))

    @staticmethod
    def _wheel(pwm: int, gain: float) -> float:
        if abs(pwm) <= PWM_DEADBAND:
            return 0.0
        speed = (abs(pwm) - PWM_DEADBAND) * WHEEL_MM_S_PER_PWM * gain
        return math.copysign(speed, pwm)

    def _hit_wall(self, x: float, y: float) -> bool:
        return any(
            _circle_hits_wall(x, y, ROBOT_RADIUS_MM, wall) for wall in self.maze.walls
        )

    def step(self, action: Sequence[float]) -> tuple[tuple[float, ...], float, bool, dict]:
        if self.done:
            raise RuntimeError("episode is over, call reset()")

        a_l = clamp(float(action[0]), -1.0, 1.0)
        a_r = clamp(float(action[1]), -1.0, 1.0)
        prev_speed = max(0.0, (self.cmd_l + self.cmd_r) * 0.5)

        # Same first-order smoothing as neuralDrive().
        self.cmd_l += (a_l - self.cmd_l) * 0.5
        self.cmd_r += (a_r - self.cmd_r) * 0.5

        left_speed = self._wheel(self._pwm(self.cmd_l), 1.0)
        right_speed = self._wheel(self._pwm(self.cmd_r), self.right_gain)
        if self.sensor_noise:
            left_speed *= 1.0 + self.rng.gauss(0.0, 0.01)
            right_speed *= 1.0 + self.rng.gauss(0.0, 0.01)

        old_x, old_y, old_heading = self.x, self.y, self.heading
        old_rad = math.radians(old_heading)
        v = (left_speed + right_speed) * 0.5
        omega = (right_speed - left_speed) / TRACK_MM
        self.yaw_rate = math.degrees(omega)
        self.heading = (self.heading + self.yaw_rate * DT + 180.0) % 360.0 - 180.0
        mid_rad = math.radians(old_heading + self.yaw_rate * DT * 0.5)
        self.x += v * DT * math.cos(mid_rad)
        self.y += v * DT * math.sin(mid_rad)

        hit = self._hit_wall(self.x, self.y)
        if hit:
            self.x, self.y, self.heading = old_x, old_y, old_heading
            self.yaw_rate = 0.0
            self.collisions += 1
            progress = 0.0
        else:
            progress = (self.x - old_x) * math.cos(old_rad) + (self.y - old_y) * math.sin(old_rad)
            self.path += math.hypot(self.x - old_x, self.y - old_y)

        self.progress += progress
        self._progress_hist.append(progress)
        if len(self._progress_hist) > 100:
            self._progress_hist.pop(0)
        self.steps += 1

        front, left, right = self._read_sensors()
        self.min_front = min(self.min_front, front)

        reward = 0.02 * progress + 0.02 - 0.02 * abs(a_l - a_r)
        if front < 300.0:
            reward -= (300.0 - front) / 300.0 * 0.6
        if front < 150.0:
            reward -= 1.5
        if left < 120.0:
            reward -= (120.0 - left) / 120.0 * 0.3
        if right < 120.0:
            reward -= (120.0 - right) / 120.0 * 0.3
        if left < 600.0 and right < 600.0:
            reward -= abs(left - right) / 600.0 * 0.2
        if hit:
            reward -= 15.0

        self.done = False
        self.done_reason = ""
        if hit and self.collision_ends:
            self.done = True
            self.done_reason = "collision"
        elif self.steps >= self.max_steps:
            self.done = True
            self.done_reason = "timeout"
        elif self.steps > 100 and sum(self._progress_hist[-80:]) < 15.0:
            self.done = True
            self.done_reason = "stuck"
            reward -= 5.0

        self.total_reward += reward
        obs = self._make_obs(front, left, right, prev_speed)
        self.prev_l, self.prev_r = left, right
        self.prev_speed = max(0.0, (self.cmd_l + self.cmd_r) * 0.5)

        info = {
            "steps": self.steps,
            "collision": hit,
            "front": front,
            "left": left,
            "right": right,
            "x": self.x,
            "y": self.y,
            "heading": self.heading,
            "path": self.path,
            "progress": self.progress,
            "min_front": self.min_front,
            "total_reward": self.total_reward,
            "done_reason": self.done_reason,
            "maze": self.maze.name,
        }
        return obs, reward, self.done, info

    def render_ascii(self, width: int = 64, height: int = 28) -> str:
        """A quick top-down map, mostly for debugging a run."""
        max_x = max(w.x1 for w in self.maze.walls)
        max_y = max(w.y1 for w in self.maze.walls)
        grid = [[" "] * width for _ in range(height)]
        cell = max(max_x / width, max_y / height) * 0.6

        for row in range(height):
            y = (row + 0.5) * max_y / height
            for col in range(width):
                x = (col + 0.5) * max_x / width
                if any(_circle_hits_wall(x, y, cell, w) for w in self.maze.walls):
                    grid[row][col] = "#"

        rx = int(clamp(self.x / max_x * width, 0, width - 1))
        ry = int(clamp(self.y / max_y * height, 0, height - 1))
        grid[ry][rx] = "R"
        return "\n".join("".join(row) for row in grid)


def reactive_action(obs: Sequence[float]) -> tuple[float, float]:
    """A tiny hand-written demo policy, independent of the trained net."""
    front, left, right = obs[0], obs[1], obs[2]
    if front < 0.28:
        return (0.15, 0.55) if left > right else (0.55, 0.15)
    turn = clamp((right - left) * 0.6, -0.25, 0.25)
    return (clamp(0.55 + turn, -1, 1), clamp(0.55 - turn, -1, 1))


def run_demo(
    episodes: int = 3,
    kind: str = "mix",
    seed: int = 0,
    max_steps: int = 600,
    render: bool = False,
) -> None:
    for ep in range(episodes):
        maze = make_maze(kind, seed + ep)
        env = MazeEnv(maze=maze, seed=seed + ep, max_steps=max_steps)
        obs = env.reset()
        while not env.done:
            obs, _, _, info = env.step(reactive_action(obs))
        print(
            f"ep {ep + 1}: {maze.name:8s} reward {info['total_reward']:7.1f} "
            f"steps {info['steps']:3d} path {info['path']:6.0f}mm "
            f"min front {info['min_front']:5.0f}mm "
            f"end {info['done_reason'] or 'running'}"
        )
        if render:
            print(env.render_ascii())
            print()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--kind", choices=("rooms", "s_curve", "mix"), default="mix")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-steps", type=int, default=600)
    ap.add_argument("--render", action="store_true")
    args = ap.parse_args()
    run_demo(args.episodes, args.kind, args.seed, args.max_steps, args.render)


if __name__ == "__main__":
    main()
