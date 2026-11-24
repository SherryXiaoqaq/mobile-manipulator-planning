"""Plan, simulate and compare docking strategies and execution timing."""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import time
from bisect import bisect_right
from dataclasses import asdict, dataclass
from pathlib import Path


Point = tuple[float, float]
Joints = tuple[float, float]
Cell = tuple[int, int]

ROOT = Path(__file__).resolve().parent
GRID = 0.20
BASE_RADIUS = 0.62
LINK_RADIUS = 0.04
CARGO_RADIUS = 0.07
LINK_LENGTHS = (0.57, 0.54)
HOME = (0.0, math.pi)
DOCK_RADII = (0.80, 1.00)
DOCK_DIRECTIONS = 16
FRAME_DT = 0.05
BASE_SPEED = 0.85
JOINT_SPEED = 1.65
ARM_CHECK_STEP = 0.024
MIN_CLEARANCE = 0.06
EPS = 1e-8
JOINT_WEIGHT = 0.18
RISK_WEIGHT = 0.20
RISK_OFFSET = 0.15
OVERLAP_STEP = 0.10
GRASP_HOLD = 0.30
RELEASE_HOLD = 0.30
COMPLETE_HOLD = 0.25


@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    w: float
    h: float
    label: str

    @property
    def corners(self) -> tuple[Point, ...]:
        return ((self.x, self.y), (self.x + self.w, self.y),
                (self.x + self.w, self.y + self.h), (self.x, self.y + self.h))

    def point_distance(self, point: Point) -> float:
        x, y = point
        dx = max(self.x - x, x - self.x - self.w, 0.0)
        dy = max(self.y - y, y - self.y - self.h, 0.0)
        return math.hypot(dx, dy)

    def segment_distance(self, start: Point, end: Point) -> float:
        # Intersect segment with rectangle using a one-dimensional interval per axis.
        enter, leave = 0.0, 1.0
        for a, b, low, high in (
            (start[0], end[0], self.x, self.x + self.w),
            (start[1], end[1], self.y, self.y + self.h),
        ):
            step = b - a
            if abs(step) <= EPS:
                if not low <= a <= high:
                    break
                continue
            near, far = sorted(((low - a) / step, (high - a) / step))
            enter, leave = max(enter, near), min(leave, far)
            if enter > leave:
                break
        else:
            return 0.0

        return min(self.point_distance(start), self.point_distance(end),
                   *(point_segment_distance(corner, start, end) for corner in self.corners))


@dataclass(frozen=True)
class Scene:
    id: str
    name: str
    description: str
    width: float
    height: float
    obstacles: tuple[Rect, ...]
    start: Point
    pickup: Point
    dropoff: Point

    def boundary_distance(self, point: Point) -> float:
        x, y = point
        return min(x, y, self.width - x, self.height - y)

    def point_clearance(self, point: Point, radius: float) -> float:
        clearance = self.boundary_distance(point)
        for item in self.obstacles:
            clearance = min(clearance, item.point_distance(point))
        return clearance - radius

    def segment_clearance(self, start: Point, end: Point, radius: float) -> float:
        clearance = min(self.boundary_distance(start), self.boundary_distance(end))
        for item in self.obstacles:
            clearance = min(clearance, item.segment_distance(start, end))
        return clearance - radius


SCENES = (
    Scene("open_floor", "开放工位", "开阔布局用于比较两种停靠策略的行驶距离及关节行程。",
          11.0, 7.0, (Rect(4.6, 5.2, 2.2, 0.8, "储物台"),),
          (1.3, 1.45), (3.0, 3.5), (8.85, 3.95)),
    Scene("narrow_aisle", "窄通道绕行", "底盘经窄门穿过隔断，再绕过第二排货架到达放置区。",
          12.0, 8.0, (Rect(5.1, 0.0, 0.72, 4.2, "下方隔断"),
                      Rect(5.1, 5.8, 0.72, 2.2, "上方隔断"),
                      Rect(8.1, 4.0, 0.7, 4.0, "货架")),
          (1.1, 1.3), (3.2, 2.1), (9.9, 6.1)),
    Scene("occluded_dock", "抓取侧遮挡", "取件点紧邻隔板。最近停靠点可供底盘到达，但机械臂被隔板挡住。",
          12.0, 8.0, (Rect(3.90, 1.65, 0.10, 1.8, "工位隔板"),
                      Rect(6.10, 0.0, 1.50, 4.3, "货架"),
                      Rect(8.6, 5.5, 0.65, 2.5, "上方隔断")),
          (1.2, 2.5), (4.15, 2.55), (10.2, 6.0)),
    Scene("blocked_passage", "通路封闭", "贯通隔墙将抓取区和放置区分离，用于检验不可行任务的报告。",
          11.0, 7.0, (Rect(5.15, 0.0, 0.70, 7.0, "隔墙"),),
          (1.3, 1.45), (3.0, 3.5), (8.85, 3.95)),
)


def point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= EPS * EPS:
        return math.dist(point, start)
    along = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
    t = min(1.0, max(0.0, along))
    return math.hypot(point[0] - start[0] - t * dx, point[1] - start[1] - t * dy)


def lerp(start: Point, end: Point, t: float) -> Point:
    return (start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t)


def nearest_equivalent(angle: float, reference: float) -> float:
    return reference + (angle - reference + math.pi) % math.tau - math.pi


def forward(base: Point, q: Joints) -> tuple[Point, Point, Point]:
    l1, l2 = LINK_LENGTHS
    elbow = (base[0] + l1 * math.cos(q[0]), base[1] + l1 * math.sin(q[0]))
    tip = (elbow[0] + l2 * math.cos(q[0] + q[1]),
           elbow[1] + l2 * math.sin(q[0] + q[1]))
    return base, elbow, tip


def inverse(base: Point, target: Point) -> tuple[Joints, ...]:
    x, y = target[0] - base[0], target[1] - base[1]
    l1, l2 = LINK_LENGTHS
    cosine = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if not -1.0 - EPS <= cosine <= 1.0 + EPS:
        return ()
    magnitude = math.acos(min(1.0, max(-1.0, cosine)))
    solutions = []
    for q2 in (magnitude, -magnitude):
        q1 = math.atan2(y, x) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        solutions.append((nearest_equivalent(q1, HOME[0]), nearest_equivalent(q2, HOME[1])))
    return tuple(solutions)


def joint_distance(start: Joints, end: Joints) -> float:
    return abs(end[0] - start[0]) + abs(end[1] - start[1])


@dataclass(frozen=True)
class ArmMotion:
    joints: tuple[Joints, ...]
    clearance: float

    @property
    def joint_motion(self) -> float:
        return 2.0 * sum(joint_distance(a, b) for a, b in zip(self.joints, self.joints[1:]))


def arm_motion_variants(scene: Scene, base: Point, target: Point) -> tuple[ArmMotion, ...]:
    """Certify interpolants in both directions with the cargo radius included."""
    if scene.point_clearance(target, CARGO_RADIUS) <= MIN_CLEARANCE:
        return ()
    if scene.point_clearance(base, BASE_RADIUS) <= MIN_CLEARANCE:
        return ()
    l1, l2 = LINK_LENGTHS
    motions = []
    for solution in inverse(base, target):
        for proposed in ((HOME, solution), (HOME, (solution[0], HOME[1]), solution)):
            joints = tuple(q for i, q in enumerate(proposed) if i == 0 or q != proposed[i - 1])
            clearance = math.inf
            for start, end in zip(joints, joints[1:]):
                # A Lipschitz bound on displacement of any point on either link.
                dq1, dq2 = abs(end[0] - start[0]), abs(end[1] - start[1])
                displacement = (l1 + l2) * dq1 + l2 * dq2
                subdivisions = max(1, math.ceil(displacement / ARM_CHECK_STEP))
                buffer = displacement / (2 * subdivisions)
                for index in range(subdivisions + 1):
                    _, elbow, tip = forward(base, lerp(start, end, index / subdivisions))
                    sample_clearance = min(
                        scene.segment_clearance(base, elbow, LINK_RADIUS),
                        scene.segment_clearance(elbow, tip, LINK_RADIUS),
                        scene.point_clearance(tip, CARGO_RADIUS),
                    )
                    clearance = min(clearance, sample_clearance - buffer)
                    if clearance <= MIN_CLEARANCE:
                        break
                if clearance <= MIN_CLEARANCE:
                    break
            if clearance > MIN_CLEARANCE:
                motions.append(ArmMotion(joints, clearance))
    return tuple(motions)


@dataclass(frozen=True)
class Route:
    points: tuple[Point, ...]
    length: float
    clearance: float


class BasePlanner:
    """A* over collision-free grid edges, then collision-checked shortcutting."""

    def __init__(self, scene: Scene):
        self.scene = scene
        self.nodes = self.build_graph()
        self.route_cache: dict[tuple[Point, Point], Route | None] = {}
        self.component = self.connected_components()

    @staticmethod
    def point(cell: Cell) -> Point:
        return (round(cell[0] * GRID, 10), round(cell[1] * GRID, 10))

    @staticmethod
    def cell(point: Point) -> Cell:
        return (round(point[0] / GRID), round(point[1] / GRID))

    def build_graph(self) -> dict[Cell, list[tuple[Cell, float]]]:
        nodes = {}
        for ix in range(math.floor(self.scene.width / GRID) + 1):
            for iy in range(math.floor(self.scene.height / GRID) + 1):
                cell = ix, iy
                if self.scene.point_clearance(self.point(cell), BASE_RADIUS) > MIN_CLEARANCE:
                    nodes[cell] = []
        for cell, neighbors in nodes.items():
            for dx, dy in ((0, 1), (1, -1), (1, 0), (1, 1)):
                other = cell[0] + dx, cell[1] + dy
                if other not in nodes:
                    continue
                if self.scene.segment_clearance(self.point(cell), self.point(other), BASE_RADIUS) > MIN_CLEARANCE:
                    weight = GRID * math.hypot(dx, dy)
                    neighbors.append((other, weight))
                    nodes[other].append((cell, weight))
        return nodes

    def connected_components(self) -> dict[Cell, int]:
        components = {}
        for cell in self.nodes:
            if cell in components:
                continue
            label = len(components)
            components[cell] = label
            pending = [cell]
            while pending:
                for adjacent, _ in self.nodes[pending.pop()]:
                    if adjacent not in components:
                        components[adjacent] = label
                        pending.append(adjacent)
        return components

    def docks(self, target: Point) -> tuple[Point, ...]:
        cells = set()
        for radius in DOCK_RADII:
            for index in range(DOCK_DIRECTIONS):
                angle = math.tau * index / DOCK_DIRECTIONS
                point = target[0] + radius * math.cos(angle), target[1] + radius * math.sin(angle)
                cell = self.cell(point)
                if cell in self.nodes and inverse(self.point(cell), target):
                    cells.add(cell)
        return tuple(self.point(cell) for cell in sorted(cells))

    def attach(self, point: Point) -> Cell | None:
        nearest = self.cell(point)
        if nearest in self.nodes and self.point(nearest) == point:
            return nearest
        nearby = ((nearest[0] + dx, nearest[1] + dy)
                  for dx in (-1, 0, 1) for dy in (-1, 0, 1))
        options = [cell for cell in nearby if cell in self.nodes
                   and self.scene.segment_clearance(point, self.point(cell), BASE_RADIUS) > MIN_CLEARANCE]
        return min(options, key=lambda cell: math.dist(point, self.point(cell)), default=None)

    def astar(self, source: Cell, goal: Cell) -> list[Point] | None:
        if self.component[source] != self.component[goal]:
            return None
        queue = [(math.dist(source, goal) * GRID, 0.0, source)]
        previous = {}
        distances = {source: 0.0}
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > distances[current] + EPS:
                continue
            if current == goal:
                cells = [goal]
                while cells[-1] != source:
                    cells.append(previous[cells[-1]])
                return [self.point(cell) for cell in reversed(cells)]
            for other, weight in self.nodes[current]:
                candidate = cost + weight
                if candidate + EPS < distances.get(other, math.inf):
                    distances[other] = candidate
                    previous[other] = current
                    priority = candidate + math.dist(other, goal) * GRID
                    heapq.heappush(queue, (priority, candidate, other))
        return None

    def route(self, start: Point, end: Point) -> Route | None:
        key = start, end
        if key not in self.route_cache:
            self.route_cache[key] = self.make_route(start, end)
        return self.route_cache[key]

    def make_route(self, start: Point, end: Point) -> Route | None:
        direct = self.scene.segment_clearance(start, end, BASE_RADIUS)
        if direct > MIN_CLEARANCE:
            return Route((start, end), math.dist(start, end), direct)
        first, last = self.attach(start), self.attach(end)
        if first is None or last is None:
            return None
        raw = self.astar(first, last)
        if raw is None:
            return None
        points = list(dict.fromkeys([start, *raw, end]))
        shortcut = [start]
        clearance = math.inf
        index = 0
        while index + 1 < len(points):
            for next_index in range(len(points) - 1, index, -1):
                edge = self.scene.segment_clearance(points[index], points[next_index], BASE_RADIUS)
                if edge > MIN_CLEARANCE:
                    shortcut.append(points[next_index])
                    clearance = min(clearance, edge)
                    index = next_index
                    break
            else:
                raise RuntimeError("A* returned an uncertified edge")
        return Route(tuple(shortcut), sum(math.dist(a, b) for a, b in zip(shortcut, shortcut[1:])),
                     clearance)


@dataclass(frozen=True)
class Plan:
    to_pick: Route
    to_drop: Route
    pick_arm: ArmMotion
    drop_arm: ArmMotion

    @property
    def base_length(self) -> float:
        return self.to_pick.length + self.to_drop.length

    @property
    def joint_motion(self) -> float:
        return self.pick_arm.joint_motion + self.drop_arm.joint_motion

    @property
    def clearance(self) -> float:
        return min(self.to_pick.clearance, self.to_drop.clearance,
                   self.pick_arm.clearance, self.drop_arm.clearance)

    @property
    def score(self) -> float:
        return (self.base_length + JOINT_WEIGHT * self.joint_motion
                + RISK_WEIGHT / (RISK_OFFSET + self.clearance))


@dataclass(frozen=True)
class Selection:
    plan: Plan | None
    reason: str
    candidate_pairs: int
    feasible_pairs: int
    pickup_dock: Point | None = None
    dropoff_dock: Point | None = None


def nearest_docks(scene: Scene, planner: BasePlanner, pickups: tuple[Point, ...],
                  drops: tuple[Point, ...]) -> Selection:
    if not pickups or not drops:
        return Selection(None, "取件区或放置区缺少停靠候选。", 0, 0)
    pick = min(pickups, key=lambda point: (math.dist(scene.start, point), point))
    drop = min(drops, key=lambda point: (math.dist(pick, point), point))
    first, second = planner.route(scene.start, pick), planner.route(pick, drop)
    if first is None or second is None:
        return Selection(None, "最近停靠点之间没有可行底盘路线。", 1, 0, pick, drop)
    pick_variants = arm_motion_variants(scene, pick, scene.pickup)
    drop_variants = arm_motion_variants(scene, drop, scene.dropoff)
    if not pick_variants:
        return Selection(None, "最近取件停靠点无法通过机械臂碰撞检查。", 1, 0, pick, drop)
    if not drop_variants:
        return Selection(None, "最近放置停靠点无法通过机械臂碰撞检查。", 1, 0, pick, drop)
    return Selection(best_variant(first, second, pick_variants, drop_variants), "", 1, 1, pick, drop)


def best_variant(first: Route, second: Route, pick_variants: tuple[ArmMotion, ...],
                 drop_variants: tuple[ArmMotion, ...]) -> Plan:
    return min((Plan(first, second, a, b) for a in pick_variants for b in drop_variants),
               key=lambda plan: plan.score)


def reachable_docks(scene: Scene, planner: BasePlanner, start: Point, candidates: tuple[Point, ...],
                    target: Point) -> list[tuple[Route, tuple[ArmMotion, ...]]]:
    options = []
    for candidate in candidates:
        route = planner.route(start, candidate)
        if route is None:
            continue
        motions = arm_motion_variants(scene, candidate, target)
        if motions:
            options.append((route, motions))
    return options


def sequential_docks(scene: Scene, planner: BasePlanner, pickups: tuple[Point, ...],
                     drops: tuple[Point, ...]) -> Selection:
    """Take the shortest feasible first leg, then the shortest feasible second leg."""
    pick_options = reachable_docks(scene, planner, scene.start, pickups, scene.pickup)
    if not pick_options:
        return Selection(None, "没有可到达且机械臂可操作的取件停靠点。", 0, 0)
    first, pick_variants = min(pick_options, key=lambda option: option[0].length)
    pick = first.points[-1]
    drop_options = reachable_docks(scene, planner, pick, drops, scene.dropoff)
    if not drop_options:
        return Selection(None, "从所选取件停靠点无法到达可操作的放置停靠点。",
                         len(drops), 0, pick)
    second, drop_variants = min(drop_options, key=lambda option: option[0].length)
    plan = best_variant(first, second, pick_variants, drop_variants)
    return Selection(plan, "", len(drops), len(drop_options), pick, second.points[-1])


def coupled_docks(scene: Scene, planner: BasePlanner, pickups: tuple[Point, ...],
                  drops: tuple[Point, ...]) -> Selection:
    candidates = len(pickups) * len(drops)
    if not candidates:
        return Selection(None, "取件区或放置区缺少停靠候选。", 0, 0)
    pick_motions = {point: arm_motion_variants(scene, point, scene.pickup) for point in pickups}
    drop_motions = {point: arm_motion_variants(scene, point, scene.dropoff) for point in drops}
    if not any(pick_motions.values()) or not any(drop_motions.values()):
        return Selection(None, "取件区或放置区的候选点都无法通过机械臂碰撞检查。", candidates, 0)
    best = None
    feasible = 0
    for pick, variants in pick_motions.items():
        if not variants:
            continue
        first = planner.route(scene.start, pick)
        if first is None:
            continue
        for drop, drop_variants in drop_motions.items():
            if not drop_variants:
                continue
            second = planner.route(pick, drop)
            if second is None:
                continue
            feasible += 1
            option = best_variant(first, second, variants, drop_variants)
            if best is None or option.score + EPS < best.score:
                best = option
    if best is None:
        return Selection(None, "可操作停靠点之间没有可行底盘路线。", candidates, feasible)
    return Selection(best, "", candidates, feasible, best.to_pick.points[-1], best.to_drop.points[-1])


def rollout(scene: Scene, plan: Plan) -> list[dict]:
    """Sample a stop-and-go execution with constant speed per path segment."""
    frames: list[dict] = []
    timestamp = 0.0
    base = scene.start
    joints = HOME
    grasped = False
    delivered = False

    def snapshot(phase: str) -> None:
        arm = forward(base, joints)
        cargo = arm[-1] if grasped else (scene.dropoff if delivered else scene.pickup)
        frames.append(dict(t=round(timestamp, 6), base=base, joints=joints, arm=arm,
                           cargo=cargo, grasped=grasped, phase=phase))

    def drive(route: Route, phase: str) -> None:
        nonlocal timestamp, base
        for start, end in zip(route.points, route.points[1:]):
            span = math.dist(start, end) / BASE_SPEED
            if span <= EPS:
                continue
            ticks = max(1, math.ceil(span / FRAME_DT))
            for i in range(1, ticks + 1):
                base = lerp(start, end, i / ticks)
                timestamp += span / ticks
                snapshot(phase)

    def move(waypoints: tuple[Joints, ...], phase: str) -> None:
        nonlocal timestamp, joints
        for start, end in zip(waypoints, waypoints[1:]):
            span = max(abs(end[0] - start[0]), abs(end[1] - start[1])) / JOINT_SPEED
            if span <= EPS:
                continue
            ticks = max(1, math.ceil(span / FRAME_DT))
            for i in range(1, ticks + 1):
                joints = lerp(start, end, i / ticks)
                timestamp += span / ticks
                snapshot(phase)

    def hold(span: float, phase: str) -> None:
        nonlocal timestamp
        ticks = max(1, math.ceil(span / FRAME_DT))
        for _ in range(ticks):
            timestamp += span / ticks
            snapshot(phase)

    snapshot("drive_to_pick")
    drive(plan.to_pick, "drive_to_pick")
    move(plan.pick_arm.joints, "extend_pick")
    grasped = True
    hold(GRASP_HOLD, "grasp")
    move(plan.pick_arm.joints[::-1], "retract_pick")
    drive(plan.to_drop, "drive_to_drop")
    move(plan.drop_arm.joints, "extend_drop")
    grasped, delivered = False, True
    hold(RELEASE_HOLD, "release")
    move(plan.drop_arm.joints[::-1], "retract_drop")
    hold(COMPLETE_HOLD, "complete")
    verify_rollout(scene, plan, frames)
    return frames


def verify_rollout(scene: Scene, plan: Plan, frames: list[dict]) -> None:
    if plan.clearance <= MIN_CLEARANCE:
        raise RuntimeError("Uncertified trajectory")
    for frame in frames:
        base, elbow, tip = frame["arm"]
        clearance = min(scene.point_clearance(base, BASE_RADIUS),
                        scene.segment_clearance(base, elbow, LINK_RADIUS),
                        scene.segment_clearance(elbow, tip, LINK_RADIUS),
                        scene.point_clearance(frame["cargo"], CARGO_RADIUS))
        if clearance <= MIN_CLEARANCE:
            raise RuntimeError("Replayed frame intersects an obstacle")
        goal = {"grasp": scene.pickup, "release": scene.dropoff}.get(frame["phase"])
        if goal is not None and math.dist(tip, goal) > EPS:
            raise RuntimeError("Gripper misses its station")
    if frames[-1]["cargo"] != scene.dropoff or frames[-1]["grasped"]:
        raise RuntimeError("Task did not finish with cargo released")


@dataclass(frozen=True)
class MotionTrack:
    times: tuple[float, ...]
    points: tuple[Point, ...]

    @classmethod
    def from_waypoints(cls, points: tuple[Point, ...], joint: bool = False) -> MotionTrack:
        times = [0.0]
        for a, b in zip(points, points[1:]):
            distance = max(abs(a[0] - b[0]), abs(a[1] - b[1])) if joint else math.dist(a, b)
            times.append(times[-1] + distance / (JOINT_SPEED if joint else BASE_SPEED))
        return cls(tuple(times), points)

    @classmethod
    def combine(cls, tracks: tuple[tuple[float, MotionTrack], ...]) -> MotionTrack:
        keyframes = {}
        for offset, track in tracks:
            for moment, point in zip(track.times, track.points):
                keyframes[offset + moment] = point
        times = tuple(sorted(keyframes))
        return cls(times, tuple(keyframes[t] for t in times))

    @property
    def duration(self) -> float:
        return self.times[-1]

    def at(self, t: float) -> Point:
        if t <= self.times[0]:
            return self.points[0]
        if t >= self.times[-1]:
            return self.points[-1]
        i = bisect_right(self.times, t) - 1
        fraction = (t - self.times[i]) / (self.times[i + 1] - self.times[i])
        return lerp(self.points[i], self.points[i + 1], fraction)


def overlap_clearance(scene: Scene, base: MotionTrack, arm: MotionTrack,
                      start: float, end: float) -> float:
    """Certify the product trajectory on each interval between track keyframes."""
    l1, l2 = LINK_LENGTHS
    breaks = sorted({start, end, *(t for t in (*base.times, *arm.times) if start < t < end)})
    clearance = math.inf
    for t0, t1 in zip(breaks, breaks[1:]):
        b0, b1, q0, q1 = base.at(t0), base.at(t1), arm.at(t0), arm.at(t1)
        # Any body point moves at most this far per unit interpolant parameter.
        angular = (l1 + l2) * abs(q1[0] - q0[0]) + l2 * abs(q1[1] - q0[1])
        displacement = math.dist(b0, b1) + angular
        samples = max(1, math.ceil(displacement / ARM_CHECK_STEP))
        buffer = displacement / (2 * samples)
        clearance = min(clearance, scene.segment_clearance(b0, b1, BASE_RADIUS))
        for i in range(samples + 1):
            u = i / samples
            pivot, elbow, tip = forward(lerp(b0, b1, u), lerp(q0, q1, u))
            sample = min(scene.segment_clearance(pivot, elbow, LINK_RADIUS),
                         scene.segment_clearance(elbow, tip, LINK_RADIUS),
                         scene.point_clearance(tip, CARGO_RADIUS))
            # Include a tip radius before pickup as a conservative envelope.
            clearance = min(clearance, sample - buffer)
            if clearance <= MIN_CLEARANCE:
                return clearance
    return clearance


def find_overlap(scene: Scene, first: MotionTrack, second: MotionTrack,
                 cap: float, base_first: bool) -> tuple[float, float, dict]:
    limit = min(first.duration, second.duration, cap)
    search = {"cap_s": round(limit, 6), "selected_s": 0.0, "attempts": []}
    if limit <= EPS:
        return 0.0, math.inf, search
    candidates = [limit]
    for step in range(math.floor(limit / OVERLAP_STEP), 0, -1):
        value = step * OVERLAP_STEP
        if limit - value > EPS:
            candidates.append(value)
    for duration in candidates:
        start = first.duration - duration
        shifted = MotionTrack.combine(((start, second),))
        base, arm = (first, shifted) if base_first else (shifted, first)
        clearance = overlap_clearance(scene, base, arm, start, first.duration)
        accepted = clearance > MIN_CLEARANCE
        search["attempts"].append({
            "duration_s": round(duration, 6),
            "accepted": accepted,
            "clearance_lower_bound_m": round(clearance, 6),
            "clearance_scope": "full_window" if accepted else "failed_check",
        })
        if accepted:
            search["selected_s"] = round(duration, 6)
            return duration, clearance, search
    return 0.0, math.inf, search


@dataclass(frozen=True)
class Execution:
    base: MotionTrack
    arm: MotionTrack
    phases: tuple[tuple[float, str], ...]
    # Each window is recorded as (start time, end time, arm phase).
    windows: tuple[tuple[float, float, str], ...]
    clearance: float
    searches: tuple[dict, ...]


def schedule_overlap(scene: Scene, plan: Plan) -> Execution:
    first_drive = MotionTrack.from_waypoints(plan.to_pick.points)
    transfer = MotionTrack.from_waypoints(plan.to_drop.points)
    pick = MotionTrack.from_waypoints(plan.pick_arm.joints, joint=True)
    retract_pick = MotionTrack.from_waypoints(plan.pick_arm.joints[::-1], joint=True)
    drop = MotionTrack.from_waypoints(plan.drop_arm.joints, joint=True)
    retract_drop = MotionTrack.from_waypoints(plan.drop_arm.joints[::-1], joint=True)

    early_pick, margin_pick, pick_search = find_overlap(scene, first_drive, pick, first_drive.duration, True)
    early_depart, margin_depart, depart_search = find_overlap(
        scene, retract_pick, transfer, transfer.duration / 2, False
    )
    early_drop, margin_drop, drop_search = find_overlap(scene, transfer, drop, transfer.duration / 2, True)
    pick_start = first_drive.duration - early_pick
    grasp = pick_start + pick.duration
    grasp_end = grasp + GRASP_HOLD
    retract_end = grasp_end + retract_pick.duration
    drive_start = retract_end - early_depart
    drive_end = drive_start + transfer.duration
    drop_start = drive_end - early_drop
    release = drop_start + drop.duration
    release_end = release + RELEASE_HOLD
    retract_drop_end = release_end + retract_drop.duration
    phases_and_searches = (
        (pick_start, first_drive.duration, "extend_pick", pick_search),
        (drive_start, retract_end, "retract_pick", depart_search),
        (drop_start, drive_end, "extend_drop", drop_search),
    )

    return Execution(
        MotionTrack.combine(((0.0, first_drive), (drive_start, transfer))),
        MotionTrack.combine(((0.0, MotionTrack((0.0,), (HOME,))), (pick_start, pick),
                             (grasp_end, retract_pick), (drop_start, drop),
                             (release_end, retract_drop))),
        ((pick_start, "drive_to_pick"), (grasp, "extend_pick"), (grasp_end, "grasp"),
         (retract_end, "retract_pick"), (drop_start, "drive_to_drop"),
         (release, "extend_drop"), (release_end, "release"), (retract_drop_end, "retract_drop"),
         (retract_drop_end + COMPLETE_HOLD, "complete")),
        tuple((a, b, phase) for a, b, phase, _ in phases_and_searches if b - a > EPS),
        min(plan.clearance, margin_pick, margin_depart, margin_drop),
        tuple(dict(search, phase=phase, interval={"start_s": round(a, 6), "end_s": round(b, 6)}
                   if b - a > EPS else None) for a, b, phase, search in phases_and_searches),
    )


def rollout_overlap(scene: Scene, plan: Plan, execution: Execution) -> list[dict]:
    if execution.clearance <= MIN_CLEARANCE:
        raise RuntimeError("Uncertified synchronized trajectory")
    frames: list[dict] = []
    breaks = sorted(set(execution.base.times + execution.arm.times
                        + tuple(end for end, _ in execution.phases)))

    def snapshot(t: float, phase: str) -> None:
        base, q = execution.base.at(t), execution.arm.at(t)
        arm = forward(base, q)
        grasped = phase in {"grasp", "retract_pick", "drive_to_drop", "extend_drop"}
        delivered = phase in {"release", "retract_drop", "complete"}
        cargo = arm[-1] if grasped else (scene.dropoff if delivered else scene.pickup)
        base_moving = bool(frames) and math.dist(frames[-1]["base"], base) > EPS
        arm_moving = bool(frames) and math.dist(frames[-1]["joints"], q) > EPS
        frames.append(dict(t=round(t, 6), base=base, joints=q, arm=arm, cargo=cargo,
                           grasped=grasped, phase=phase, base_moving=base_moving, arm_moving=arm_moving))

    snapshot(0.0, "drive_to_pick")
    for start, end in zip(breaks, breaks[1:]):
        if end - start <= EPS:
            continue
        phase = next(label for boundary, label in execution.phases if (start + end) / 2 < boundary)
        ticks = max(1, math.ceil((end - start) / FRAME_DT))
        for i in range(1, ticks + 1):
            snapshot(start + (end - start) * i / ticks, phase)
    verify_rollout(scene, plan, frames)
    return frames


def evaluate(scene: Scene) -> dict:
    result = asdict(scene)
    result["methods"] = {}
    for key, select in (("baseline", nearest_docks), ("coupled", coupled_docks),
                        ("sequential", sequential_docks)):
        clock = time.perf_counter()
        planner = BasePlanner(scene)
        pickups, drops = planner.docks(scene.pickup), planner.docks(scene.dropoff)
        selection = select(scene, planner, pickups, drops)
        planning_ms = (time.perf_counter() - clock) * 1000
        plan = selection.plan
        frames = rollout(scene, plan) if plan else []
        result["methods"][key] = {
            "success": plan is not None,
            "reason": selection.reason,
            "pickup_dock": selection.pickup_dock,
            "dropoff_dock": selection.dropoff_dock,
            "paths": {"to_pick": plan.to_pick.points, "to_drop": plan.to_drop.points} if plan else {},
            "metrics": {
                "base_length_m": round(plan.base_length, 6) if plan else None,
                "joint_motion_rad": round(plan.joint_motion, 6) if plan else None,
                "min_clearance_m": round(plan.clearance, 6) if plan else None,
                "simulated_time_s": frames[-1]["t"] if frames else None,
                "planning_time_ms": round(planning_ms, 3),
                "candidate_pairs": selection.candidate_pairs,
                "feasible_pairs": selection.feasible_pairs,
                "score": round(plan.score, 6) if plan else None,
            },
            "frames": frames,
        }
        if key == "coupled":
            coupled_selection = selection
            coupled_ms = planning_ms
    result["methods"]["overlap"] = evaluate_overlap(
        scene, coupled_selection, coupled_ms, result["methods"]["coupled"]
    )
    return result


def evaluate_overlap(scene: Scene, selection: Selection, planning_ms: float, coupled: dict) -> dict:
    plan = selection.plan
    if plan is None:
        return {**coupled, "overlap_intervals": [], "overlap_search": [],
                "metrics": {**coupled["metrics"], "saved_time_s": None, "concurrent_time_s": None,
                            "overlap_windows": 0, "retiming_time_ms": 0.0}}
    clock = time.perf_counter()
    execution = schedule_overlap(scene, plan)
    retiming_ms = (time.perf_counter() - clock) * 1000
    frames = rollout_overlap(scene, plan, execution)
    concurrent = sum(end - start for start, end, _ in execution.windows)
    saved = coupled["metrics"]["simulated_time_s"] - frames[-1]["t"]
    if abs(saved - concurrent) > 2e-6:
        raise RuntimeError("Overlap duration disagrees with the verified execution time")
    return {
        **coupled,
        "frames": frames,
        "overlap_intervals": [
            {"start_s": round(a, 6), "end_s": round(b, 6), "phase": phase}
            for a, b, phase in execution.windows
        ],
        "overlap_search": execution.searches,
        "metrics": {
            **coupled["metrics"],
            "min_clearance_m": round(execution.clearance, 6),
            "simulated_time_s": frames[-1]["t"],
            "planning_time_ms": round(planning_ms + retiming_ms, 3),
            "saved_time_s": round(saved, 6),
            "concurrent_time_s": round(concurrent, 6),
            "overlap_windows": len(execution.windows),
            "retiming_time_ms": round(retiming_ms, 3),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "results",
                        help="Directory for experiment.json and metrics.csv")
    args = parser.parse_args()
    if max(LINK_LENGTHS[0] + LINK_RADIUS, abs(LINK_LENGTHS[0] - LINK_LENGTHS[1]) + CARGO_RADIUS) > BASE_RADIUS:
        raise RuntimeError("Stowed robot and payload must fit inside the navigation disk")
    args.out.mkdir(parents=True, exist_ok=True)
    data = {
        "model": {
            "base_radius": BASE_RADIUS, "link_radius": LINK_RADIUS, "cargo_radius": CARGO_RADIUS,
            "link_lengths": LINK_LENGTHS, "home_joints": HOME, "frame_dt": FRAME_DT,
            "grid_m": GRID, "dock_radii_m": DOCK_RADII, "dock_directions": DOCK_DIRECTIONS,
            "base_speed_m_s": BASE_SPEED, "joint_speed_rad_s": JOINT_SPEED,
            "arm_check_step_m": ARM_CHECK_STEP, "min_required_clearance_m": MIN_CLEARANCE,
            "joint_weight": JOINT_WEIGHT,
            "risk_weight": RISK_WEIGHT, "risk_offset_m": RISK_OFFSET,
            "overlap_step_s": OVERLAP_STEP,
        },
        "demo_scenario": "occluded_dock",
        "scenarios": [evaluate(scene) for scene in SCENES],
    }
    with (args.out / "experiment.json").open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        stream.write("\n")
    rows = []
    for scene in data["scenarios"]:
        for key, method in scene["methods"].items():
            rows.append({"scenario": scene["id"], "method": key, "success": method["success"],
                         **method["metrics"], "reason": method["reason"]})
    with (args.out / "metrics.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(key for row in rows for key in row)))
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        print(f"{row['scenario']:<17} {row['method']:<8} success={row['success']!s:<5} "
              f"base={row['base_length_m']} m, clearance={row['min_clearance_m']} m, "
              f"pairs={row['feasible_pairs']}/{row['candidate_pairs']}")
    print(f"Results: {args.out.resolve()}")


if __name__ == "__main__":
    main()
