"""
FastObstacleNavigator
=====================

A ready-made navigation algorithm for the AI Master prerequisite checkpoint.

Students are NOT expected to implement this algorithm. They only need to:

1. Read /odom.
2. Read /scan.
3. Call navigator.compute(...).
4. Publish the returned linear/angular velocity to /cmd_vel.
5. Stop when the target has been reached.

The algorithm is deliberately simple, fast and robust for a teaching
simulation. It uses three states:

    GO_TO_GOAL
        Drive directly toward the target.

    TURN_AWAY
        When a close obstacle is first detected, make one decisive turn toward
        the side with more free space.

    WALL_FOLLOW
        Move forward along the obstacle boundary instead of repeatedly deciding
        left/right. This avoids the oscillation typical of naive reactive
        obstacle avoidance.

Once the goal direction is clear again, the robot returns to GO_TO_GOAL.
"""

import math


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class FastObstacleNavigator:
    # Motion limits for the simulation.
    MAX_LINEAR = 0.75
    MAX_ANGULAR = 1.60

    # Distances in meters.
    OBSTACLE_TRIGGER = 0.85
    EMERGENCY_DISTANCE = 0.42
    WALL_TARGET_DISTANCE = 0.62
    WALL_TOO_CLOSE = 0.48
    PATH_CLEAR_DISTANCE = 1.15

    # Goal following.
    TURN_IN_PLACE_ERROR = math.radians(65.0)
    FULL_SPEED_ERROR = math.radians(12.0)
    EXIT_GOAL_ERROR = math.radians(28.0)

    # Number of consecutive control cycles that must be clear before leaving
    # wall-follow mode. This is the main anti-oscillation hysteresis.
    CLEAR_CYCLES_REQUIRED = 8

    def __init__(self):
        self.state = "GO_TO_GOAL"

        # +1 = follow obstacle with obstacle on the left side
        # -1 = follow obstacle with obstacle on the right side
        self.follow_side = 0

        self.clear_cycles = 0
        self.hit_goal_distance = float("inf")

    def reset(self):
        self.state = "GO_TO_GOAL"
        self.follow_side = 0
        self.clear_cycles = 0
        self.hit_goal_distance = float("inf")

    def compute(
        self,
        current_x,
        current_y,
        current_yaw,
        target_x,
        target_y,
        front,
        front_left,
        front_right,
        left,
        right,
    ):
        """
        Return:

            linear_x, angular_z, state

        Sensor distances should be finite values in meters. If a sector has no
        valid reading, pass float("inf").
        """

        dx = target_x - current_x
        dy = target_y - current_y
        goal_distance = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        goal_error = normalize_angle(desired_yaw - current_yaw)

        # Emergency collision avoidance has priority over all states.
        if front < self.EMERGENCY_DISTANCE:
            if self.follow_side == 0:
                # Choose the most open side once.
                self.follow_side = 1 if left >= right else -1

            # If obstacle is on our left, rotate right; vice versa.
            angular = -self.follow_side * self.MAX_ANGULAR
            return 0.0, angular, "EMERGENCY_TURN"

        if self.state == "GO_TO_GOAL":
            return self._go_to_goal(
                goal_distance,
                goal_error,
                front,
                left,
                right,
            )

        if self.state == "TURN_AWAY":
            return self._turn_away(
                goal_error,
                front,
                front_left,
                front_right,
                left,
                right,
            )

        return self._wall_follow(
            goal_distance,
            goal_error,
            front,
            front_left,
            front_right,
            left,
            right,
        )

    def _go_to_goal(self, goal_distance, goal_error, front, left, right):
        # Obstacle detected: select one side and keep it.
        if front < self.OBSTACLE_TRIGGER:
            self.follow_side = 1 if left >= right else -1
            self.hit_goal_distance = goal_distance
            self.clear_cycles = 0
            self.state = "TURN_AWAY"
            return self._turn_away(
                goal_error,
                front,
                float("inf"),
                float("inf"),
                left,
                right,
            )

        # Large heading error: rotate decisively first.
        if abs(goal_error) > self.TURN_IN_PLACE_ERROR:
            angular = clamp(
                2.2 * goal_error,
                -self.MAX_ANGULAR,
                self.MAX_ANGULAR,
            )
            return 0.0, angular, self.state

        # Drive toward the target while steering.
        angular = clamp(
            1.45 * goal_error,
            -1.10,
            1.10,
        )

        # Very fast when aligned, progressively slower only for large error.
        error_ratio = abs(goal_error) / self.TURN_IN_PLACE_ERROR
        speed_scale = 1.0 - 0.70 * error_ratio
        speed_scale = clamp(speed_scale, 0.30, 1.0)

        if abs(goal_error) <= self.FULL_SPEED_ERROR:
            speed_scale = 1.0

        linear = self.MAX_LINEAR * speed_scale

        # Slow down close to destination to avoid overshoot.
        if goal_distance < 1.0:
            linear = min(
                linear,
                max(0.16, 0.60 * goal_distance),
            )

        return linear, angular, self.state

    def _turn_away(
        self,
        goal_error,
        front,
        front_left,
        front_right,
        left,
        right,
    ):
        # Make one decisive turn, not alternating scan-by-scan.
        #
        # follow_side = +1 -> obstacle kept on LEFT -> turn RIGHT initially.
        # follow_side = -1 -> obstacle kept on RIGHT -> turn LEFT initially.
        turn_sign = -self.follow_side

        # Keep rotating until there is enough forward room to start moving
        # alongside the obstacle.
        side_front = front_left if self.follow_side == 1 else front_right

        if front > 0.95 and side_front > 0.55:
            self.state = "WALL_FOLLOW"
            self.clear_cycles = 0
            return 0.40, turn_sign * 0.55, self.state

        return 0.08, turn_sign * 1.35, self.state

    def _wall_follow(
        self,
        goal_distance,
        goal_error,
        front,
        front_left,
        front_right,
        left,
        right,
    ):
        """
        Follow the obstacle boundary quickly.

        We keep the chosen obstacle on the selected side and control the
        side-distance with a small proportional correction.
        """

        side_distance = left if self.follow_side == 1 else right
        side_front = front_left if self.follow_side == 1 else front_right

        # --------------------------------------------------------------
        # Decide whether we can leave the obstacle.
        # --------------------------------------------------------------
        # Require several consecutive clear cycles so noise cannot cause
        # GOAL/WALL/GOAL/WALL oscillation.
        goal_direction_clear = (
            front > self.PATH_CLEAR_DISTANCE
            and abs(goal_error) < self.EXIT_GOAL_ERROR
        )

        # Prefer leaving once we have actually made progress relative to the
        # place where the obstacle was encountered.
        progress_made = goal_distance < (self.hit_goal_distance - 0.15)

        if goal_direction_clear and progress_made:
            self.clear_cycles += 1
        else:
            self.clear_cycles = 0

        if self.clear_cycles >= self.CLEAR_CYCLES_REQUIRED:
            self.state = "GO_TO_GOAL"
            self.follow_side = 0
            self.clear_cycles = 0
            return self._go_to_goal(
                goal_distance,
                goal_error,
                front,
                left,
                right,
            )

        # --------------------------------------------------------------
        # Front obstacle while wall-following.
        # --------------------------------------------------------------
        if front < 0.60:
            # Strong turn away from obstacle, but keep a little forward speed
            # so the robot rounds corners instead of rotating forever.
            angular = -self.follow_side * 1.35
            return 0.16, angular, self.state

        # --------------------------------------------------------------
        # Normal wall following.
        # --------------------------------------------------------------
        if not math.isfinite(side_distance):
            # We temporarily lost the wall. Curve gently toward the side where
            # the obstacle should be until it reappears.
            angular = self.follow_side * 0.48
            return 0.48, angular, self.state

        wall_error = self.WALL_TARGET_DISTANCE - side_distance

        # Positive wall_error means the wall is too far away: steer toward it.
        # Sign depends on which side is being followed.
        angular = self.follow_side * 1.25 * wall_error

        # If the obstacle is diagonally close, steer away before it reaches the
        # front sector.
        if side_front < self.WALL_TOO_CLOSE:
            angular += -self.follow_side * 0.85

        angular = clamp(angular, -1.15, 1.15)

        # Fast along open wall sections.
        linear = 0.62

        # Reduce speed only when turning sharply.
        if abs(angular) > 0.80:
            linear = 0.38
        elif abs(angular) > 0.45:
            linear = 0.50

        return linear, angular, self.state
