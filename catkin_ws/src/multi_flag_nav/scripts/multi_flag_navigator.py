#!/usr/bin/env python3
"""
Multi-flag maze navigation for TurtleBot3.

Collects N goal points ("flags") at runtime from RViz 'Publish Point' clicks, then
visits them greedily nearest-first. "Nearest" is measured along the *navigable path*
returned by the global planner rather than in a straight line, so the robot does not
walk into a wall that happens to be close to a flag on the far side of it.

No maze geometry is hardcoded: every flag position arrives at runtime.
"""

import math

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


class MultiFlagNavigator:
    def __init__(self):
        rospy.init_node("multi_flag_navigator")

        # --- Tunables (override via <param> in the launch file) -------------
        self.capture_radius = rospy.get_param("~capture_radius", 0.3)
        self.num_flags = rospy.get_param("~num_flags", 5)
        self.max_retries = rospy.get_param("~max_retries", 10)
        # How far short of the flag centre to aim, so goals that sit on top of a
        # wall still land in free space. Must stay under capture_radius.
        self.approach_offset = rospy.get_param("~approach_offset", 0.10)
        # A returned plan shorter than this fraction of the straight-line distance
        # is geometrically impossible and indicates a stale or truncated plan.
        self.plan_sanity_ratio = rospy.get_param("~plan_sanity_ratio", 0.9)

        if self.approach_offset >= self.capture_radius:
            rospy.logwarn(
                "approach_offset (%.2f) >= capture_radius (%.2f); goals may be "
                "reported captured without the robot closing on the flag.",
                self.approach_offset,
                self.capture_radius,
            )

        self.flags = []
        self.remaining_flags = []
        self.captured_order = []  # capture sequence, for the post-run report
        self.collecting = True

        self.robot_x = None
        self.robot_y = None
        self.pose_ready = False

        # Latched so an RViz instance opened mid-run still receives the markers.
        self.marker_pub = rospy.Publisher(
            "/flag_markers", MarkerArray, queue_size=10, latch=True
        )
        # Used only for manual recovery manoeuvres, never during normal navigation.
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_callback)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        rospy.wait_for_service("/move_base/clear_costmaps")
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        # Global planner's path query. Present as soon as move_base is up; if the
        # planner is not NavfnROS this lookup fails and scoring falls back to
        # straight-line distance.
        self.make_plan = None
        try:
            rospy.wait_for_service("/move_base/NavfnROS/make_plan", timeout=10.0)
            self.make_plan = rospy.ServiceProxy(
                "/move_base/NavfnROS/make_plan", GetPlan
            )
            rospy.loginfo("Path-length scoring enabled (NavfnROS/make_plan).")
        except rospy.ROSException:
            rospy.logwarn(
                "NavfnROS/make_plan unavailable - falling back to Euclidean scoring."
            )

    # ---------------------------------------------------------- Callbacks

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.pose_ready = True

    def clicked_point_callback(self, msg):
        """Register one RViz 'Publish Point' click as a flag."""
        if not self.collecting or len(self.flags) >= self.num_flags:
            return

        flag_id = len(self.flags) + 1
        self.flags.append({"id": flag_id, "x": msg.point.x, "y": msg.point.y})
        rospy.loginfo(
            "Flag %d set at (%.2f, %.2f)  [%d/%d]",
            flag_id,
            msg.point.x,
            msg.point.y,
            flag_id,
            self.num_flags,
        )
        self.publish_markers()

        if len(self.flags) == self.num_flags:
            rospy.loginfo("All flags collected. Starting navigation...")
            self.collecting = False
            self.remaining_flags = list(self.flags)

    # ---------------------------------------------------------- Markers

    def publish_markers(self, captured_ids=None):
        """Sphere + text label per flag; green once captured, red while pending."""
        captured_ids = captured_ids or set()
        array = MarkerArray()

        for flag in self.flags:
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "flags"
            sphere.id = flag["id"]
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = flag["x"]
            sphere.pose.position.y = flag["y"]
            sphere.pose.position.z = 0.2
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35
            if flag["id"] in captured_ids:
                sphere.color.g, sphere.color.a = 1.0, 1.0
            else:
                sphere.color.r, sphere.color.a = 1.0, 1.0
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = rospy.Time.now()
            label.ns = "flag_labels"
            label.id = flag["id"] + 100  # keep out of the sphere id range
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = flag["x"]
            label.pose.position.y = flag["y"]
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = "Flag {}".format(flag["id"])
            array.markers.append(label)

        self.marker_pub.publish(array)

    # ---------------------------------------------------------- Scoring

    @staticmethod
    def euclidean(x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def _stamped(x, y, frame="map"):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def path_length(self, goal_x, goal_y):
        """
        Length of the global planner's path from the robot to (goal_x, goal_y),
        or None when no valid plan exists.

        Two rejections matter here. An empty plan means the goal is unreachable
        from the current pose. A plan shorter than the straight-line distance is
        geometrically impossible, so it signals a stale transform or a truncated
        plan rather than a genuine shortcut; both are treated as "no plan".
        """
        if self.make_plan is None:
            return None

        try:
            response = self.make_plan(
                start=self._stamped(self.robot_x, self.robot_y),
                goal=self._stamped(goal_x, goal_y),
                tolerance=self.capture_radius,
            )
        except rospy.ServiceException as exc:
            rospy.logwarn("make_plan failed: %s", exc)
            return None

        poses = response.plan.poses
        if len(poses) < 2:
            return None

        length = 0.0
        for previous, current in zip(poses, poses[1:]):
            length += self.euclidean(
                previous.pose.position.x,
                previous.pose.position.y,
                current.pose.position.x,
                current.pose.position.y,
            )

        straight = self.euclidean(self.robot_x, self.robot_y, goal_x, goal_y)
        if length < straight * self.plan_sanity_ratio:
            rospy.logwarn(
                "Discarding implausible plan: %.2f m path vs %.2f m straight line.",
                length,
                straight,
            )
            return None

        return length

    def select_next_flag(self):
        """
        Pick the nearest remaining flag by navigable path length.

        Every candidate's cost is logged so the choice can be checked against the
        terminal output after a run. Flags with no valid plan are scored by
        straight-line distance and deprioritised, so an unreachable-looking flag
        is still attempted once everything else is done rather than dropped.
        """
        best = None
        best_cost = float("inf")

        for flag in self.remaining_flags:
            straight = self.euclidean(
                self.robot_x, self.robot_y, flag["x"], flag["y"]
            )
            path = self.path_length(flag["x"], flag["y"])

            if path is None:
                cost = straight + 1000.0  # sorts behind every reachable flag
                rospy.loginfo(
                    "  Flag %d: no plan (straight %.2f m) -> deprioritised",
                    flag["id"],
                    straight,
                )
            else:
                cost = path
                rospy.loginfo(
                    "  Flag %d: path %.2f m (straight %.2f m)",
                    flag["id"],
                    path,
                    straight,
                )

            if cost < best_cost:
                best_cost = cost
                best = flag

        return best, best_cost

    def get_approach_pose(self, flag):
        """
        Aim a short distance in front of the flag, on the robot's side of it.

        A flag clicked against a wall sits inside the costmap's inflation layer,
        where every trajectory scores as a collision and the local planner freezes.
        Backing the goal off by approach_offset puts it in free space while staying
        well inside capture_radius, so the flag still counts as reached.
        """
        dx = flag["x"] - self.robot_x
        dy = flag["y"] - self.robot_y
        distance = math.hypot(dx, dy)

        # Already inside the offset: aim at the flag itself, there is nothing to back off from.
        if distance <= self.approach_offset:
            return flag["x"], flag["y"]

        scale = (distance - self.approach_offset) / distance
        return self.robot_x + dx * scale, self.robot_y + dy * scale

    # ---------------------------------------------------------- Navigation

    def send_goal(self, flag):
        goal_x, goal_y = self.get_approach_pose(flag)

        goal = MoveBaseGoal()
        goal.target_pose = self._stamped(goal_x, goal_y)

        rospy.loginfo(
            ">>> Navigating to Flag %d at (%.2f, %.2f) via approach pose (%.2f, %.2f)",
            flag["id"],
            flag["x"],
            flag["y"],
            goal_x,
            goal_y,
        )
        self.client.send_goal(goal)

    def is_captured(self, flag):
        """
        True once the robot is inside capture_radius of the flag centre.

        Checked independently of move_base's own goal tolerance: the robot often
        passes through the capture zone before the action server reports SUCCEEDED,
        and there is no reason to keep driving after the flag is already taken.
        """
        return (
            self.euclidean(self.robot_x, self.robot_y, flag["x"], flag["y"])
            <= self.capture_radius
        )

    def mark_captured(self, flag, captured_ids):
        captured_ids.add(flag["id"])
        self.captured_order.append(flag["id"])
        self.remaining_flags = [
            f for f in self.remaining_flags if f["id"] != flag["id"]
        ]
        self.publish_markers(captured_ids=captured_ids)

        rospy.loginfo("=" * 55)
        rospy.loginfo("  FLAG %d CAPTURED", flag["id"])
        rospy.loginfo("  Location: (%.2f, %.2f)", flag["x"], flag["y"])
        rospy.loginfo("  Remaining: %d flag(s)", len(self.remaining_flags))
        rospy.loginfo("=" * 55)

    # ---------------------------------------------------------- Recovery

    def stop_robot(self):
        """Zero-velocity repeatedly; a single message can be missed on a lossy link."""
        for _ in range(5):
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.1)

    def _drive(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        end = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self.stop_robot()

    def do_recovery(self, attempt):
        """
        Escalating recovery, run after each ABORTED/REJECTED goal:

          1st  clear costmaps, rotate in place  (lightest touch)
          2nd  reverse out, rotate the other way
          3rd+ longer reverse, harder rotation, second costmap clear

        The active goal is cancelled first so these velocity commands are not
        fighting move_base for /cmd_vel.
        """
        rospy.logwarn(
            "[Recovery] attempt %d/%d", attempt, self.max_retries
        )
        self.client.cancel_goal()
        rospy.sleep(0.3)

        try:
            self.clear_costmaps()
            rospy.loginfo("[Recovery] Costmaps cleared.")
        except rospy.ServiceException as exc:
            rospy.logwarn("[Recovery] Could not clear costmaps: %s", exc)

        if attempt == 1:
            self._drive(angular=0.5, duration=2.5)
        elif attempt == 2:
            self._drive(linear=-0.1, duration=1.5)
            self._drive(angular=-0.5, duration=2.5)
        else:
            self._drive(linear=-0.1, duration=2.5)
            self._drive(angular=0.8, duration=3.0)
            try:
                self.clear_costmaps()
            except rospy.ServiceException:
                pass

        rospy.sleep(0.5)
        rospy.logwarn("[Recovery] Done, retrying goal.")

    # ---------------------------------------------------------- Main loop

    def run(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and not self.pose_ready:
            rospy.loginfo_throttle(3, "Waiting for robot pose on /amcl_pose...")
            rate.sleep()

        rospy.loginfo(
            "Pose acquired. Click %d points in RViz with the 'Publish Point' tool.",
            self.num_flags,
        )
        while not rospy.is_shutdown() and self.collecting:
            rospy.loginfo_throttle(
                3,
                "Waiting for flag clicks... ({}/{})".format(
                    len(self.flags), self.num_flags
                ),
            )
            rate.sleep()

        captured_ids = set()

        while not rospy.is_shutdown() and self.remaining_flags:
            rospy.loginfo(
                "--- Selecting next flag (%d remaining) ---",
                len(self.remaining_flags),
            )
            # Stale obstacles inflate path costs and can make a clear flag look
            # unreachable, so the costmaps are refreshed before every decision.
            try:
                self.clear_costmaps()
            except rospy.ServiceException as exc:
                rospy.logwarn("Could not clear costmaps before planning: %s", exc)

            target, cost = self.select_next_flag()
            if target is None:
                rospy.logwarn("No selectable flag remains.")
                break
            rospy.loginfo("-> Flag %d selected (cost %.2f m)", target["id"], cost)

            attempt = 0
            flag_done = False

            while not rospy.is_shutdown() and not flag_done:
                self.send_goal(target)

                # Poll until the flag is captured, move_base succeeds, or it aborts.
                while not rospy.is_shutdown():
                    if self.is_captured(target):
                        self.client.cancel_goal()
                        self.mark_captured(target, captured_ids)
                        flag_done = True
                        break

                    state = self.client.get_state()

                    if state == GoalStatus.SUCCEEDED:
                        self.mark_captured(target, captured_ids)
                        flag_done = True
                        break

                    if state in (GoalStatus.ABORTED, GoalStatus.REJECTED):
                        attempt += 1
                        if attempt > self.max_retries:
                            rospy.logwarn(
                                "Flag %d unreachable after %d recoveries. Skipping.",
                                target["id"],
                                self.max_retries,
                            )
                            self.remaining_flags = [
                                f
                                for f in self.remaining_flags
                                if f["id"] != target["id"]
                            ]
                            self.publish_markers(captured_ids=captured_ids)
                            flag_done = True
                        else:
                            self.do_recovery(attempt)
                        break  # re-send the goal, or move on

                    rate.sleep()

        rospy.loginfo("=" * 55)
        rospy.loginfo("  MISSION COMPLETE")
        rospy.loginfo(
            "  Capture order: %s",
            " -> ".join("F{}".format(i) for i in self.captured_order) or "(none)",
        )
        rospy.loginfo(
            "  Captured %d/%d flags", len(self.captured_order), self.num_flags
        )
        rospy.loginfo("=" * 55)


if __name__ == "__main__":
    try:
        MultiFlagNavigator().run()
    except rospy.ROSInterruptException:
        pass
