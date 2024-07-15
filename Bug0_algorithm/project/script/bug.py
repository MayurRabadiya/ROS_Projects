#!/usr/bin/env python

import math
import sys
import roslib
import rospy
import tf.transformations as transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from location import Location, necessary_heading
from dist import Dist
from std_srvs.srv import *

# Initialize global variables
current_location = Location()
current_dists = Dist()

# Constants
DELTA = 0.1
WALL_PADDING = 0.5

# Direction constants
STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3

# Target position parameters
tx = rospy.get_param('des_pos_x')
ty = rospy.get_param('des_pos_y')

def euclidean_distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def closest_in(points, x, y):
    """Find the closest point to (x, y) from a list of points."""
    ds = map(lambda p: euclidean_distance(p, (x, y)), points)
    return filter(lambda p: p[0] == min(ds), zip(ds, points))[0][1]

def init_listener():
    """Initialize ROS node and subscribers."""
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth', Odometry, location_callback)
    rospy.Subscriber('base_scan', LaserScan, lambda d: current_dists.update(d))

def location_callback(data):
    """Callback function for updating current location from Odometry data."""
    p = data.pose.pose.position
    q = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    )
    t = transform.euler_from_quaternion(q)[2]
    current_location.update_location(p.x, p.y, t)

class Bug:
    def __init__(self, tx, ty):
        """Initialize Bug class with target coordinates and default parameters."""
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.initial = (None, None)
        self.target = (tx, ty)
        self.battery = 100
        self.speed = 2
        self.state = "GO_UNTIL_OBSTACLE"
        self.states = {
            'GO_UNTIL_OBSTACLE': self.go_until_obstacle,
            'FOLLOW_WALL': self.follow_wall,
        }
        self.temp_loc = (None, None)
        self.prompted = False
        self.gopast10asked = True
        self.qat20asked = False

    def go_until_obstacle(self):
        """State function to move towards the target until an obstacle is encountered."""
        front, _ = current_dists.get()
        if front <= WALL_PADDING:
            return "FOLLOW_WALL"

        if current_location.facing_point(*self.target):
            self.go(STRAIGHT, self.speed)
        elif current_location.faster_left(*self.target):
            self.go(LEFT, self.speed)
        else:
            self.go(RIGHT, self.speed)
        return "GO_UNTIL_OBSTACLE"

    def follow_wall(self):
        """State function to follow the wall when an obstacle is encountered."""
        if current_dists.get()[0] <= WALL_PADDING:
            self.go(RIGHT, self.speed)
            return "FOLLOW_WALL"

        if not self.should_leave_wall():
            front, left = current_dists.get()
            if front <= WALL_PADDING:
                self.go(RIGHT, self.speed)
            elif WALL_PADDING - 0.1 <= left <= WALL_PADDING + 0.1:
                self.go(STRAIGHT, self.speed)
            elif left > WALL_PADDING + 0.1:
                self.go(LEFT, self.speed)
            else:
                self.go(RIGHT, self.speed)
            return "FOLLOW_WALL"
        else:
            return "GO_UNTIL_OBSTACLE"

    def goto_dest(self):
        """Navigate to the destination."""
        self.target = self.temp_loc
        self.battery = 100
        print("Returning to path to target charged.")

    def should_leave_wall(self):
        """Determine whether the robot should leave the wall and move towards the target."""
        x, y, t = current_location.current_location()
        g = current_location.global_to_local(necessary_heading(x, y, *self.target))
        at = current_dists.at(g)
        _, left = current_dists.get()
        return at > 10

    def go(self, direction, speed):
        """Publish movement commands to the robot."""
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = speed
        elif direction == LEFT:
            cmd.angular.z = 0.25
        elif direction == RIGHT:
            cmd.angular.z = -0.25
        elif direction == MSG_STOP:
            pass
        self.pub.publish(cmd)

    def step(self):
        """Execute one step of the state machine."""
        self.state = self.states[self.state]()
        x, y, _ = current_location.current_location()
        rospy.sleep(0.1)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: rosrun bugs bug.py X Y")
        sys.exit(1)

    print("Setting target:", (tx, ty))
    bug = Bug(tx, ty)
    init_listener()
    rospy.sleep(1)
    ix, iy, _ = current_location.current_location()
    bug.initial = (ix, iy)
    while current_location.distance(*bug.target) > DELTA:
        bug.step()
